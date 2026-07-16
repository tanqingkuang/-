"""位置解算策略管理器。注意：策略只在初始化时创建，运行期仅切换缓存对象引用。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    FormStageE,
    PosInEarthS,
    PosTrackCommandE,
    RallyPhaseE,
    copy_position,
    zero_velocity,
)
from src.algorithm.units.algo.pos_calc.base import (
    PosCalcBase,
    PosCalcInitS,
    PosCalcInputS,
    PosCalcOutputS,
    PosCalcStrategyE,
)
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RallyJoinPos,
    RallyJoinPosInitS,
    loiter_speed_bounds,
    rally_loose_target,
    resolve_formation_slot,
    route_heading_rad,
)
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityInitS, EntityManagerInitS, VelCmdLimitS


_LEADER_L1_DISTANCE_M = 0.0  # 长机航线目标不使用额外 L1 前视距离
_LEADER_FF_LEAD_TIME_S = 0.5  # 长机速度前馈时间
_SLOT_TD_VMAX_LATERAL_DEFAULT = 6.0  # 槽位横侧向默认速度权限
_SLOT_TD_VMAX_FORWARD_FALLBACK = 5.0  # 前向权限无法推导时的兜底值
_SLOT_TD_VMAX_VERTICAL_FALLBACK = 3.0  # 垂向权限无法推导时的兜底值
_POS_TRACK_COMMAND_BY_STRATEGY = {
    PosCalcStrategyE.NOOP: PosTrackCommandE.NOOP,
    PosCalcStrategyE.RALLY_JOIN: PosTrackCommandE.SPEED_TRACK,
    PosCalcStrategyE.ROUTE_INTERP: PosTrackCommandE.SPEED_TRACK,
    PosCalcStrategyE.SLOT_GEOMETRY: PosTrackCommandE.POSITION_TRACK,
}


class _NoopPosCalc(PosCalcBase):
    """NONE 阶段位置解算空策略。注意：完整写出当前位置和零速度。"""

    def init(self, cfg: PosCalcInitS) -> None:
        """初始化空策略。注意：空策略没有构造参数和动态资源。"""
        return None

    def step(self, u: PosCalcInputS, y: PosCalcOutputS) -> None:
        """输出当前位置和零速度。注意：输入输出端口必须完成绑定。"""
        if u.selfState is None or y.selfCmd is None:
            raise ValueError("NoopPosCalc ports must be bound")
        copy_position(u.selfState.pos, y.selfCmd.pos)
        zero_velocity(y.selfCmd.v)

    def reset(self) -> None:
        """复位空策略。注意：空策略没有运行期状态。"""
        return None


class PosCalcManager:
    """创建、缓存并路由位置解算策略。注意：Entity 不应访问具体策略对象。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：产品只在 init 时创建，端口由每拍 step 显式传入。"""
        self._default_strategy = PosCalcStrategyE.NOOP  # 非特殊阶段使用的角色默认产品
        self._routes: tuple[PosCalcStrategyE, ...] = ()  # 该实体额外启用的阶段能力
        self._registry: dict[PosCalcStrategyE, PosCalcBase] = {}  # 已初始化产品缓存
        self._active_strategy: PosCalcStrategyE | None = None  # 上一拍执行产品，用于边沿复位

    def init(self, cfg: EntityManagerInitS) -> None:
        """按实体身份证建造并缓存策略。注意：运行期不得再次调用。"""
        # 配置边界只接受枚举，防止普通整数绕过策略语义。
        entity_cfg = cfg.entity
        process_spec = cfg.process
        default_strategy = _require_strategy(process_spec.default_strategy, "pos_calc.default_strategy")
        routes = tuple(_require_strategy(item, "pos_calc.strategies") for item in process_spec.strategies)
        # 同一能力重复登记通常意味着配置表拼接错误，应在初始化期暴露。
        if len(routes) != len(set(routes)):
            raise ValueError("processes.pos_calc.strategies 不得包含重复策略")
        # 当前阶段路由只定义了 RALLY_JOIN，其他产品只能作为角色默认策略。
        unsupported_routes = set(routes) - {PosCalcStrategyE.RALLY_JOIN}
        if unsupported_routes:
            names = ", ".join(sorted(item.name for item in unsupported_routes))
            raise ValueError(f"processes.pos_calc.strategies 包含不支持的附加能力: {names}")
        # NOOP 是系统保底，RALLY_JOIN 是附加阶段能力，均不能充当常规飞行策略。
        if default_strategy in (PosCalcStrategyE.NOOP, PosCalcStrategyE.RALLY_JOIN):
            raise ValueError(
                "processes.pos_calc.default_strategy 只允许 ROUTE_INTERP 或 SLOT_GEOMETRY"
            )

        self._default_strategy = default_strategy
        self._routes = routes
        # 产品只在此处创建一次，运行期路由只切换缓存引用。
        required = {PosCalcStrategyE.NOOP, default_strategy, *routes}
        self._registry = {
            strategy: self._create_strategy(strategy, entity_cfg) for strategy in required
        }
        self._active_strategy = None

    def step(self, u: PosCalcInputS, y: PosCalcOutputS) -> None:
        """按当前 cmd 选择缓存策略并推进一拍。注意：本方法不创建或重新初始化策略。"""
        if u.cmd is None:
            raise ValueError("PosCalcManager cmd port must be bound")
        # cmd 是唯一运行期路由依据，Entity 不参与具体产品判断。
        strategy_type = self._select_strategy(u.cmd.stage, u.cmd.step)
        # 进入停控阶段时只复位一次集结产品，避免每拍清空刚生成的停控输出。
        if strategy_type == PosCalcStrategyE.NOOP and self._active_strategy != PosCalcStrategyE.NOOP:
            rally_join = self._registry.get(PosCalcStrategyE.RALLY_JOIN)
            if rally_join is not None:
                rally_join.reset()
        # 所有产品共享同一套端口，切换产品不改变黑板对象引用。
        self._registry[strategy_type].step(u, y)
        self._write_pos_track_command(strategy_type, y)
        self._active_strategy = strategy_type
        self._write_status(y)

    def reset(self) -> None:
        """复位全部缓存策略。注意：保留初始化配置和统一端口绑定。"""
        # reset 清动态状态但保留产品实例和初始化参数。
        for strategy in self._registry.values():
            strategy.reset()
        self._active_strategy = None

    def _write_status(self, y: PosCalcOutputS) -> None:
        """原地更新统一状态输出。注意：保持黑板绑定对象的引用不变。"""
        status = y.status
        if status is None:
            return
        status.active_strategy = self._active_strategy  # 先发布通用策略诊断
        rally_join = self._registry.get(PosCalcStrategyE.RALLY_JOIN)
        # 未装配集结产品时显式写默认值，不能遗留上一轮实体状态。
        if not isinstance(rally_join, RallyJoinPos):
            status.rally_state = ""
            status.planned_path_length_m = -1.0
            status.remaining_path_length_m = -1.0
            status.remaining_loops = 0
            status.reached_slot_once = False
            status.join_exited = False
            return
        # 集结细节只在 Manager 内部读取具体产品，再统一投影到黑板状态。
        status.rally_state = rally_join.state
        status.planned_path_length_m = rally_join.planned_path_length_m
        status.remaining_path_length_m = rally_join.remaining_path_length_m
        status.remaining_loops = rally_join.remaining_loops
        status.reached_slot_once = rally_join.reached_slot_once
        status.join_exited = rally_join.state == RALLY_STATE_EXITED

    @staticmethod
    def _write_pos_track_command(strategy_type: PosCalcStrategyE, y: PosCalcOutputS) -> None:
        """发布与本拍位置解算产品对应的控制命令。注意：不泄漏具体控制器类型。"""
        if y.posTrackCommand is None:
            return
        y.posTrackCommand.mode = _POS_TRACK_COMMAND_BY_STRATEGY[strategy_type]

    def _select_strategy(self, stage: FormStageE, step: int) -> PosCalcStrategyE:
        """由任务指令选择策略枚举。注意：集结子状态路由不向 Entity 配置层暴露。"""
        if stage == FormStageE.NONE:  # NONE 对所有角色强制选择停控产品
            return PosCalcStrategyE.NOOP
        if PosCalcStrategyE.RALLY_JOIN in self._routes:
            # 待命和正式 JOINING 共用同一个有状态集结产品，保证圆几何连续。
            if stage == FormStageE.STANDBY:
                return PosCalcStrategyE.RALLY_JOIN
            if stage == FormStageE.RALLY and step == RallyPhaseE.JOINING:
                return PosCalcStrategyE.RALLY_JOIN
        return self._default_strategy

    def _create_strategy(self, strategy_type: PosCalcStrategyE, cfg: EntityInitS) -> PosCalcBase:
        """创建并初始化单个产品。注意：只允许由 init 调用一次。"""
        if strategy_type == PosCalcStrategyE.NOOP:  # 系统停控产品无需业务配置
            strategy = _NoopPosCalc()
            strategy.init(PosCalcInitS())
            return strategy
        if strategy_type == PosCalcStrategyE.ROUTE_INTERP:  # 长机任务航线解算产品
            strategy = RouteInterp()
            strategy.init(
                RouteInterpInitS(
                    lookAheadDistance=_LEADER_L1_DISTANCE_M,
                    leadTimeS=_LEADER_FF_LEAD_TIME_S,
                )
            )
            return strategy
        if strategy_type == PosCalcStrategyE.SLOT_GEOMETRY:  # 僚机队形槽位解算产品
            strategy = SlotGeometry()
            rally_enabled = PosCalcStrategyE.RALLY_JOIN in self._routes
            if rally_enabled:  # 集结实体保留 CATCHUP 分层高度且不启用重构 TD
                init_cfg = SlotGeometryInitS(
                    cfg.selfInit.id,
                    cfg.commInit.formPat,
                    cfg.commInit.formPos,
                    catchupAltitudeM=cfg.rally_layer_altitude_m,
                )
            else:  # 普通保持实体按速度权限配置槽位重构 TD
                v_fwd, v_up, v_lat = _slot_td_vmax(cfg.velCmdLimit)
                init_cfg = SlotGeometryInitS(
                    cfg.selfInit.id,
                    cfg.commInit.formPat,
                    cfg.commInit.formPos,
                    control_period_s=cfg.control_period_s,
                    vMaxForward=v_fwd,
                    vMaxVertical=v_up,
                    vMaxLateral=v_lat,
                )
            strategy.init(init_cfg)
            return strategy
        if strategy_type == PosCalcStrategyE.RALLY_JOIN:  # 待命到切出的有状态集结产品
            strategy = RallyJoinPos()
            strategy.init(_rally_join_init(cfg, self._default_strategy))
            return strategy
        raise ValueError(f"不支持的位置解算策略: {strategy_type!r}")


def _require_strategy(value: object, field_name: str) -> PosCalcStrategyE:
    """校验并返回策略枚举。注意：禁止把普通整数静默当作枚举。"""
    if not isinstance(value, PosCalcStrategyE):
        raise ValueError(f"{field_name} 必须是 PosCalcStrategyE")
    return value


def _rally_join_init(cfg: EntityInitS, default_strategy: PosCalcStrategyE) -> RallyJoinPosInitS:
    """由实体公共配置生成集结位置解算初始化参数。注意：角色差异由默认策略推导。"""
    if len(cfg.route) < 2:
        raise ValueError("RALLY_JOIN: route 至少需要两个航点")
    rally_cfg = cfg.rally_cfg
    if not isinstance(rally_cfg, RallyTaskInitS):
        raise ValueError("RALLY_JOIN: rally_cfg must be RallyTaskInitS")

    route_start = cfg.route[0].pos  # 统一航线起点同时作为长机松散点基准
    heading = route_heading_rad(cfg.route)
    if default_strategy == PosCalcStrategyE.ROUTE_INTERP:
        # 长机槽位位于航线起点，只按分层配置覆盖高度。
        loose_slot = PosInEarthS(route_start.east, route_start.north, route_start.h)
    elif default_strategy == PosCalcStrategyE.SLOT_GEOMETRY:
        # 僚机先按目标队形找槽位，再按松散比例旋转到任务航迹系。
        slot = resolve_formation_slot(cfg.commInit, rally_cfg.targetPattern, cfg.selfInit.id)
        if slot is None:
            raise ValueError(
                f"RALLY_JOIN: 节点 {cfg.selfInit.id!r} 在目标队形 {rally_cfg.targetPattern!r} "
                "的槽位表中未找到对应条目（目标队形不在 formPat 中，或 formPos 缺少该队形/该节点）"
            )
        loose_slot = rally_loose_target(route_start, heading, rally_cfg.looseScale, slot)
    else:
        raise ValueError("RALLY_JOIN 需要 ROUTE_INTERP 或 SLOT_GEOMETRY 作为默认策略")
    if cfg.rally_layer_altitude_m is not None:  # JOINING 阶段使用防碰撞分层高度
        loose_slot.h = cfg.rally_layer_altitude_m

    loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)  # 水平权限决定协调调速区间
    slow_radius_m = max(rally_cfg.arrival_radius_m * 3.0, 60.0)
    v_up_min = cfg.velCmdLimit.verticalMin if math.isfinite(cfg.velCmdLimit.verticalMin) else -3.0  # 下降权限
    v_up_max = cfg.velCmdLimit.verticalMax if math.isfinite(cfg.velCmdLimit.verticalMax) else 3.0  # 爬升权限
    return RallyJoinPosInitS(
        self_id=cfg.selfInit.id,
        loose_slot=loose_slot,
        approach_speed_mps=cfg.rally_approach_speed_mps,
        slow_radius_m=slow_radius_m,
        arrival_radius_m=rally_cfg.arrival_radius_m,
        loiter_radius_m=rally_cfg.loiter_radius_m,
        loiter_speed_min_mps=loiter_min,
        loiter_speed_max_mps=loiter_max,
        mission_heading_rad=heading,
        mission_speed_mps=cfg.rally_approach_speed_mps,
        v_up_min_mps=v_up_min,
        v_up_max_mps=v_up_max,
        control_period_s=cfg.control_period_s,
        standby_altitude_m=cfg.rally_layer_altitude_m,
    )


def _slot_td_vmax(vel_limit: VelCmdLimitS) -> tuple[float, float, float]:
    """由速度权限推导槽位 TD 三轴参考速度上界。注意：数值保持原保持僚机装配语义。"""
    if math.isfinite(vel_limit.forwardMin) and math.isfinite(vel_limit.forwardMax):
        # 前向 TD 使用可用速度区间的一半，避免参考轨迹顶到固定翼速度边界。
        v_fwd = 0.5 * (vel_limit.forwardMax - vel_limit.forwardMin)
    else:
        v_fwd = _SLOT_TD_VMAX_FORWARD_FALLBACK
    # 垂向 TD 取上下行权限较小者，保证对称重构轨迹两侧均可执行。
    v_up = min(abs(vel_limit.verticalMin), abs(vel_limit.verticalMax))
    if not math.isfinite(v_up):
        v_up = _SLOT_TD_VMAX_VERTICAL_FALLBACK
    return 0.8 * v_fwd, 0.8 * v_up, 0.8 * _SLOT_TD_VMAX_LATERAL_DEFAULT
