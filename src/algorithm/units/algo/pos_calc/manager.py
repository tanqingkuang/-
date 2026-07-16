"""位置解算策略管理器。注意：策略只在初始化时创建，运行期仅切换缓存对象引用。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    FormStageE,
    PosInEarthS,
    RallyPhaseE,
)
from src.algorithm.units.algo.pos_calc.base import (
    PosCalcBase,
    PosCalcInitS,
    PosCalcStrategyE,
)
from src.algorithm.units.algo.pos_calc.noop import NoopPosCalc
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
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
    from src.algorithm.entity.types import EntityInitS, EntityManagerInitS, EntityRuntimeS, VelCmdLimitS


_LEADER_L1_DISTANCE_M = 0.0  # 长机航线目标不使用额外 L1 前视距离
_LEADER_FF_LEAD_TIME_S = 0.5  # 长机速度前馈时间
_SLOT_TD_VMAX_LATERAL_DEFAULT = 6.0  # 槽位横侧向默认速度权限
_SLOT_TD_VMAX_FORWARD_FALLBACK = 5.0  # 前向权限无法推导时的兜底值
_SLOT_TD_VMAX_VERTICAL_FALLBACK = 3.0  # 垂向权限无法推导时的兜底值


class PosCalcManager:
    """创建、缓存并路由位置解算策略。注意：Entity 不应访问具体策略对象。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：产品只在 init 时创建，运行期不重建。"""
        self._default_strategy = PosCalcStrategyE.NOOP  # 非特殊阶段使用的角色默认产品
        self._routes: tuple[PosCalcStrategyE, ...] = ()  # 该实体额外启用的阶段能力
        self._registry: dict[PosCalcStrategyE, PosCalcBase] = {}  # 已初始化产品缓存
        self._active_strategy: PosCalcStrategyE | None = None  # 上一拍执行产品，用于边沿复位
        self._cxt: FormContextS | None = None

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：Manager只转交黑板，不创建统一输入输出端口。"""
        # Manager只保存选择策略所需的黑板引用，具体字段访问留给子策略。
        self._cxt = runtime.context

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
        # NOOP 始终作为系统停控能力创建，不要求 Profile 重复声明。
        # required 使用集合去重，默认策略与附加能力重合时也只创建一个实例。
        # 有状态产品的生命周期与 Entity 相同，不能在 step 中临时构造。
        required = {PosCalcStrategyE.NOOP, default_strategy, *routes}
        self._registry = {
            strategy: self._create_strategy(strategy, entity_cfg) for strategy in required
        }
        if self._cxt is not None:
            # 每个产品自行建立专属输入输出快照，Manager不参与端口构造。
            for strategy in self._registry.values():
                strategy.bind(self._cxt)
        self._active_strategy = None

    def step(self) -> None:
        """选择并调用缓存策略。注意：所有黑板读写均由具体子类完成。"""
        if self._cxt is None:
            raise ValueError("PosCalcManager 尚未绑定黑板")
        cmd = self._cxt.cmd
        # cmd 是唯一运行期路由依据，Entity 不参与具体产品判断。
        # 选择过程只返回枚举，算法输入由产品随后从黑板自行冻结。
        # registry 缺项属于初始化配置错误，不允许静默退回默认策略。
        strategy_type = self._select_strategy(cmd.stage, cmd.step)
        # 进入停控阶段时只复位一次集结产品，避免每拍清空刚生成的停控输出。
        if strategy_type == PosCalcStrategyE.NOOP and self._active_strategy != PosCalcStrategyE.NOOP:
            rally_join = self._registry.get(PosCalcStrategyE.RALLY_JOIN)
            if rally_join is not None:
                rally_join.reset()
        strategy = self._registry[strategy_type]
        # 子策略完成快照读取、内部计算和黑板提交的完整事务。
        strategy.step()
        self._active_strategy = strategy_type

    def reset(self) -> None:
        """复位全部缓存策略。注意：保留初始化配置、实例和黑板绑定。"""
        # reset 清动态状态但保留产品实例和初始化参数。
        for strategy in self._registry.values():
            strategy.reset()
        self._active_strategy = None

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
            strategy = NoopPosCalc()
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
            if cfg.rally_enabled:  # 集结任务保留 CATCHUP 分层高度且不启用重构 TD
                # 集结期间槽位由状态机连续压缩，不能再叠加普通保持的 TD 过渡。
                # 分层高度只在 CATCHUP 生效，进入 LOOSE 后恢复编队槽位高度。
                init_cfg = SlotGeometryInitS(
                    cfg.selfInit.id,
                    cfg.commInit.formPat,
                    cfg.commInit.formPos,
                    catchupAltitudeM=cfg.rally_layer_altitude_m,
                )
            else:  # 直接 HOLD 的统一实体仍按速度权限配置原槽位重构 TD
                # 普通保持需要 TD 软化初始槽位误差以及运行期队形重构阶跃。
                # 速度上界从本机权限推导，保证参考轨迹给反馈控制保留余量。
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
