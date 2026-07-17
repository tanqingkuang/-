"""位置解算策略管理器。注意：策略只在初始化时创建，运行期仅切换缓存对象引用。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    PosInEarthS,
)
from src.algorithm.entity.types import EntityProfileE
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
    from src.algorithm.entity.types import (
        EntityInitS,
        EntityManagerInitS,
        EntityProfileS,
        EntityRuntimeS,
        VelCmdLimitS,
    )


_LEADER_L1_DISTANCE_M = 0.0  # 长机航线目标不使用额外 L1 前视距离
_LEADER_FF_LEAD_TIME_S = 0.5  # 长机速度前馈时间
_SLOT_TD_VMAX_LATERAL_DEFAULT = 6.0  # 槽位横侧向默认速度权限
_SLOT_TD_VMAX_FORWARD_FALLBACK = 5.0  # 前向权限无法推导时的兜底值
_SLOT_TD_VMAX_VERTICAL_FALLBACK = 3.0  # 垂向权限无法推导时的兜底值


class PosCalcManager:
    """创建、缓存并路由位置解算策略。注意：Entity 不应访问具体策略对象。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：产品只在 init 时创建，运行期不重建。"""
        self._registry: dict[PosCalcStrategyE, PosCalcBase] = {}  # 已初始化产品缓存
        self._active_strategy: PosCalcStrategyE | None = None  # 上一拍执行产品，用于边沿复位
        self._profile: EntityProfileS | None = None  # 完整状态路由表
        self._cmd = None
        self._binding_cxt: FormContextS | None = None

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：Manager只转交黑板，不创建统一输入输出端口。"""
        # 完整黑板仅用于随后给产品绑定端口；运行期只保留策略选择所需的 cmd。
        self._cmd = runtime.context.cmd
        self._binding_cxt = runtime.context

    def init(self, cfg: EntityManagerInitS) -> None:
        """按完整路由表建造并缓存策略。注意：不读取旧流程策略规格。"""
        entity_cfg = cfg.entity
        profile = cfg.profile
        if profile is None:
            raise ValueError("PosCalcManager 初始化必须提供 EntityProfile")
        # 实例集合直接从完整表的 pos_calc 列去重，表中未出现的产品不会创建。
        required = {
            _require_strategy(strategies.pos_calc, "route_table.pos_calc")
            for strategies in profile.route_table.values()
        }
        if not entity_cfg.rally_enabled:
            # 直接 HOLD 不会进入 Profile 中的集结状态，不创建或校验集结专用产品。
            # Profile 描述身份的完整能力上限，实例开关描述本次任务实际启用的子集。
            # 保留同一 Profile 可让普通 leader/wingman 与集结角色共享实体工厂。
            # Rally 任务在禁用时只会输出 NONE/HOLD，因此运行期不会请求被移除的产品。
            # 在建造前剔除尤为重要，否则未使用产品仍会校验盘旋半径和速度范围。
            # 其他产品仍严格来自完整表，不能在这里补充任何隐式默认策略。
            required.discard(PosCalcStrategyE.RALLY_JOIN)
        self._registry = {
            strategy: self._create_strategy(strategy, entity_cfg, profile.identity)
            for strategy in required
        }
        if self._binding_cxt is not None:
            # 每个产品自行绑定专属输入输出端口，Manager不参与端口构造。
            for strategy in self._registry.values():
                strategy.bind(self._binding_cxt)
            self._binding_cxt = None
        self._profile = profile
        self._active_strategy = None

    def step(self) -> None:
        """选择并调用缓存策略。注意：所有黑板读写均由具体子类完成。"""
        if self._cmd is None:
            raise ValueError("PosCalcManager 尚未绑定命令端口")
        cmd = self._cmd
        profile = self._profile
        if profile is None:
            raise ValueError("PosCalcManager 尚未初始化")
        # 运行期只做严格表查询，不再判断阶段、不设默认策略或兜底策略。
        strategy_type = _require_strategy(
            profile.require_strategies(cmd.stage, cmd.step).pos_calc,
            "route_table.pos_calc",
        )
        # 进入停控阶段时只复位一次集结产品，避免每拍清空刚生成的停控输出。
        if strategy_type == PosCalcStrategyE.NOOP and self._active_strategy != PosCalcStrategyE.NOOP:
            rally_join = self._registry.get(PosCalcStrategyE.RALLY_JOIN)
            if rally_join is not None:
                rally_join.reset()
        strategy = self._registry.get(strategy_type)
        if strategy is None:
            raise ValueError(f"位置解算策略未初始化: {strategy_type.name}")
        # 子策略通过初始化期绑定的专属端口直接读写黑板字段。
        strategy.step()
        self._active_strategy = strategy_type

    def reset(self) -> None:
        """复位全部缓存策略。注意：保留初始化配置、实例和黑板绑定。"""
        # reset 清动态状态但保留产品实例和初始化参数。
        for strategy in self._registry.values():
            strategy.reset()
        self._active_strategy = None

    def _create_strategy(
        self,
        strategy_type: PosCalcStrategyE,
        cfg: EntityInitS,
        identity: EntityProfileE,
    ) -> PosCalcBase:
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
                # 集结期间直接使用最终槽位，不能再叠加普通保持的 TD 过渡。
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
            strategy.init(_rally_join_init(cfg, identity))
            return strategy
        raise ValueError(f"不支持的位置解算策略: {strategy_type!r}")


def _require_strategy(value: object, field_name: str) -> PosCalcStrategyE:
    """校验并返回策略枚举。注意：禁止把普通整数静默当作枚举。"""
    if not isinstance(value, PosCalcStrategyE):
        raise ValueError(f"{field_name} 必须是 PosCalcStrategyE")
    return value


def _rally_join_init(cfg: EntityInitS, identity: EntityProfileE) -> RallyJoinPosInitS:
    """由实体公共配置生成集结位置解算参数。注意：角色由静态实体身份决定。"""
    if len(cfg.route) < 2:
        raise ValueError("RALLY_JOIN: route 至少需要两个航点")
    rally_cfg = cfg.rally_cfg
    if not isinstance(rally_cfg, RallyTaskInitS):
        raise ValueError("RALLY_JOIN: rally_cfg must be RallyTaskInitS")

    route_start = cfg.route[0].pos  # 统一航线起点同时作为长机松散点基准
    heading = route_heading_rad(cfg.route)
    if identity == EntityProfileE.LEADER:
        # 长机槽位位于航线起点，只按分层配置覆盖高度。
        loose_slot = PosInEarthS(route_start.east, route_start.north, route_start.h)
    elif identity == EntityProfileE.FOLLOWER:
        # 僚机先按目标队形找槽位，再按松散比例旋转到任务航迹系。
        slot = resolve_formation_slot(cfg.commInit, rally_cfg.targetPattern, cfg.selfInit.id)
        if slot is None:
            raise ValueError(
                f"RALLY_JOIN: 节点 {cfg.selfInit.id!r} 在目标队形 {rally_cfg.targetPattern!r} "
                "的槽位表中未找到对应条目（目标队形不在 formPat 中，或 formPos 缺少该队形/该节点）"
            )
        loose_slot = rally_loose_target(route_start, heading, rally_cfg.looseScale, slot)
    else:
        raise ValueError(f"RALLY_JOIN 不支持的实体身份: {identity!r}")
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
