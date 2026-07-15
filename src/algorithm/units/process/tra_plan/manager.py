"""轨迹规划策略管理器。注意：产品只在初始化时创建，运行期仅选择缓存对象。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import FormStageE, RallyPhaseE, WayLineS
from src.algorithm.units.process.tra_plan.base import (
    TraPlanBase,
    TraPlanInitS,
    TraPlanInputS,
    TraPlanOutputS,
    TraPlanStrategyE,
)
from src.algorithm.units.process.tra_plan.leader_route import (
    LeaderRoute,
    LeaderRouteInitS,
    waypoint_inputs_to_waylines,
)
from src.algorithm.units.process.tra_plan.noop import Noop

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityInitS


class TraPlanManager:
    """创建、缓存并路由轨迹规划策略。注意：Entity 不应访问具体产品对象。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：必须先调用 init 才能执行 step。"""
        self._default_strategy: TraPlanStrategyE | None = None
        self._registry: dict[TraPlanStrategyE, TraPlanBase] = {}

    def init(self, cfg: EntityInitS) -> None:
        """按实体配置创建全部已声明产品。注意：不得隐式补充策略。"""
        default_strategy = _require_strategy(cfg.tra_plan_default, "tra_plan_default")
        strategies = tuple(
            _require_strategy(strategy, "tra_plan_strategies") for strategy in cfg.tra_plan_strategies
        )
        if not strategies:
            raise ValueError("tra_plan_strategies 不得为空")
        if len(strategies) != len(set(strategies)):
            raise ValueError("tra_plan_strategies 不得包含重复策略")
        if default_strategy not in strategies:
            raise ValueError("tra_plan_default 必须包含在 tra_plan_strategies 中")
        if TraPlanStrategyE.NOOP not in strategies:
            raise ValueError("tra_plan_strategies 必须显式包含 NOOP")

        self._default_strategy = default_strategy
        # 产品只在初始化阶段创建一次，运行期路由只查表切换引用。
        self._registry = {strategy: self._create_strategy(strategy, cfg) for strategy in strategies}

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        """按任务指令选择缓存产品并推进一拍。注意：本方法不创建产品。"""
        if u.cmd is None:
            raise ValueError("TraPlanManager cmd port must be bound")
        strategy_type = self._select_strategy(u.cmd.stage, u.cmd.step)
        strategy = self._registry.get(strategy_type)
        if strategy is None:
            raise ValueError(f"轨迹规划策略未配置: {strategy_type.name}")
        strategy.step(u, y)

    def reset(self) -> None:
        """复位全部缓存产品。注意：保留配置和产品实例。"""
        for strategy in self._registry.values():
            strategy.reset()

    def get_route(self) -> list[WayLineS]:
        """返回长机任务航线，供 Runner 初始显示。注意：未配置时返回空列表。"""
        strategy = self._registry.get(TraPlanStrategyE.LEADER_ROUTE)
        return strategy.get_route() if isinstance(strategy, LeaderRoute) else []

    def _select_strategy(self, stage: FormStageE, step: int) -> TraPlanStrategyE:
        """由任务指令选择策略。注意：任务阶段映射属于 TraPlan 内部语义。"""
        if stage in (FormStageE.NONE, FormStageE.STANDBY):
            return TraPlanStrategyE.NOOP
        if stage == FormStageE.RALLY and step == RallyPhaseE.JOINING:
            return TraPlanStrategyE.NOOP
        if self._default_strategy is None:
            raise ValueError("TraPlanManager 尚未初始化")
        return self._default_strategy

    @staticmethod
    def _create_strategy(strategy_type: TraPlanStrategyE, cfg: EntityInitS) -> TraPlanBase:
        """创建并初始化单个产品。注意：只允许由 init 调用。"""
        if strategy_type == TraPlanStrategyE.NOOP:
            strategy = Noop()
            strategy.init(TraPlanInitS())
            return strategy
        if strategy_type == TraPlanStrategyE.LEADER_ROUTE:
            strategy = LeaderRoute()
            strategy.init(LeaderRouteInitS(waypoint_inputs_to_waylines(cfg.route)))
            return strategy
        raise ValueError(f"不支持的轨迹规划策略: {strategy_type!r}")


def _require_strategy(value: object, field_name: str) -> TraPlanStrategyE:
    """校验策略枚举。注意：禁止普通整数静默充当枚举。"""
    if not isinstance(value, TraPlanStrategyE):
        raise ValueError(f"{field_name} 必须是 TraPlanStrategyE")
    return value
