"""轨迹规划策略管理器。注意：产品只在初始化时创建，运行期仅选择缓存对象。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import WayLineS
from src.algorithm.units.process.tra_plan.base import (
    TraPlanBase,
    TraPlanInitS,
    TraPlanStrategyE,
)
from src.algorithm.units.process.tra_plan.leader_route import (
    LeaderRoute,
    LeaderRouteInitS,
    waypoint_inputs_to_waylines,
)
from src.algorithm.units.process.tra_plan.noop import Noop

if TYPE_CHECKING:
    from src.algorithm.entity.types import (
        EntityInitS,
        EntityManagerInitS,
        EntityProfileS,
        EntityRuntimeS,
    )


class TraPlanManager:
    """创建、缓存并路由轨迹规划策略。注意：Entity 不应访问具体产品对象。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：必须先调用 init 才能执行 step。"""
        self._registry: dict[TraPlanStrategyE, TraPlanBase] = {}
        self._profile: EntityProfileS | None = None
        self._cmd = None
        self._binding_runtime: EntityRuntimeS | None = None

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定路由命令及产品运行环境。"""
        self._cmd = runtime.context.cmd
        self._binding_runtime = runtime

    def init(self, cfg: EntityManagerInitS) -> None:
        """按完整路由表创建全部轨迹规划产品。"""
        profile = cfg.profile
        strategies = {
            _require_strategy(item.tra_plan, "route_table.tra_plan")
            for item in profile.route_table.values()
        }
        # 产品只在初始化阶段创建一次，运行期路由只查表切换引用。
        # LeaderRoute 保存跨帧航段索引，因此绝不能在阶段切换时重新构造。
        # registry 保存产品对象而非类，切换策略不会丢失各自内部状态。
        self._registry = {
            strategy: self._create_strategy(strategy, cfg.entity) for strategy in strategies
        }
        if self._binding_runtime is None:
            raise ValueError("TraPlanManager 尚未绑定运行环境")
        for strategy in self._registry.values():
            strategy.bind(self._binding_runtime)
        self._binding_runtime = None
        self._profile = profile

    def step(self) -> None:
        """严格查询完整表并推进缓存产品。注意：本方法不创建产品。"""
        if self._cmd is None:
            raise ValueError("TraPlanManager 尚未绑定命令端口")
        profile = self._profile
        if profile is None:
            raise ValueError("TraPlanManager 尚未初始化")
        strategy_type = _require_strategy(
            profile.require_strategies(self._cmd.stage, self._cmd.step).tra_plan,
            "route_table.tra_plan",
        )
        strategy = self._registry.get(strategy_type)
        if strategy is None:
            raise ValueError(f"轨迹规划策略未配置: {strategy_type.name}")
        strategy.step()

    def reset(self) -> None:
        """复位全部缓存产品。注意：保留配置和产品实例。"""
        # 每个缓存产品都可能持有独立跨帧状态，必须全部复位。
        for strategy in self._registry.values():
            strategy.reset()

    def get_route(self) -> list[WayLineS]:
        """返回长机任务航线，供 Runner 初始显示。注意：未配置时返回空列表。"""
        # 这是显示适配接口，不参与 step 的策略路由。
        strategy = self._registry.get(TraPlanStrategyE.LEADER_ROUTE)
        return strategy.get_route() if isinstance(strategy, LeaderRoute) else []

    @staticmethod
    def _create_strategy(strategy_type: TraPlanStrategyE, cfg: EntityInitS) -> TraPlanBase:
        """创建并初始化单个产品。注意：只允许由 init 调用。"""
        # 建造逻辑集中在 Manager 内，Entity 只感知枚举和统一端口。
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
