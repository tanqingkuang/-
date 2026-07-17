"""轨迹规划策略管理器低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    FormStageE,
    PosInEarthS,
    RallyPhaseE,
    WayLineS,
    WayPointInputS,
    copy_wayline,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityManagerInitS,
    EntityProcessSpecS,
    EntityRuntimeS,
)
from src.algorithm.units.process.tra_plan import (
    TraPlanManager,
    TraPlanStrategyE,
)


def _route() -> list[WayPointInputS]:
    """构造两航段任务航线。"""
    return [
        WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=20.0),
        WayPointInputS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=20.0),
        WayPointInputS(idx=2, pos=PosInEarthS(200.0, 0.0, 1000.0), vdCmd=20.0),
    ]


def _entity_cfg(
    default_strategy: object,
    strategies: tuple[object, ...],
    **kwargs: object,
) -> EntityManagerInitS:
    """构造仅配置轨迹规划流程的实体初始化参数。"""

    return EntityManagerInitS(
        entity=EntityInitS(**kwargs),
        process=EntityProcessSpecS(
            default_strategy=default_strategy,
            strategies=strategies,
        ),
    )


class TraPlanManagerTests(unittest.TestCase):
    """验证显式配置、阶段路由和有状态产品缓存。"""

    def test_init_rejects_incomplete_strategy_table(self) -> None:
        """验证缺省、重复和默认产品未登记的配置均在初始化期失败。"""
        cases = (
            (_entity_cfg(None, ()), "tra_plan.default_strategy 必须是 TraPlanStrategyE"),
            (
                _entity_cfg(
                    TraPlanStrategyE.NOOP,
                    (TraPlanStrategyE.NOOP, TraPlanStrategyE.NOOP),
                ),
                "不得包含重复策略",
            ),
            (
                _entity_cfg(
                    TraPlanStrategyE.LEADER_ROUTE,
                    (TraPlanStrategyE.NOOP,),
                    route=_route(),
                ),
                "必须包含在 processes.tra_plan.strategies",
            ),
        )

        for cfg, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                TraPlanManager().init(cfg)

    def test_init_requires_explicit_noop_product(self) -> None:
        """验证长机配置不能依赖 Manager 隐式补充 NOOP。"""
        manager = TraPlanManager()

        with self.assertRaisesRegex(ValueError, "显式包含 NOOP"):
            manager.init(
                _entity_cfg(
                    TraPlanStrategyE.LEADER_ROUTE,
                    (TraPlanStrategyE.LEADER_ROUTE,),
                    route=_route(),
                )
            )

    def test_noop_only_configuration_supports_follower(self) -> None:
        """验证僚机显式只配置 NOOP 时可在任意正常阶段执行。"""
        runtime = EntityRuntimeS()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.wayLine.idx = 9
        manager = TraPlanManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(TraPlanStrategyE.NOOP, (TraPlanStrategyE.NOOP,))
        )

        manager.step()

        self.assertEqual(runtime.context.wayLine.idx, 9)

    def test_leader_routes_by_cmd_and_preserves_cached_route_state(self) -> None:
        """验证待命不推进航段，恢复任务航线后继续使用同一产品状态。"""
        runtime = EntityRuntimeS()
        manager = TraPlanManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                TraPlanStrategyE.LEADER_ROUTE,
                (TraPlanStrategyE.NOOP, TraPlanStrategyE.LEADER_ROUTE),
                route=_route(),
            )
        )
        product_ids = {strategy: id(product) for strategy, product in manager._registry.items()}
        cxt = runtime.context
        cxt.cmd.stage = FormStageE.STANDBY
        cxt.cmd.step = RallyPhaseE.JOINING
        cxt.selfState.pos = PosInEarthS(120.0, 0.0, 1000.0)
        copy_wayline(WayLineS(idx=9), cxt.wayLine)

        manager.step()
        self.assertEqual(cxt.wayLine.idx, 9)

        cxt.cmd.stage = FormStageE.RALLY
        cxt.cmd.step = RallyPhaseE.CATCHUP
        manager.step()
        self.assertEqual(cxt.wayLine.idx, 1)

        cxt.cmd.stage = FormStageE.STANDBY
        manager.step()
        cxt.cmd.stage = FormStageE.HOLD
        cxt.selfState.pos.east = 10.0
        manager.step()

        self.assertEqual(cxt.wayLine.idx, 1)
        self.assertEqual(
            {strategy: id(product) for strategy, product in manager._registry.items()},
            product_ids,
        )

        manager.reset()
        manager.step()
        self.assertEqual(cxt.wayLine.idx, 0)


if __name__ == "__main__":
    unittest.main()
