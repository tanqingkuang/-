"""轨迹规划策略管理器低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    PosInEarthS,
    RallyPhaseE,
    WayLineS,
    WayPointInputS,
)
from src.algorithm.entity.leader_follower_hold.leader import (
    waypoint_inputs_to_waylines as legacy_waypoint_inputs_to_waylines,
)
from src.algorithm.entity.types import EntityInitS
from src.algorithm.units.process.tra_plan import (
    TraPlanInputS,
    TraPlanManager,
    TraPlanOutputS,
    TraPlanStrategyE,
)
from src.algorithm.units.process.tra_plan.leader_route import waypoint_inputs_to_waylines


def _route() -> list[WayPointInputS]:
    """构造两航段任务航线。"""
    return [
        WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=20.0),
        WayPointInputS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=20.0),
        WayPointInputS(idx=2, pos=PosInEarthS(200.0, 0.0, 1000.0), vdCmd=20.0),
    ]


class TraPlanManagerTests(unittest.TestCase):
    """验证显式配置、阶段路由和有状态产品缓存。"""

    def test_init_rejects_incomplete_strategy_table(self) -> None:
        """验证缺省、重复和默认产品未登记的配置均在初始化期失败。"""
        cases = (
            (EntityInitS(), "tra_plan_default 必须是 TraPlanStrategyE"),
            (
                EntityInitS(
                    tra_plan_default=TraPlanStrategyE.NOOP,
                    tra_plan_strategies=(TraPlanStrategyE.NOOP, TraPlanStrategyE.NOOP),
                ),
                "不得包含重复策略",
            ),
            (
                EntityInitS(
                    route=_route(),
                    tra_plan_default=TraPlanStrategyE.LEADER_ROUTE,
                    tra_plan_strategies=(TraPlanStrategyE.NOOP,),
                ),
                "必须包含在 tra_plan_strategies",
            ),
        )

        for cfg, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                TraPlanManager().init(cfg)

    def test_route_conversion_matches_legacy_hold_implementation(self) -> None:
        """验证新旧架构并存期间两份航点转换实现保持完全等价。"""
        route = [
            WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=20.0),
            WayPointInputS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=20.0, r=20.0),
            WayPointInputS(idx=2, pos=PosInEarthS(100.0, 100.0, 1000.0), vdCmd=20.0),
        ]

        self.assertEqual(
            waypoint_inputs_to_waylines(route),
            legacy_waypoint_inputs_to_waylines(route),
        )

    def test_init_requires_explicit_noop_product(self) -> None:
        """验证长机配置不能依赖 Manager 隐式补充 NOOP。"""
        manager = TraPlanManager()

        with self.assertRaisesRegex(ValueError, "显式包含 NOOP"):
            manager.init(
                EntityInitS(
                    route=_route(),
                    tra_plan_default=TraPlanStrategyE.LEADER_ROUTE,
                    tra_plan_strategies=(TraPlanStrategyE.LEADER_ROUTE,),
                )
            )

    def test_noop_only_configuration_supports_follower(self) -> None:
        """验证僚机显式只配置 NOOP 时可在任意正常阶段执行。"""
        manager = TraPlanManager()
        manager.init(
            EntityInitS(
                tra_plan_default=TraPlanStrategyE.NOOP,
                tra_plan_strategies=(TraPlanStrategyE.NOOP,),
            )
        )
        way_line = WayLineS(idx=9)

        manager.step(
            TraPlanInputS(cmd=FormSnapshotS(stage=FormStageE.HOLD), wayLine=way_line),
            TraPlanOutputS(wayLine=way_line),
        )

        self.assertEqual(way_line.idx, 9)

    def test_leader_routes_by_cmd_and_preserves_cached_route_state(self) -> None:
        """验证待命不推进航段，恢复任务航线后继续使用同一产品状态。"""
        manager = TraPlanManager()
        manager.init(
            EntityInitS(
                route=_route(),
                tra_plan_default=TraPlanStrategyE.LEADER_ROUTE,
                tra_plan_strategies=(TraPlanStrategyE.NOOP, TraPlanStrategyE.LEADER_ROUTE),
            )
        )
        product_ids = {strategy: id(product) for strategy, product in manager._registry.items()}
        cmd = FormSnapshotS(stage=FormStageE.STANDBY, step=RallyPhaseE.JOINING)
        self_state = MotionProfS(pos=PosInEarthS(120.0, 0.0, 1000.0))
        way_line = WayLineS(idx=9)
        u = TraPlanInputS(cmd=cmd, wayLine=way_line, selfState=self_state)
        y = TraPlanOutputS(wayLine=way_line)

        manager.step(u, y)
        self.assertEqual(way_line.idx, 9)

        cmd.stage = FormStageE.RALLY
        cmd.step = RallyPhaseE.CATCHUP
        manager.step(u, y)
        self.assertEqual(way_line.idx, 1)

        cmd.stage = FormStageE.STANDBY
        manager.step(u, y)
        cmd.stage = FormStageE.HOLD
        self_state.pos.east = 10.0
        manager.step(u, y)

        self.assertEqual(way_line.idx, 1)
        self.assertEqual(
            {strategy: id(product) for strategy, product in manager._registry.items()},
            product_ids,
        )

        manager.reset()
        manager.step(u, y)
        self.assertEqual(way_line.idx, 0)


if __name__ == "__main__":
    unittest.main()
