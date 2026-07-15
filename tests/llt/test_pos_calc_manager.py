"""位置解算 Manager 的低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FormCommInitS,
    FormPosS,
    FormSelfInitS,
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    PosInEarthS,
    RallyPhaseE,
    RallyPlanS,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
)
from src.algorithm.entity.types import EntityInitS
from src.algorithm.units.algo.pos_calc import (
    PosCalcInputS,
    PosCalcManager,
    PosCalcOutputS,
    PosCalcStatusS,
    PosCalcStrategyE,
)
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS


def _motion(east: float = 0.0, north: float = 0.0, h: float = 500.0) -> MotionProfS:
    """构造测试运动状态。"""

    return MotionProfS(
        pos=PosInEarthS(east, north, h),
        v=VdInEarthS(vEast=20.0, vd=20.0),
    )


def _route() -> list[WayPointInputS]:
    """构造水平向东的两点航线。"""

    return [
        WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 500.0), vdCmd=20.0),
        WayPointInputS(idx=1, pos=PosInEarthS(1000.0, 0.0, 500.0), vdCmd=20.0),
    ]


def _ports() -> tuple[PosCalcInputS, PosCalcOutputS]:
    """构造绑定完整的统一输入输出端口。"""

    state = _motion()
    cmd = FormSnapshotS(stage=FormStageE.HOLD)
    line = WayLineS(
        start=WayPointS(pos=PosInEarthS(0.0, 0.0, 500.0), vdCmd=20.0),
        end=WayPointS(pos=PosInEarthS(1000.0, 0.0, 500.0), vdCmd=20.0),
    )
    return (
        PosCalcInputS(selfState=state, cmd=cmd, wayLine=line),
        PosCalcOutputS(selfCmd=MotionProfS(), status=PosCalcStatusS()),
    )


class PosCalcManagerTests(unittest.TestCase):
    """验证 Manager 的配置校验、缓存路由和统一输出。"""

    def test_rejects_duplicate_routes(self) -> None:
        """相同附加能力重复登记时应明确失败。"""

        u, y = _ports()
        manager = PosCalcManager()

        with self.assertRaises(ValueError):
            manager.init(
                EntityInitS(
                    pos_calc_default=PosCalcStrategyE.ROUTE_INTERP,
                    pos_calc_routes=(PosCalcStrategyE.RALLY_JOIN, PosCalcStrategyE.RALLY_JOIN),
                )
            )

    def test_rejects_noop_as_entity_default(self) -> None:
        """NOOP 是系统停控策略，不允许配置成实体常规飞行策略。"""

        u, y = _ports()
        manager = PosCalcManager()

        with self.assertRaises(ValueError):
            manager.init(EntityInitS(pos_calc_default=PosCalcStrategyE.NOOP))

    def test_noop_writes_current_position_and_zero_velocity(self) -> None:
        """NONE 阶段应选择系统空策略并完整输出停控目标。"""

        u, y = _ports()
        assert u.cmd is not None
        assert u.selfState is not None
        assert y.selfCmd is not None
        u.cmd.stage = FormStageE.NONE
        u.selfState.pos = PosInEarthS(12.0, 34.0, 560.0)
        manager = PosCalcManager()
        manager.init(EntityInitS(pos_calc_default=PosCalcStrategyE.ROUTE_INTERP))

        manager.step(u, y)

        assert y.status is not None
        self.assertEqual(y.status.active_strategy, PosCalcStrategyE.NOOP)
        self.assertEqual(y.selfCmd.pos, u.selfState.pos)
        self.assertEqual(y.selfCmd.v, VdInEarthS())

    def test_rally_route_switches_cached_products_without_rebuilding(self) -> None:
        """STANDBY/JOINING 应选集结策略，HOLD 应返回同一个默认策略实例。"""

        u, y = _ports()
        assert u.cmd is not None
        cfg = EntityInitS(
            selfInit=FormSelfInitS("A01"),
            commInit=FormCommInitS(
                formPat=["wedge"],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
            ),
            route=_route(),
            rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
            pos_calc_default=PosCalcStrategyE.ROUTE_INTERP,
            pos_calc_routes=(PosCalcStrategyE.RALLY_JOIN,),
        )
        manager = PosCalcManager()
        manager.init(cfg)
        rally_product = manager._registry[PosCalcStrategyE.RALLY_JOIN]
        route_product = manager._registry[PosCalcStrategyE.ROUTE_INTERP]

        u.cmd.stage = FormStageE.STANDBY
        manager.step(u, y)
        assert y.status is not None
        self.assertEqual(y.status.active_strategy, PosCalcStrategyE.RALLY_JOIN)

        u.cmd.stage = FormStageE.RALLY
        u.cmd.step = RallyPhaseE.JOINING
        manager.step(u, y)
        self.assertEqual(y.status.active_strategy, PosCalcStrategyE.RALLY_JOIN)

        u.cmd.stage = FormStageE.HOLD
        manager.step(u, y)
        self.assertEqual(y.status.active_strategy, PosCalcStrategyE.ROUTE_INTERP)
        self.assertIs(manager._registry[PosCalcStrategyE.RALLY_JOIN], rally_product)
        self.assertIs(manager._registry[PosCalcStrategyE.ROUTE_INTERP], route_product)

    def test_manager_writes_runtime_status_to_bound_output(self) -> None:
        """Manager 应原地更新绑定状态，供黑板其他单元读取上一拍结果。"""

        u, y = _ports()
        assert u.cmd is not None
        assert y.status is not None
        status = y.status
        manager = PosCalcManager()
        manager.init(
            EntityInitS(
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
                route=_route(),
                rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
                pos_calc_default=PosCalcStrategyE.ROUTE_INTERP,
                pos_calc_routes=(PosCalcStrategyE.RALLY_JOIN,),
            )
        )
        u.cmd.stage = FormStageE.STANDBY

        manager.step(u, y)

        self.assertIs(y.status, status)
        self.assertEqual(status.active_strategy, PosCalcStrategyE.RALLY_JOIN)
        self.assertNotEqual(status.rally_state, "")
        self.assertGreaterEqual(status.planned_path_length_m, -1.0)

    def test_rally_product_reads_dynamic_values_from_blackboard_references(self) -> None:
        """集结产品应从黑板取得时钟、公共时刻和本机圈数，不要求 Entity 搬运标量。"""

        u, y = _ports()
        assert u.cmd is not None
        u.clock = AlgorithmClockS(now_s=12.5)
        u.rallyPlan = RallyPlanS(t_ref=80.0, valid=True, loop_counts={"A01": 2})
        manager = PosCalcManager()
        manager.init(
            EntityInitS(
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
                route=_route(),
                rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
                pos_calc_default=PosCalcStrategyE.ROUTE_INTERP,
                pos_calc_routes=(PosCalcStrategyE.RALLY_JOIN,),
            )
        )

        u.cmd.stage = FormStageE.STANDBY
        manager.step(u, y)
        assert y.status is not None
        self.assertEqual(y.status.remaining_loops, 0)

        u.cmd.stage = FormStageE.RALLY
        u.cmd.step = RallyPhaseE.JOINING
        manager.step(u, y)
        self.assertEqual(y.status.remaining_loops, 2)


if __name__ == "__main__":
    unittest.main()
