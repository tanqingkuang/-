"""位置解算 Manager 的低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    FormCommInitS,
    FormPosS,
    FormSelfInitS,
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    PosInEarthS,
    PosTrackCommandE,
    RallyPhaseE,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityManagerInitS,
    EntityProcessSpecS,
    EntityRuntimeS,
)
from src.algorithm.units.algo.pos_calc import (
    PosCalcManager,
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


def _runtime() -> EntityRuntimeS:
    """构造绑定完整的实体运行环境。"""

    runtime = EntityRuntimeS()
    runtime.context.selfState = _motion()
    runtime.context.cmd = FormSnapshotS(stage=FormStageE.HOLD)
    runtime.context.wayLine = WayLineS(
        start=WayPointS(pos=PosInEarthS(0.0, 0.0, 500.0), vdCmd=20.0),
        end=WayPointS(pos=PosInEarthS(1000.0, 0.0, 500.0), vdCmd=20.0),
    )
    return runtime


def _entity_cfg(
    default_strategy: object,
    strategies: tuple[object, ...] = (),
    **kwargs: object,
) -> EntityManagerInitS:
    """构造仅配置位置解算流程的实体初始化参数。"""

    return EntityManagerInitS(
        entity=EntityInitS(**kwargs),
        process=EntityProcessSpecS(
            default_strategy=default_strategy,
            strategies=strategies,
        ),
    )


class PosCalcManagerTests(unittest.TestCase):
    """验证 Manager 的配置校验、缓存路由和统一输出。"""

    def test_rejects_duplicate_routes(self) -> None:
        """相同附加能力重复登记时应明确失败。"""

        manager = PosCalcManager()

        with self.assertRaises(ValueError):
            manager.init(
                _entity_cfg(
                    PosCalcStrategyE.ROUTE_INTERP,
                    (PosCalcStrategyE.RALLY_JOIN, PosCalcStrategyE.RALLY_JOIN),
                )
            )

    def test_rejects_noop_as_entity_default(self) -> None:
        """NOOP 是系统停控策略，不允许配置成实体常规飞行策略。"""

        manager = PosCalcManager()

        with self.assertRaises(ValueError):
            manager.init(_entity_cfg(PosCalcStrategyE.NOOP))

    def test_noop_writes_current_position_and_zero_velocity(self) -> None:
        """NONE 阶段应选择系统空策略并完整输出停控目标。"""

        runtime = _runtime()
        cxt = runtime.context
        cxt.cmd.stage = FormStageE.NONE
        cxt.selfState.pos = PosInEarthS(12.0, 34.0, 560.0)
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(_entity_cfg(PosCalcStrategyE.ROUTE_INTERP))

        manager.step()

        self.assertEqual(cxt.posCalcStatus.active_strategy, PosCalcStrategyE.NOOP)
        self.assertEqual(cxt.posTrackCommand.mode, PosTrackCommandE.NOOP)
        self.assertEqual(cxt.selfCmd.pos, cxt.selfState.pos)
        self.assertEqual(cxt.selfCmd.v, VdInEarthS())

    def test_rally_route_switches_cached_products_without_rebuilding(self) -> None:
        """STANDBY/JOINING 应选集结策略，HOLD 应返回同一个默认策略实例。"""

        runtime = _runtime()
        cxt = runtime.context
        cfg = _entity_cfg(
            PosCalcStrategyE.ROUTE_INTERP,
            (PosCalcStrategyE.RALLY_JOIN,),
            selfInit=FormSelfInitS("A01"),
            commInit=FormCommInitS(
                formPat=["wedge"],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
            ),
            route=_route(),
            rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
        )
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(cfg)
        rally_product = manager._registry[PosCalcStrategyE.RALLY_JOIN]
        route_product = manager._registry[PosCalcStrategyE.ROUTE_INTERP]

        cxt.cmd.stage = FormStageE.STANDBY
        manager.step()
        self.assertEqual(cxt.posCalcStatus.active_strategy, PosCalcStrategyE.RALLY_JOIN)
        self.assertEqual(cxt.posTrackCommand.mode, PosTrackCommandE.SPEED_TRACK)

        cxt.cmd.stage = FormStageE.RALLY
        cxt.cmd.step = RallyPhaseE.JOINING
        manager.step()
        self.assertEqual(cxt.posCalcStatus.active_strategy, PosCalcStrategyE.RALLY_JOIN)

        cxt.cmd.stage = FormStageE.HOLD
        manager.step()
        self.assertEqual(cxt.posCalcStatus.active_strategy, PosCalcStrategyE.ROUTE_INTERP)
        self.assertEqual(cxt.posTrackCommand.mode, PosTrackCommandE.SPEED_TRACK)
        self.assertIs(manager._registry[PosCalcStrategyE.RALLY_JOIN], rally_product)
        self.assertIs(manager._registry[PosCalcStrategyE.ROUTE_INTERP], route_product)

    def test_direct_hold_keeps_slot_transition_differentiator_enabled(self) -> None:
        """统一僚机直接进入 HOLD 时应保留原保持场景的槽位 TD。"""

        runtime = _runtime()
        cfg = _entity_cfg(
            PosCalcStrategyE.SLOT_GEOMETRY,
            (PosCalcStrategyE.RALLY_JOIN,),
            selfInit=FormSelfInitS("A02"),
            commInit=FormCommInitS(
                formPat=["wedge"],
                formPos=[[FormPosS("A02", -50.0, 0.0, 50.0)]],
            ),
            route=_route(),
            rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
            rally_enabled=False,
        )
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(cfg)

        slot_product = manager._registry[PosCalcStrategyE.SLOT_GEOMETRY]
        self.assertTrue(slot_product._td_enabled)  # type: ignore[attr-defined]

    def test_manager_writes_runtime_status_to_bound_output(self) -> None:
        """Manager 应原地更新绑定状态，供黑板其他单元读取上一拍结果。"""

        runtime = _runtime()
        cxt = runtime.context
        status = cxt.posCalcStatus
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                PosCalcStrategyE.ROUTE_INTERP,
                (PosCalcStrategyE.RALLY_JOIN,),
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
                route=_route(),
                rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
            )
        )
        cxt.cmd.stage = FormStageE.STANDBY

        manager.step()

        self.assertIs(cxt.posCalcStatus, status)
        self.assertEqual(status.active_strategy, PosCalcStrategyE.RALLY_JOIN)
        self.assertNotEqual(status.rally_state, "")
        self.assertGreaterEqual(status.planned_path_length_m, -1.0)

    def test_rally_product_reads_dynamic_values_from_blackboard_references(self) -> None:
        """集结产品应从黑板取得时钟、公共时刻和本机圈数，不要求 Entity 搬运标量。"""

        runtime = _runtime()
        cxt = runtime.context
        cxt.clock.now_s = 12.5
        cxt.rallyPlan.t_ref = 80.0
        cxt.rallyPlan.valid = True
        cxt.rallyPlan.loop_counts = {"A01": 2}
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                PosCalcStrategyE.ROUTE_INTERP,
                (PosCalcStrategyE.RALLY_JOIN,),
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
                route=_route(),
                rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
            )
        )

        cxt.cmd.stage = FormStageE.STANDBY
        manager.step()
        self.assertEqual(cxt.posCalcStatus.remaining_loops, 0)

        cxt.cmd.stage = FormStageE.RALLY
        cxt.cmd.step = RallyPhaseE.JOINING
        manager.step()
        self.assertEqual(cxt.posCalcStatus.remaining_loops, 2)


if __name__ == "__main__":
    unittest.main()
