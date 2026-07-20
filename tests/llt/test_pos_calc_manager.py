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
    PosTrackStrategyE,
    RallyPhaseE,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityManagerInitS,
    EntityProfileE,
    EntityProfileS,
    EntityRouteChangeS,
    EntityRuntimeS,
    EntityStrategiesS,
)
from src.algorithm.entity.leader_follower import (
    FOLLOWER_PROFILE,
    LEADER_PROFILE,
    FORMATION_STATE_SEQUENCE,
)
from src.algorithm.units.algo.pos_calc import (
    PosCalcManager,
    PosCalcStrategyE,
)
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.tra_plan import TraPlanStrategyE


def _motion(east: float = 0.0, north: float = 0.0, h: float = 500.0) -> MotionProfS:
    """构造测试运动状态。"""

    return MotionProfS(
        pos=PosInEarthS(east, north, h),
        v=VdInEarthS(vEast=20.0, vd=20.0),
    )


def _route() -> list[WayPointInputS]:
    """构造水平向东的两点航线。"""

    return [
        WayPointInputS(pos=PosInEarthS(0.0, 0.0, 500.0), vdCmd=20.0),
        WayPointInputS(pos=PosInEarthS(1000.0, 0.0, 500.0), vdCmd=20.0),
    ]


def _runtime() -> EntityRuntimeS:
    """构造绑定完整的实体运行环境。"""

    runtime = EntityRuntimeS()
    runtime.context.selfState = _motion()
    runtime.context.cmd = FormSnapshotS(stage=FormStageE.HOLD)
    runtime.context.wayLine = WayLineS(
        start=PosInEarthS(0.0, 0.0, 500.0),
        end=PosInEarthS(1000.0, 0.0, 500.0),
        vdCmd=20.0,
    )
    return runtime


def _entity_cfg(
    profile: EntityProfileS = LEADER_PROFILE,
    **kwargs: object,
) -> EntityManagerInitS:
    """构造由完整 Profile 驱动的位置解算初始化参数。"""

    kwargs.setdefault("route", _route())
    kwargs.setdefault("rally_cfg", RallyTaskInitS(expectedFollowerIds=[]))
    return EntityManagerInitS(
        entity=EntityInitS(**kwargs),
        profile=profile,
    )


class PosCalcManagerTests(unittest.TestCase):
    """验证 Manager 的配置校验、缓存路由和统一输出。"""

    def test_profile_expands_change_points_into_complete_pos_calc_routes(self) -> None:
        """长机只填写三个变化点，初始化后应覆盖全部六个合法状态。"""

        expected = {
            (FormStageE.NONE, RallyPhaseE.JOINING): PosCalcStrategyE.NOOP,
            (FormStageE.STANDBY, RallyPhaseE.JOINING): PosCalcStrategyE.RALLY_JOIN,
            (FormStageE.RALLY, RallyPhaseE.JOINING): PosCalcStrategyE.RALLY_JOIN,
            (FormStageE.RALLY, RallyPhaseE.CATCHUP): PosCalcStrategyE.ROUTE_INTERP,
            (FormStageE.RALLY, RallyPhaseE.LOOSE): PosCalcStrategyE.ROUTE_INTERP,
            (FormStageE.HOLD, RallyPhaseE.JOINING): PosCalcStrategyE.ROUTE_INTERP,
        }

        self.assertEqual(
            {
                state: strategies.pos_calc
                for state, strategies in LEADER_PROFILE.route_table.items()
            },
            expected,
        )

    def test_manager_creates_only_pos_calc_products_used_by_profile_table(self) -> None:
        """实例集合应从完整路由表按列去重，不能再读取旧策略能力表。"""

        manager = PosCalcManager()
        manager.init(
            _entity_cfg(
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
            )
        )

        self.assertEqual(
            set(manager._registry),
            {
                PosCalcStrategyE.NOOP,
                PosCalcStrategyE.RALLY_JOIN,
                PosCalcStrategyE.ROUTE_INTERP,
            },
        )

    def test_rally_join_allows_multiple_later_pos_calc_products(self) -> None:
        """集结产品的角色初始化不应限制完整表只能出现一种普通位置策略。"""

        profile = EntityProfileS(
            identity=EntityProfileE.LEADER,
            state_sequence=FORMATION_STATE_SEQUENCE,
            route_changes=(
                EntityRouteChangeS(
                    (FormStageE.NONE, RallyPhaseE.JOINING),
                    EntityStrategiesS(
                        TraPlanStrategyE.NOOP,
                        PosCalcStrategyE.NOOP,
                        PosTrackStrategyE.NOOP,
                    ),
                ),
                EntityRouteChangeS(
                    (FormStageE.STANDBY, RallyPhaseE.JOINING),
                    EntityStrategiesS(
                        TraPlanStrategyE.NOOP,
                        PosCalcStrategyE.RALLY_JOIN,
                        PosTrackStrategyE.PID_SPEED,
                    ),
                ),
                EntityRouteChangeS(
                    (FormStageE.RALLY, RallyPhaseE.CATCHUP),
                    EntityStrategiesS(
                        TraPlanStrategyE.LEADER_ROUTE,
                        PosCalcStrategyE.ROUTE_INTERP,
                        PosTrackStrategyE.PID_SPEED,
                    ),
                ),
                EntityRouteChangeS(
                    (FormStageE.RALLY, RallyPhaseE.LOOSE),
                    EntityStrategiesS(
                        TraPlanStrategyE.NOOP,
                        PosCalcStrategyE.SLOT_GEOMETRY,
                        PosTrackStrategyE.PID_POSITION,
                    ),
                ),
            ),
        )
        manager = PosCalcManager()

        manager.init(
            _entity_cfg(
                profile,
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
            )
        )

        self.assertEqual(
            set(manager._registry),
            {
                PosCalcStrategyE.NOOP,
                PosCalcStrategyE.RALLY_JOIN,
                PosCalcStrategyE.ROUTE_INTERP,
                PosCalcStrategyE.SLOT_GEOMETRY,
            },
        )

    def test_unconfigured_stage_step_fails_without_default_strategy(self) -> None:
        """运行期遇到表外状态必须直接报错，不能退回角色默认策略。"""

        runtime = _runtime()
        runtime.context.cmd.stage = 99  # type: ignore[assignment]
        runtime.context.cmd.step = RallyPhaseE.JOINING
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
            )
        )

        with self.assertRaisesRegex(ValueError, "非法"):
            manager.step()

    def test_noop_writes_current_position_and_zero_velocity(self) -> None:
        """NONE 阶段应选择系统空策略并完整输出停控目标。"""

        runtime = _runtime()
        cxt = runtime.context
        cxt.cmd.stage = FormStageE.NONE
        cxt.selfState.pos = PosInEarthS(12.0, 34.0, 560.0)
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(_entity_cfg())

        manager.step()

        self.assertEqual(manager._active_strategy, PosCalcStrategyE.NOOP)
        self.assertEqual(cxt.selfCmd.pos, cxt.selfState.pos)
        self.assertEqual(cxt.selfCmd.v, VdInEarthS())

    def test_rally_route_switches_cached_products_without_rebuilding(self) -> None:
        """STANDBY/JOINING 应选集结策略，HOLD 应切回同一个航线策略实例。"""

        runtime = _runtime()
        cxt = runtime.context
        cfg = _entity_cfg(
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
        self.assertEqual(manager._active_strategy, PosCalcStrategyE.RALLY_JOIN)

        cxt.cmd.stage = FormStageE.RALLY
        cxt.cmd.step = RallyPhaseE.JOINING
        manager.step()
        self.assertEqual(manager._active_strategy, PosCalcStrategyE.RALLY_JOIN)

        cxt.cmd.stage = FormStageE.HOLD
        manager.step()
        self.assertEqual(manager._active_strategy, PosCalcStrategyE.ROUTE_INTERP)
        self.assertIs(manager._registry[PosCalcStrategyE.RALLY_JOIN], rally_product)
        self.assertIs(manager._registry[PosCalcStrategyE.ROUTE_INTERP], route_product)

    def test_direct_hold_keeps_slot_transition_differentiator_enabled(self) -> None:
        """统一僚机直接进入 HOLD 时应保留原保持场景的槽位 TD。"""

        runtime = _runtime()
        cfg = _entity_cfg(
            FOLLOWER_PROFILE,
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
        self.assertNotIn(PosCalcStrategyE.RALLY_JOIN, manager._registry)
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
        self.assertEqual(manager._active_strategy, PosCalcStrategyE.RALLY_JOIN)
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
        rally = manager._registry[PosCalcStrategyE.RALLY_JOIN]
        self.assertEqual(rally.remaining_loops, 0)  # type: ignore[attr-defined]

        cxt.cmd.stage = FormStageE.RALLY
        cxt.cmd.step = RallyPhaseE.JOINING
        manager.step()
        self.assertEqual(rally.remaining_loops, 2)  # type: ignore[attr-defined]

    def test_all_products_bind_private_ports_without_holding_context(self) -> None:
        """所有位置解算产品应在 bind 时绑定所需字段，不得在 step 中持有完整黑板。"""

        runtime = _runtime()
        cxt = runtime.context
        manager = PosCalcManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                selfInit=FormSelfInitS("A01"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0)]],
                ),
                route=_route(),
                rally_cfg=RallyTaskInitS(expectedFollowerIds=[]),
            )
        )

        noop = manager._registry[PosCalcStrategyE.NOOP]
        route = manager._registry[PosCalcStrategyE.ROUTE_INTERP]
        rally = manager._registry[PosCalcStrategyE.RALLY_JOIN]
        for product in manager._registry.values():
            self.assertFalse(hasattr(product, "_cxt"))
            self.assertIs(product._y.selfCmd, cxt.selfCmd)  # type: ignore[attr-defined]
            self.assertIs(product._y.status, cxt.posCalcStatus)  # type: ignore[attr-defined]
        self.assertIs(noop._u.selfState, cxt.selfState)  # type: ignore[attr-defined]
        self.assertIs(route._u.selfState, cxt.selfState)  # type: ignore[attr-defined]
        self.assertIs(route._u.wayLine, cxt.wayLine)  # type: ignore[attr-defined]
        self.assertIs(route._u.nextWayLine, cxt.nextWayLine)  # type: ignore[attr-defined]
        self.assertIs(rally._u.selfState, cxt.selfState)  # type: ignore[attr-defined]
        self.assertIs(rally._u.cmd, cxt.cmd)  # type: ignore[attr-defined]
        self.assertIs(rally._u.clock, cxt.clock)  # type: ignore[attr-defined]
        self.assertIs(rally._u.rallyPlan, cxt.rallyPlan)  # type: ignore[attr-defined]

        follower_runtime = _runtime()
        follower_cxt = follower_runtime.context
        follower_manager = PosCalcManager()
        follower_manager.bind(follower_runtime)
        follower_manager.init(
            _entity_cfg(
                FOLLOWER_PROFILE,
                selfInit=FormSelfInitS("A02"),
                commInit=FormCommInitS(
                    formPat=["wedge"],
                    formPos=[[FormPosS("A02", -50.0, 0.0, 50.0)]],
                ),
            )
        )
        slot = follower_manager._registry[PosCalcStrategyE.SLOT_GEOMETRY]
        self.assertFalse(hasattr(slot, "_cxt"))
        self.assertIs(slot._u.selfState, follower_cxt.selfState)  # type: ignore[attr-defined]
        self.assertIs(slot._u.leaderState, follower_cxt.leaderState)  # type: ignore[attr-defined]
        self.assertIs(slot._u.leaderCmd, follower_cxt.leaderCmd)  # type: ignore[attr-defined]
        self.assertIs(slot._u.cmd, follower_cxt.cmd)  # type: ignore[attr-defined]
        self.assertIs(slot._y.selfCmd, follower_cxt.selfCmd)  # type: ignore[attr-defined]


if __name__ == "__main__":
    unittest.main()
