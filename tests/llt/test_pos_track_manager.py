"""位置跟踪 Manager 的低层测试。"""

from __future__ import annotations

import math
import unittest
from dataclasses import replace

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormStageE,
    MotionProfS,
    PosInEarthS,
    RallyPhaseE,
    VdInEarthS,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityManagerInitS,
    EntityProfileS,
    EntityRuntimeS,
)
from src.algorithm.entity.leader_follower import (
    FOLLOWER_PROFILE,
    LEADER_PROFILE,
)
from src.algorithm.units.algo.pos_track import (
    PosTrackManager,
    PosTrackStrategyE,
)


def _runtime() -> EntityRuntimeS:
    """构造完整绑定的位置跟踪运行环境。"""

    runtime = EntityRuntimeS()
    runtime.context.selfState = MotionProfS(
        pos=PosInEarthS(0.0, 0.0, 500.0),
        v=VdInEarthS(vEast=10.0, vd=10.0),
    )
    runtime.context.selfCmd = MotionProfS(
        pos=PosInEarthS(100.0, 0.0, 500.0),
        v=VdInEarthS(vEast=10.0, vd=10.0),
    )
    return runtime


def _entity_cfg(profile: EntityProfileS = LEADER_PROFILE) -> EntityManagerInitS:
    """构造由完整 Profile 驱动的位置跟踪初始化参数。"""

    return EntityManagerInitS(
        entity=EntityInitS(),
        profile=profile,
    )


def _follower_profile(strategy: PosTrackStrategyE) -> EntityProfileS:
    """复制僚机策略表并仅替换编队飞行阶段的位置跟踪产品。"""
    return replace(
        FOLLOWER_PROFILE,
        route_changes=tuple(
            replace(
                change,
                strategies=replace(
                    change.strategies,
                    pos_track=strategy,
                ),
            )
            if change.strategies.pos_track == PosTrackStrategyE.PID_POSITION
            else change
            for change in FOLLOWER_PROFILE.route_changes
        ),
    )


class PosTrackManagerTests(unittest.TestCase):
    """验证显式配置、固定映射和缓存产品。"""

    def test_init_creates_only_products_used_by_profile_table(self) -> None:
        """长机与僚机产品集合应分别从完整表的 pos_track 列去重得到。"""

        leader = PosTrackManager()
        follower = PosTrackManager()
        leader.bind(_runtime())
        follower.bind(_runtime())
        leader.init(_entity_cfg())
        follower.init(_entity_cfg(FOLLOWER_PROFILE))

        self.assertEqual(
            set(leader._registry),
            {PosTrackStrategyE.NOOP, PosTrackStrategyE.PID_SPEED},
        )
        self.assertEqual(
            set(follower._registry),
            {
                PosTrackStrategyE.NOOP,
                PosTrackStrategyE.PID_SPEED,
                PosTrackStrategyE.PID_POSITION,
            },
        )

    def test_follower_profile_removes_turn_transport_and_centripetal_feedforward(self) -> None:
        """无非惯性补偿产品应移除槽位运输速度和向心前馈，但不得改写位置解算原始指令。"""

        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.cmd.step = RallyPhaseE.JOINING
        runtime.context.leaderState = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0, dVPsi=0.1),
        )
        runtime.context.leaderCmd = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0, dVPsi=0.1),
        )
        runtime.context.selfState = MotionProfS(
            pos=PosInEarthS(40.0, -30.0, 500.0),
            v=VdInEarthS(vEast=21.0, vNorth=-2.0, vd=(21.0**2 + 2.0**2) ** 0.5),
        )
        # r=(40,-30)，omega=0.1 时 omega×r=(3,4)；另保留 TD 重构速度 (1,-2)。
        runtime.context.selfCmd = MotionProfS(
            pos=PosInEarthS(40.0, -30.0, 500.0),
            v=VdInEarthS(
                vEast=24.0,
                vNorth=2.0,
                vd=(24.0**2 + 2.0**2) ** 0.5,
                dVPsi=0.1,
            ),
        )
        manager = PosTrackManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                _follower_profile(PosTrackStrategyE.PID_POSITION_NO_INERTIAL)
            )
        )

        manager.step()

        self.assertAlmostEqual(runtime.context.selfAccCmd.accEast, 0.0)
        self.assertAlmostEqual(runtime.context.selfAccCmd.accNorth, 0.0)
        self.assertAlmostEqual(runtime.context.effectiveCmd.v.vEast, 21.0)
        self.assertAlmostEqual(runtime.context.effectiveCmd.v.vNorth, -2.0)
        self.assertAlmostEqual(runtime.context.effectiveCmd.v.dVPsi, 0.0)
        self.assertAlmostEqual(runtime.posTrackDiag.cmd_vel_east_mps, 21.0)
        self.assertAlmostEqual(runtime.posTrackDiag.cmd_vel_north_mps, -2.0)
        # 供其他模块读取的 PosCalc 原始目标不能被消融产品原地修改。
        self.assertAlmostEqual(runtime.context.selfCmd.v.vEast, 24.0)
        self.assertAlmostEqual(runtime.context.selfCmd.v.vNorth, 2.0)
        self.assertAlmostEqual(runtime.context.selfCmd.v.dVPsi, 0.1)

    def test_full_inertial_product_adds_leader_translational_acceleration(self) -> None:
        """直线加速时完整产品应直接叠加长机平动加速度，现有位置反馈误差保持为零。"""

        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.cmd.step = RallyPhaseE.JOINING
        runtime.context.selfState = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.selfCmd = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderState = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderAccCmd = AccInEarthS(2.0, 0.0, 0.0)
        manager = PosTrackManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                _follower_profile(PosTrackStrategyE.PID_POSITION_FULL_INERTIAL)
            )
        )

        manager.step()

        self.assertAlmostEqual(runtime.context.selfAccCmd.accEast, 2.0)
        self.assertAlmostEqual(runtime.context.selfAccCmd.accNorth, 0.0)

    def test_full_inertial_product_limits_total_forward_acceleration(self) -> None:
        """完整前馈与反馈叠加后的前向总加速度不得突破既有控制限幅。"""

        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.cmd.step = RallyPhaseE.JOINING
        runtime.context.selfState = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.selfCmd = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderState = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderAccCmd = AccInEarthS(20.0, 0.0, 0.0)
        manager = PosTrackManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                _follower_profile(PosTrackStrategyE.PID_POSITION_FULL_INERTIAL)
            )
        )

        manager.step()

        self.assertAlmostEqual(runtime.context.selfAccCmd.accEast, 6.0)

    def test_full_inertial_product_adds_angular_acceleration_and_rotating_slot_terms(self) -> None:
        """变转率时完整产品应补出 alpha×r 和 omega×(omega×r)。"""

        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.cmd.step = RallyPhaseE.JOINING
        runtime.context.selfState = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.selfCmd = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderState = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        manager = PosTrackManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                _follower_profile(PosTrackStrategyE.PID_POSITION_FULL_INERTIAL)
            )
        )
        runtime.context.leaderClock.now_s = 0.0
        manager.step()

        runtime.context.leaderClock.now_s = 1.0
        runtime.context.selfCmd.v.dVPsi = 0.1
        manager.step()

        self.assertAlmostEqual(runtime.context.selfAccCmd.accEast, -1.0)
        self.assertAlmostEqual(
            runtime.context.selfAccCmd.accNorth,
            9.80665 * math.tan(math.radians(40.0)),
        )

    def test_full_inertial_alpha_uses_elapsed_time_between_new_yaw_rate_samples(self) -> None:
        """重复读取旧转率时不得缩短角加速度差分时间。"""

        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.HOLD
        runtime.context.cmd.step = RallyPhaseE.JOINING
        runtime.context.selfState = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.selfCmd = MotionProfS(
            pos=PosInEarthS(100.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        runtime.context.leaderState = MotionProfS(
            pos=PosInEarthS(0.0, 0.0, 500.0),
            v=VdInEarthS(vEast=20.0, vd=20.0),
        )
        manager = PosTrackManager()
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                _follower_profile(PosTrackStrategyE.PID_POSITION_FULL_INERTIAL)
            )
        )
        for now_s in (0.0, 1.0, 2.0, 3.0):
            # 本机时钟推进，但没有新长机报文，leaderClock 保持旧采样时刻。
            runtime.context.clock.now_s = now_s
            manager.step()

        runtime.context.leaderClock.now_s = 4.0
        runtime.context.selfCmd.v.dVPsi = 0.1
        manager.step()

        # alpha=0.1/4，r=(100,0)，故 alpha×r 的北向分量为 2.5。
        self.assertAlmostEqual(runtime.context.selfAccCmd.accEast, -1.0)
        self.assertAlmostEqual(runtime.context.selfAccCmd.accNorth, 2.5)

    def test_stage_step_selects_cached_product_instead_of_pos_calc_command(self) -> None:
        """运行期应查完整表，不能继续按 PosCalc 控制命令选择产品。"""

        manager = PosTrackManager()
        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.STANDBY
        runtime.context.cmd.step = RallyPhaseE.JOINING
        manager.bind(runtime)
        manager.init(_entity_cfg())
        product_ids = {key: id(value) for key, value in manager._registry.items()}

        manager.step()
        speed_acc_east = runtime.context.selfAccCmd.accEast
        runtime.context.cmd.stage = FormStageE.NONE
        manager.step()

        self.assertAlmostEqual(speed_acc_east, 0.0)
        self.assertEqual(runtime.context.selfAccCmd, AccInEarthS())
        self.assertEqual(
            {key: id(value) for key, value in manager._registry.items()},
            product_ids,
        )

    def test_unconfigured_stage_step_fails_without_command_fallback(self) -> None:
        """运行期遇到表外状态必须失败，不能退回 PosCalc 控制命令。"""

        manager = PosTrackManager()
        runtime = _runtime()
        runtime.context.cmd.stage = 99  # type: ignore[assignment]
        runtime.context.cmd.step = RallyPhaseE.JOINING
        manager.bind(runtime)
        manager.init(_entity_cfg())

        with self.assertRaisesRegex(ValueError, "非法"):
            manager.step()

    def test_noop_clears_control_output(self) -> None:
        """NOOP 应只清零加速度并保留既有诊断和 PosCalc 目标快照。"""

        manager = PosTrackManager()
        runtime = _runtime()
        runtime.context.cmd.stage = FormStageE.NONE
        manager.bind(runtime)
        manager.init(_entity_cfg())
        runtime.context.selfAccCmd.accEast = 3.0
        runtime.posTrackDiag.cmd_pos_east_m = 8.0

        manager.step()

        self.assertEqual(runtime.context.selfAccCmd, AccInEarthS())
        self.assertEqual(runtime.posTrackDiag.cmd_pos_east_m, 8.0)
        self.assertEqual(runtime.context.effectiveCmd, runtime.context.selfCmd)

if __name__ == "__main__":
    unittest.main()
