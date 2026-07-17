"""位置跟踪 Manager 的低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormStageE,
    MotionProfS,
    PosInEarthS,
    PosTrackCommandE,
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


def _runtime(command: PosTrackCommandE) -> EntityRuntimeS:
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
    runtime.context.posTrackCommand.mode = command
    return runtime


def _entity_cfg(profile: EntityProfileS = LEADER_PROFILE) -> EntityManagerInitS:
    """构造由完整 Profile 驱动的位置跟踪初始化参数。"""

    return EntityManagerInitS(
        entity=EntityInitS(),
        profile=profile,
    )


class PosTrackManagerTests(unittest.TestCase):
    """验证显式配置、固定映射和缓存产品。"""

    def test_init_creates_only_products_used_by_profile_table(self) -> None:
        """长机与僚机产品集合应分别从完整表的 pos_track 列去重得到。"""

        leader = PosTrackManager()
        follower = PosTrackManager()
        leader.bind(_runtime(PosTrackCommandE.NOOP))
        follower.bind(_runtime(PosTrackCommandE.NOOP))
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

    def test_stage_step_selects_cached_product_instead_of_pos_calc_command(self) -> None:
        """运行期应查完整表，不能继续按 PosCalc 控制命令选择产品。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.POSITION_TRACK)
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
        runtime = _runtime(PosTrackCommandE.SPEED_TRACK)
        runtime.context.cmd.stage = FormStageE.RECONFIG
        runtime.context.cmd.step = RallyPhaseE.JOINING
        manager.bind(runtime)
        manager.init(_entity_cfg())

        with self.assertRaisesRegex(ValueError, "未配置"):
            manager.step()

    def test_noop_clears_control_output(self) -> None:
        """NOOP 应只清零加速度并保留既有诊断和 PosCalc 目标快照。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.NOOP)
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
