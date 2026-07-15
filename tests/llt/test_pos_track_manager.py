"""位置跟踪 Manager 的低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    MotionProfS,
    PosInEarthS,
    PosTrackCommandE,
    PosTrackCommandS,
    PosTrackDiagS,
    VdInEarthS,
)
from src.algorithm.entity.leader_follower_hold.leader import (
    _default_tracker_init,
    _follower_tracker_init,
)
from src.algorithm.entity.types import EntityInitS, VelCmdLimitS
from src.algorithm.units.algo.pos_track import (
    PosTrackInputS,
    PosTrackManager,
    PosTrackOutputS,
    PosTrackStrategyE,
)
from src.algorithm.units.algo.pos_track.manager import _pid_position_init, _pid_speed_init


def _ports(command: PosTrackCommandE) -> tuple[PosTrackInputS, PosTrackOutputS]:
    """构造完整绑定的位置跟踪端口。"""

    self_state = MotionProfS(
        pos=PosInEarthS(0.0, 0.0, 500.0),
        v=VdInEarthS(vEast=10.0, vd=10.0),
    )
    self_cmd = MotionProfS(
        pos=PosInEarthS(100.0, 0.0, 500.0),
        v=VdInEarthS(vEast=10.0, vd=10.0),
    )
    return (
        PosTrackInputS(
            command=PosTrackCommandS(command),
            selfCmd=self_cmd,
            selfState=self_state,
        ),
        PosTrackOutputS(
            accCmd=AccInEarthS(),
            diag=PosTrackDiagS(),
            effectiveCmd=MotionProfS(),
        ),
    )


class PosTrackManagerTests(unittest.TestCase):
    """验证显式配置、固定映射和缓存产品。"""

    def test_init_rejects_incomplete_strategy_table(self) -> None:
        """空表、重复策略和缺少 NOOP 均应在初始化期失败。"""

        cases = (
            (EntityInitS(), "不得为空"),
            (
                EntityInitS(
                    pos_track_strategies=(PosTrackStrategyE.NOOP, PosTrackStrategyE.NOOP)
                ),
                "不得包含重复策略",
            ),
            (
                EntityInitS(pos_track_strategies=(PosTrackStrategyE.PID_SPEED,)),
                "必须显式包含 NOOP",
            ),
        )
        for cfg, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                PosTrackManager().init(cfg)

    def test_command_selects_cached_one_to_one_product(self) -> None:
        """运行期命令应选择固定策略，且切换前后不重建有状态产品。"""

        manager = PosTrackManager()
        manager.init(
            EntityInitS(
                pos_track_strategies=(
                    PosTrackStrategyE.NOOP,
                    PosTrackStrategyE.PID_SPEED,
                    PosTrackStrategyE.PID_POSITION,
                )
            )
        )
        product_ids = {key: id(value) for key, value in manager._registry.items()}

        speed_u, speed_y = _ports(PosTrackCommandE.SPEED_TRACK)
        manager.step(speed_u, speed_y)
        position_u, position_y = _ports(PosTrackCommandE.POSITION_TRACK)
        manager.step(position_u, position_y)

        assert speed_y.accCmd is not None
        assert position_y.accCmd is not None
        self.assertAlmostEqual(speed_y.accCmd.accEast, 0.0)
        self.assertGreater(position_y.accCmd.accEast, 0.0)
        self.assertEqual(
            {key: id(value) for key, value in manager._registry.items()},
            product_ids,
        )

    def test_rejects_command_whose_product_was_not_configured(self) -> None:
        """命令对应产品未装配时应明确失败，不允许隐式创建。"""

        manager = PosTrackManager()
        manager.init(
            EntityInitS(
                pos_track_strategies=(PosTrackStrategyE.NOOP, PosTrackStrategyE.PID_SPEED)
            )
        )
        u, y = _ports(PosTrackCommandE.POSITION_TRACK)

        with self.assertRaisesRegex(ValueError, "PID_POSITION"):
            manager.step(u, y)

    def test_rejects_plain_integer_command(self) -> None:
        """运行期命令必须使用语义枚举，普通整数不得绕过接口契约。"""

        manager = PosTrackManager()
        manager.init(EntityInitS(pos_track_strategies=(PosTrackStrategyE.NOOP,)))
        u, y = _ports(PosTrackCommandE.NOOP)
        assert u.command is not None
        u.command.mode = 0  # type: ignore[assignment]

        with self.assertRaisesRegex(ValueError, "PosTrackCommandE"):
            manager.step(u, y)

    def test_noop_clears_control_output(self) -> None:
        """NOOP 应只清零加速度并保留既有诊断和 PosCalc 目标快照。"""

        manager = PosTrackManager()
        manager.init(EntityInitS(pos_track_strategies=(PosTrackStrategyE.NOOP,)))
        u, y = _ports(PosTrackCommandE.NOOP)
        assert y.accCmd is not None
        assert y.diag is not None
        assert y.effectiveCmd is not None
        assert u.selfCmd is not None
        y.accCmd.accEast = 3.0
        y.diag.cmd_pos_east_m = 8.0

        manager.step(u, y)

        self.assertEqual(y.accCmd, AccInEarthS())
        self.assertEqual(y.diag.cmd_pos_east_m, 8.0)
        self.assertEqual(y.effectiveCmd, u.selfCmd)

    def test_pid_builders_preserve_previous_rally_configuration(self) -> None:
        """Manager 创建的速度/位置 PID 参数必须与重构前装配值逐字段一致。"""

        period_s = 0.2
        limit = VelCmdLimitS(
            forwardMin=8.0,
            forwardMax=25.0,
            verticalMin=-4.0,
            verticalMax=5.0,
        )

        self.assertEqual(_pid_speed_init(period_s, limit), _default_tracker_init(period_s, limit))
        self.assertEqual(
            _pid_position_init(period_s, limit),
            _follower_tracker_init(period_s, limit),
        )


if __name__ == "__main__":
    unittest.main()
