"""位置跟踪 Manager 的低层测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    MotionProfS,
    PosInEarthS,
    PosTrackCommandE,
    VdInEarthS,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityManagerInitS,
    EntityProcessSpecS,
    EntityRuntimeS,
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


def _entity_cfg(strategies: tuple[object, ...]) -> EntityManagerInitS:
    """构造仅配置位置跟踪流程的实体初始化参数。"""

    return EntityManagerInitS(
        entity=EntityInitS(),
        process=EntityProcessSpecS(strategies=strategies),
    )


class PosTrackManagerTests(unittest.TestCase):
    """验证显式配置、固定映射和缓存产品。"""

    def test_init_rejects_incomplete_strategy_table(self) -> None:
        """空表、重复策略和缺少 NOOP 均应在初始化期失败。"""

        cases = (
            (_entity_cfg(()), "不得为空"),
            (
                _entity_cfg((PosTrackStrategyE.NOOP, PosTrackStrategyE.NOOP)),
                "不得包含重复策略",
            ),
            (
                _entity_cfg((PosTrackStrategyE.PID_SPEED,)),
                "必须显式包含 NOOP",
            ),
        )
        for cfg, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                PosTrackManager().init(cfg)

    def test_command_selects_cached_one_to_one_product(self) -> None:
        """运行期命令应选择固定策略，且切换前后不重建有状态产品。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.SPEED_TRACK)
        manager.bind(runtime)
        manager.init(
            _entity_cfg(
                (
                    PosTrackStrategyE.NOOP,
                    PosTrackStrategyE.PID_SPEED,
                    PosTrackStrategyE.PID_POSITION,
                ),
            )
        )
        product_ids = {key: id(value) for key, value in manager._registry.items()}

        manager.step()
        speed_acc_east = runtime.context.selfAccCmd.accEast
        runtime.context.posTrackCommand.mode = PosTrackCommandE.POSITION_TRACK
        manager.step()

        self.assertAlmostEqual(speed_acc_east, 0.0)
        self.assertGreater(runtime.context.selfAccCmd.accEast, 0.0)
        self.assertEqual(
            {key: id(value) for key, value in manager._registry.items()},
            product_ids,
        )

    def test_rejects_command_whose_product_was_not_configured(self) -> None:
        """命令对应产品未装配时应明确失败，不允许隐式创建。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.POSITION_TRACK)
        manager.bind(runtime)
        manager.init(
            _entity_cfg((PosTrackStrategyE.NOOP, PosTrackStrategyE.PID_SPEED))
        )

        with self.assertRaisesRegex(ValueError, "PID_POSITION"):
            manager.step()

    def test_rejects_plain_integer_command(self) -> None:
        """运行期命令必须使用语义枚举，普通整数不得绕过接口契约。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.NOOP)
        manager.bind(runtime)
        manager.init(_entity_cfg((PosTrackStrategyE.NOOP,)))
        runtime.context.posTrackCommand.mode = 0  # type: ignore[assignment]

        with self.assertRaisesRegex(ValueError, "PosTrackCommandE"):
            manager.step()

    def test_noop_clears_control_output(self) -> None:
        """NOOP 应只清零加速度并保留既有诊断和 PosCalc 目标快照。"""

        manager = PosTrackManager()
        runtime = _runtime(PosTrackCommandE.NOOP)
        manager.bind(runtime)
        manager.init(_entity_cfg((PosTrackStrategyE.NOOP,)))
        runtime.context.selfAccCmd.accEast = 3.0
        runtime.posTrackDiag.cmd_pos_east_m = 8.0

        manager.step()

        self.assertEqual(runtime.context.selfAccCmd, AccInEarthS())
        self.assertEqual(runtime.posTrackDiag.cmd_pos_east_m, 8.0)
        self.assertEqual(runtime.context.effectiveCmd, runtime.context.selfCmd)

if __name__ == "__main__":
    unittest.main()
