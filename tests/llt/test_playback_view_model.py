"""播放控制 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.playback_view_model import PlaybackViewModel
from src.ui.gui.view_models import PLAYBACK_RATE_SLIDER_MAX


class PlaybackViewModelTests(unittest.TestCase):
    """覆盖播放控制稳定业务规则，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """PlaybackViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/playback_view_model.py"), Path(__file__)]
        for path in paths:
            with self.subTest(path=str(path)):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                imported_roots = {
                    alias.name.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Import)
                    for alias in node.names
                }
                imported_roots.update(
                    node.module.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module is not None
                )
                self.assertNotIn("PySide6", imported_roots)

    def test_slider_uses_segmented_rate_steps_and_clamps_illegal_values(self) -> None:
        """倍率滑条按离散分档映射，越界值夹到最近合法档位。"""

        view_model = PlaybackViewModel()
        cases = [
            (-100, 0.1, 1),
            (1, 0.1, 1),
            (10, 1.0, 10),
            (20, 2.0, 20),
            (21, 3.0, 21),
            (28, 10.0, 28),
            (29, 12.0, 29),
            (33, 20.0, 33),
            (34, 23.0, 34),
            (PLAYBACK_RATE_SLIDER_MAX, 50.0, PLAYBACK_RATE_SLIDER_MAX),
            (999, 50.0, PLAYBACK_RATE_SLIDER_MAX),
        ]

        for slider_value, expected_rate, expected_slider in cases:
            with self.subTest(slider_value=slider_value):
                update = view_model.on_slider_changed(slider_value)

                self.assertEqual(update.display_rate, expected_rate)
                self.assertEqual(update.controller_rate, expected_rate)
                self.assertEqual(update.slider_value, expected_slider)
                self.assertEqual(update.label_text, f"{expected_rate:.1f}x")

    def test_repeated_user_slider_value_remains_idempotent_rate_request(self) -> None:
        """重复收到同一滑条值时，输出保持一致且不会改变倍率语义。"""

        view_model = PlaybackViewModel()

        first = view_model.on_slider_changed(10)
        second = view_model.on_slider_changed(10)

        self.assertEqual(first, second)
        self.assertEqual(view_model.current_rate, 1.0)

    def test_programmatic_slider_sync_does_not_request_controller_rate(self) -> None:
        """程序回填滑条期间，即使触发 valueChanged 也不得重复下发倍率。"""

        view_model = PlaybackViewModel()

        sync_update = view_model.begin_programmatic_slider_sync(2.0)
        signal_update = view_model.on_slider_changed(sync_update.slider_value or 0)
        view_model.finish_programmatic_slider_sync()

        self.assertEqual(sync_update.slider_value, 20)
        self.assertEqual(sync_update.label_text, "2.0x")
        self.assertIsNone(sync_update.controller_rate)
        self.assertEqual(signal_update.display_rate, 2.0)
        self.assertIsNone(signal_update.controller_rate)

    def test_config_loaded_uses_controller_rate_for_label_and_nearest_slider(self) -> None:
        """加载配置后标签显示控制器真实倍率，滑条只吸附最近档位。"""

        view_model = PlaybackViewModel()

        update = view_model.on_config_loaded(2.5)

        self.assertEqual(update.display_rate, 2.5)
        self.assertEqual(update.label_text, "2.5x")
        self.assertEqual(update.slider_value, 20)
        self.assertIsNone(update.controller_rate)

    def test_reset_keeps_current_playback_rate_and_reapplies_it(self) -> None:
        """重置后保持当前倍率，并要求 adapter 重新下发给控制器。"""

        view_model = PlaybackViewModel()
        view_model.on_slider_changed(PLAYBACK_RATE_SLIDER_MAX)

        update = view_model.on_reset()

        self.assertEqual(update.display_rate, 50.0)
        self.assertEqual(update.controller_rate, 50.0)
        self.assertEqual(update.slider_value, PLAYBACK_RATE_SLIDER_MAX)
        self.assertFalse(view_model.paused)

    def test_paused_pause_request_never_becomes_resume(self) -> None:
        """暂停态下直接 pause 请求仍是 pause 幂等语义，不会变成 start。"""

        view_model = PlaybackViewModel()

        pause_decision = view_model.command_for_pause_request("PAUSED")
        toggle_decision = view_model.command_for_toggle("PAUSED")

        self.assertTrue(pause_decision.should_pause)
        self.assertFalse(pause_decision.should_start)
        self.assertFalse(toggle_decision.should_pause)
        self.assertTrue(toggle_decision.should_start)


if __name__ == "__main__":
    unittest.main()
