"""Regression tests for avoidance obstacle config parsing (UI display only)."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from src.data.config_loader import resolve_config_references
from src.ui.gui.main_window import ObstacleView, parse_avoidance_config


class ParseAvoidanceConfigTests(unittest.TestCase):
    """Cover circle/rect parsing, the top-level switch, and safe-parse fallbacks."""

    def _write(self, payload: object) -> str:
        """Write payload as JSON to a temp file and return its path."""
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", encoding="utf-8", delete=False
        )
        json.dump(payload, handle)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_circle_and_rect_parsed(self) -> None:
        path = self._write(
            {
                "avoidance": {
                    "clearance_m": 120.0,
                    "obstacles": [
                        {
                            "id": "C1",
                            "type": "circle",
                            "enabled": True,
                            "center": {"east_m": 900.0, "north_m": 0.0},
                            "radius_m": 200.0,
                        },
                        {
                            "id": "R1",
                            "type": "rect",
                            "enabled": False,
                            "min": {"east_m": 350.0, "north_m": -180.0},
                            "max": {"east_m": 650.0, "north_m": 180.0},
                        },
                    ],
                }
            }
        )
        obstacles, clearance = parse_avoidance_config(path)
        self.assertEqual(clearance, 120.0)
        self.assertEqual([o.obstacle_id for o in obstacles], ["C1", "R1"])
        circle, rect = obstacles
        self.assertEqual(circle.kind, "circle")
        self.assertTrue(circle.enabled)
        self.assertEqual((circle.center_x, circle.center_y, circle.radius), (900.0, 0.0, 200.0))
        self.assertEqual(rect.kind, "rect")
        self.assertFalse(rect.enabled)
        self.assertEqual((rect.min_x, rect.min_y, rect.max_x, rect.max_y), (350.0, -180.0, 650.0, 180.0))

    def test_top_level_disabled_returns_empty(self) -> None:
        """文档约定：顶层 enabled=false 即完全跳过避障，即使含启用障碍。"""
        path = self._write(
            {
                "avoidance": {
                    "enabled": False,
                    "clearance_m": 120.0,
                    "obstacles": [
                        {"id": "C1", "type": "circle", "enabled": True,
                         "center": {"east_m": 10.0, "north_m": 20.0}, "radius_m": 30.0},
                    ],
                }
            }
        )
        obstacles, clearance = parse_avoidance_config(path)
        self.assertEqual(obstacles, [])
        self.assertEqual(clearance, 0.0)

    def test_missing_avoidance_section_returns_empty(self) -> None:
        path = self._write({"duration_s": 200.0})
        self.assertEqual(parse_avoidance_config(path), ([], 0.0))

    def test_empty_obstacles_returns_empty_list(self) -> None:
        path = self._write({"avoidance": {"clearance_m": 50.0, "obstacles": []}})
        obstacles, clearance = parse_avoidance_config(path)
        self.assertEqual(obstacles, [])
        self.assertEqual(clearance, 50.0)

    def test_bad_clearance_value_falls_back_safely(self) -> None:
        """UI-only 字段填错不得抛异常拖垮加载流程。"""
        path = self._write(
            {
                "avoidance": {
                    "clearance_m": "bad",
                    "obstacles": [
                        {"id": "C1", "type": "circle",
                         "center": {"east_m": 1.0, "north_m": 2.0}, "radius_m": 3.0},
                    ],
                }
            }
        )
        obstacles, clearance = parse_avoidance_config(path)
        self.assertEqual(clearance, 0.0)
        self.assertEqual(len(obstacles), 1)

    def test_bad_numeric_obstacle_fields_fall_back(self) -> None:
        path = self._write(
            {
                "avoidance": {
                    "obstacles": [
                        {"id": "C1", "type": "circle",
                         "center": {"east_m": "x", "north_m": None}, "radius_m": "nan?"},
                    ],
                }
            }
        )
        obstacles, _ = parse_avoidance_config(path)
        self.assertEqual(len(obstacles), 1)
        circle = obstacles[0]
        self.assertEqual((circle.center_x, circle.center_y, circle.radius), (0.0, 0.0, 0.0))

    def test_non_dict_and_malformed_entries_skipped(self) -> None:
        path = self._write(
            {
                "avoidance": {
                    "obstacles": [
                        "not-a-dict",
                        42,
                        {"id": "OK", "type": "circle", "center": {"east_m": 5.0}, "radius_m": 6.0},
                    ],
                }
            }
        )
        obstacles, _ = parse_avoidance_config(path)
        self.assertEqual([o.obstacle_id for o in obstacles], ["OK"])

    def test_obstacles_not_a_list_keeps_clearance_only(self) -> None:
        path = self._write({"avoidance": {"clearance_m": 80.0, "obstacles": "oops"}})
        obstacles, clearance = parse_avoidance_config(path)
        self.assertEqual(obstacles, [])
        self.assertEqual(clearance, 80.0)

    def test_external_obstacles_file_is_resolved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "obstacles.json").write_text(
                json.dumps([
                    {
                        "id": "C1",
                        "type": "circle",
                        "enabled": True,
                        "center": {"east_m": 10.0, "north_m": 20.0},
                        "radius_m": 30.0,
                    }
                ]),
                encoding="utf-8",
            )
            config = root / "base.json"
            config.write_text(
                json.dumps({"avoidance": {"enabled": True, "clearance_m": 12.0, "obstacles_file": "element/obstacles.json"}}),
                encoding="utf-8",
            )

            obstacles, clearance = parse_avoidance_config(str(config))

        self.assertEqual(clearance, 12.0)
        self.assertEqual([o.obstacle_id for o in obstacles], ["C1"])
        self.assertEqual((obstacles[0].center_x, obstacles[0].center_y, obstacles[0].radius), (10.0, 20.0, 30.0))

    def test_bad_route_file_does_not_clear_obstacle_display(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "obstacles.json").write_text(
                json.dumps([
                    {
                        "id": "C1",
                        "type": "circle",
                        "center": {"east_m": 10.0, "north_m": 20.0},
                        "radius_m": 30.0,
                    }
                ]),
                encoding="utf-8",
            )
            config = root / "base.json"
            config.write_text(
                json.dumps({
                    "route_file": "missing-line.json",
                    "avoidance": {"enabled": True, "clearance_m": 12.0, "obstacles_file": "element/obstacles.json"},
                }),
                encoding="utf-8",
            )

            obstacles, clearance = parse_avoidance_config(str(config))

        self.assertEqual(clearance, 12.0)
        self.assertEqual([o.obstacle_id for o in obstacles], ["C1"])

    def test_obstacles_file_object_without_obstacles_is_config_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "obstacles.json").write_text(json.dumps({"items": []}), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "must contain obstacles"):
                resolve_config_references(
                    {"avoidance": {"enabled": True, "obstacles_file": "element/obstacles.json"}},
                    root / "base.json",
                )

    def test_missing_id_gets_generated_default(self) -> None:
        path = self._write(
            {"avoidance": {"obstacles": [{"type": "circle", "radius_m": 1.0}]}}
        )
        obstacles, _ = parse_avoidance_config(path)
        self.assertEqual(obstacles[0].obstacle_id, "OB1")

    def test_unreadable_or_invalid_json_returns_empty(self) -> None:
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", encoding="utf-8", delete=False
        )
        handle.write("{not valid json")
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        self.assertEqual(parse_avoidance_config(handle.name), ([], 0.0))
        self.assertEqual(parse_avoidance_config("does-not-exist.json"), ([], 0.0))

    def test_default_base_config_still_parses(self) -> None:
        """确保夹具 test.json 仍能正确解析（启用 C1/C2、禁用 R1）。"""
        base = Path(__file__).resolve().parent / "fixtures" / "test.json"
        obstacles, clearance = parse_avoidance_config(str(base))
        # 不硬编码数值：以夹具 test.json 当前配置为准，避免调参后测试陈旧。
        expected_clearance = json.loads(base.read_text(encoding="utf-8"))["avoidance"]["clearance_m"]
        self.assertEqual(clearance, expected_clearance)
        enabled = {o.obstacle_id: o.enabled for o in obstacles}
        self.assertEqual(enabled, {"C1": True, "C2": True, "R1": False})
        self.assertIsInstance(obstacles[0], ObstacleView)


if __name__ == "__main__":
    unittest.main()
