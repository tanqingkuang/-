"""航线文件策略和管理器回归测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from src.data.linefile import LineFileManager, LineFileStrategyFactory
from src.data.config_loader import resolve_config_references
from tests.llt._geo_route import geodetic_route


class LineFileTests(unittest.TestCase):
    """覆盖 route_file 的解析、生成和策略工厂选择。"""

    def test_json_strategy_loads_route_object_by_relative_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "line.json").write_text(
                json.dumps({"speed_mps": 20.0, "waypoints": [{"x_m": 1.0, "y_m": 2.0, "altitude_m": 3.0}]}),
                encoding="utf-8",
            )

            route = LineFileManager().load_route(root / "base.json", "element/line.json")

        self.assertEqual(route["speed_mps"], 20.0)
        self.assertEqual(route["waypoints"], [{"x_m": 1.0, "y_m": 2.0, "altitude_m": 3.0}])

    def test_json_strategy_saves_route_object(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manager = LineFileManager()
            route = {"speed_mps": 18.0, "waypoints": [{"x_m": 10.0, "y_m": 0.0, "altitude_m": 1000.0}]}

            written = manager.save_route(root / "base.json", "element/line.json", route)

            payload = json.loads(written.read_text(encoding="utf-8"))
        self.assertEqual(payload, route)

    def test_factory_rejects_unsupported_route_file_format(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported route_file format"):
            LineFileStrategyFactory().create("line.csv")

    def test_route_file_must_be_non_empty_string(self) -> None:
        with self.assertRaisesRegex(ValueError, "route_file must be a non-empty string"):
            LineFileManager().load_route("base.json", "")

    def test_config_loader_resolves_rally_route_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "mission.json").write_text(
                json.dumps(geodetic_route({"speed_mps": 20.0, "waypoints": [{"x_m": 1.0, "y_m": 2.0, "altitude_m": 3.0}]})),
                encoding="utf-8",
            )
            (element / "rally.json").write_text(
                json.dumps(geodetic_route({"speed_mps": 18.0, "waypoints": [{"x_m": 4.0, "y_m": 5.0, "altitude_m": 6.0}]})),
                encoding="utf-8",
            )

            resolved = resolve_config_references(
                {"route_file": "element/mission.json", "rally_route_file": "element/rally.json"},
                root / "rally_demo.json",
            )

        self.assertEqual(resolved["route"]["speed_mps"], 20.0)
        self.assertEqual(resolved["rally_route"]["speed_mps"], 18.0)

    def test_config_loader_resolves_formation_files(self) -> None:
        """formation_files 应按主配置位置展开成内部 formations 列表。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element" / "formations"
            element.mkdir(parents=True)
            (element / "triangle.json").write_text(
                json.dumps({
                    "name": "TRIANGLE",
                    "slots": [
                        {"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                        {"node_id": "A02", "x_m": -54.0, "y_m": 0.0, "z_m": -58.0},
                    ],
                }),
                encoding="utf-8",
            )

            resolved = resolve_config_references(
                {
                    "formation": {
                        "coordinate_system": "x_forward_y_up_z_right",
                        "formation_files": ["element/formations/triangle.json"],
                    }
                },
                root / "base.json",
            )

        formations = resolved["formation"]["formations"]  # type: ignore[index]
        self.assertEqual(formations[0]["name"], "TRIANGLE")
        self.assertEqual(formations[0]["slots"][1]["node_id"], "A02")

    def test_config_loader_rejects_inline_formation_config(self) -> None:
        """文件入口不再接受旧的 pattern/slots 内联队形写法。"""
        with self.assertRaisesRegex(ValueError, "formation_files"):
            resolve_config_references(
                {
                    "formation": {
                        "pattern": "TRIANGLE",
                        "coordinate_system": "x_forward_y_up_z_right",
                        "slots": [{"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0}],
                    }
                },
                "base.json",
            )


if __name__ == "__main__":
    unittest.main()
