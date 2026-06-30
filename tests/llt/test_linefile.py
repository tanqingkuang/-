"""航线文件策略和管理器回归测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from src.data.linefile import LineFileManager, LineFileStrategyFactory


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


if __name__ == "__main__":
    unittest.main()
