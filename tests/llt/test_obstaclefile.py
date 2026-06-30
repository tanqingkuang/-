"""障碍文件策略和管理器回归测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from src.data.obstaclefile import ObstacleFileManager, ObstacleFileStrategyFactory


class ObstacleFileTests(unittest.TestCase):
    """覆盖 obstacles_file 的解析、生成和策略工厂选择。"""

    def test_json_strategy_loads_array_by_relative_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            obstacles = [{"id": "C1", "type": "circle", "center": {"east_m": 1.0, "north_m": 2.0}, "radius_m": 3.0}]
            (element / "obstacles.json").write_text(json.dumps(obstacles), encoding="utf-8")

            loaded = ObstacleFileManager().load_obstacles(root / "base.json", "element/obstacles.json")

        self.assertEqual(loaded, obstacles)

    def test_json_strategy_loads_object_with_obstacles_key(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            obstacles = [{"id": "R1", "type": "rect", "min": {"east_m": 0.0}, "max": {"east_m": 10.0}}]
            (element / "obstacles.json").write_text(json.dumps({"obstacles": obstacles}), encoding="utf-8")

            loaded = ObstacleFileManager().load_obstacles(root / "base.json", "element/obstacles.json")

        self.assertEqual(loaded, obstacles)

    def test_json_strategy_saves_obstacles_array(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manager = ObstacleFileManager()
            obstacles = [{"id": "C1", "type": "circle", "radius_m": 30.0}]

            written = manager.save_obstacles(root / "base.json", "element/obstacles.json", obstacles)

            payload = json.loads(written.read_text(encoding="utf-8"))
        self.assertEqual(payload, obstacles)

    def test_factory_rejects_unsupported_obstacles_file_format(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported obstacles_file format"):
            ObstacleFileStrategyFactory().create("obstacles.csv")

    def test_obstacles_file_must_be_non_empty_string(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be a non-empty string"):
            ObstacleFileManager().load_obstacles("base.json", "")


if __name__ == "__main__":
    unittest.main()
