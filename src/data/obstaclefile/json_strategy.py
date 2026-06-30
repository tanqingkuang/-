"""JSON 障碍文件策略。注意：当前默认 obstacles.json 仍保持原 obstacles 数组结构。"""

from __future__ import annotations

import json
from pathlib import Path

from src.data.obstaclefile.strategy import ObstacleFileStrategy


class JsonObstacleFileStrategy(ObstacleFileStrategy):
    """标准 JSON 障碍文件策略。注意：文件根可为 obstacles 数组或含 obstacles 的对象。"""

    def supports(self, path: Path) -> bool:
        """按 .json 后缀识别标准障碍文件。注意：大小写不敏感。"""
        return path.suffix.lower() == ".json"

    def load(self, path: Path) -> list[object]:
        """读取 JSON 障碍文件。注意：对象根必须显式包含 obstacles 键。"""
        data = json.loads(path.read_text(encoding="utf-8"))
        # 数组根是当前默认格式；对象根只用于未来附带版本、来源等元数据。
        if isinstance(data, dict):
            # 对象根用于未来附带元数据，但障碍数组字段必须显式存在，避免笔误被当作无障碍。
            if "obstacles" not in data:
                raise ValueError("avoidance.obstacles_file object must contain obstacles")
            data = data["obstacles"]
        # 策略输出必须是旧契约的 obstacles 数组，调用方不需要知道文件根形态。
        if not isinstance(data, list):
            raise ValueError("avoidance.obstacles_file must point to an obstacles array")
        return data

    def save(self, path: Path, obstacles: list[object]) -> None:
        """写入 JSON 障碍文件。注意：默认生成数组根，保持当前 element/obstacles.json 形态。"""
        # 生成时自动建目录，便于后续 GUI 直接输出到 configs/element 之类的新路径。
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(obstacles, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
