"""JSON 航线文件策略。注意：当前默认 line.json 仍保持原 route 对象结构。"""

from __future__ import annotations

import json
from pathlib import Path

from src.data.linefile.strategy import LineFileStrategy


class JsonLineFileStrategy(LineFileStrategy):
    """标准 JSON 航线文件策略。注意：文件根必须是完整 route 对象。"""

    def supports(self, path: Path) -> bool:
        """按 .json 后缀识别标准航线文件。注意：大小写不敏感。"""
        return path.suffix.lower() == ".json"

    def load(self, path: Path) -> dict[str, object]:
        """读取 JSON 航线文件。注意：非法结构按配置错误抛出 ValueError。"""
        data = json.loads(path.read_text(encoding="utf-8"))
        # JSON 策略的输出契约是 route 对象本身，不能再包一层 route。
        if not isinstance(data, dict):
            raise ValueError("route_file must point to a route object")
        return data

    def save(self, path: Path, route: dict[str, object]) -> Path:
        """写入 JSON 航线文件并返回实际路径。注意：保持中文和缩进，便于客户直接编辑。"""
        # 生成时自动建目录，便于后续 GUI 直接输出到 configs/element 之类的新路径。
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(route, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return path

    def default_output_filename(self) -> str:
        """返回 JSON 避障航线默认输出名。注意：保持历史 GUI 默认行为。"""
        return "avoidance_route.json"
