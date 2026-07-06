"""航线文件策略工厂。注意：调用方不应直接依赖某个具体策略类。"""

from __future__ import annotations

from pathlib import Path

from src.data.linefile.diamond_xml_strategy import DiamondXmlLineFileStrategy
from src.data.linefile.json_strategy import JsonLineFileStrategy
from src.data.linefile.strategy import LineFileStrategy


class LineFileStrategyFactory:
    """根据航线文件路径选择策略。注意：后续客户格式通过注册策略扩展。"""

    def __init__(self, strategies: list[LineFileStrategy] | None = None) -> None:
        """初始化策略工厂。注意：默认注册标准 JSON 与钻石 XML 策略。"""
        # 测试或客户集成可以注入自定义策略列表，避免修改默认工厂代码。
        self._strategies = strategies or [JsonLineFileStrategy(), DiamondXmlLineFileStrategy()]

    def create(self, path: str | Path) -> LineFileStrategy:
        """选择支持 path 的航线文件策略。注意：无匹配策略时抛出明确错误。"""
        route_path = Path(path)
        # 工厂只看文件特征，不读取文件内容，避免策略选择阶段产生 IO 副作用。
        for strategy in self._strategies:
            if strategy.supports(route_path):
                return strategy
        raise ValueError(f"unsupported route_file format: {route_path.suffix or '<none>'}")
