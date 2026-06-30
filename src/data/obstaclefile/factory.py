"""障碍文件策略工厂。注意：调用方不应直接依赖某个具体策略类。"""

from __future__ import annotations

from pathlib import Path

from src.data.obstaclefile.json_strategy import JsonObstacleFileStrategy
from src.data.obstaclefile.strategy import ObstacleFileStrategy


class ObstacleFileStrategyFactory:
    """根据障碍文件路径选择策略。注意：后续客户格式通过注册策略扩展。"""

    def __init__(self, strategies: list[ObstacleFileStrategy] | None = None) -> None:
        """初始化策略工厂。注意：默认只注册标准 JSON 策略。"""
        # 测试或客户集成可以注入自定义策略列表，避免修改默认工厂代码。
        self._strategies = strategies or [JsonObstacleFileStrategy()]

    def create(self, path: str | Path) -> ObstacleFileStrategy:
        """选择支持 path 的障碍文件策略。注意：无匹配策略时抛出明确错误。"""
        obstacle_path = Path(path)
        # 与航线工厂一致，策略选择阶段不读取文件内容，保持错误边界清晰。
        # 工厂只看文件特征，不读取文件内容，避免策略选择阶段产生 IO 副作用。
        for strategy in self._strategies:
            if strategy.supports(obstacle_path):
                return strategy
        raise ValueError(f"unsupported obstacles_file format: {obstacle_path.suffix or '<none>'}")
