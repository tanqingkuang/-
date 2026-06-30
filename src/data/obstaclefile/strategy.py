"""障碍文件策略接口。注意：新增客户格式时应实现本接口并交给工厂注册。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path


class ObstacleFileStrategy(ABC):
    """障碍文件读写策略基类。注意：策略只处理文件格式，不处理主配置合并。"""

    @abstractmethod
    def supports(self, path: Path) -> bool:
        """判断该策略是否支持指定障碍文件。注意：通常按扩展名或文件特征判断。"""

    @abstractmethod
    def load(self, path: Path) -> list[object]:
        """读取障碍文件并返回 obstacles 数组。注意：返回结构需兼容现有 avoidance.obstacles。"""

    @abstractmethod
    def save(self, path: Path, obstacles: list[object]) -> None:
        """把 obstacles 数组写入障碍文件。注意：用于后续界面导出或客户格式生成。"""
