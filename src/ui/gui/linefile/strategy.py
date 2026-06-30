"""航线文件策略接口。注意：新增客户格式时应实现本接口并交给工厂注册。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path


class LineFileStrategy(ABC):
    """航线文件读写策略基类。注意：策略只处理文件格式，不处理主配置合并。"""

    @abstractmethod
    def supports(self, path: Path) -> bool:
        """判断该策略是否支持指定航线文件。注意：通常按扩展名或文件特征判断。"""

    @abstractmethod
    def load(self, path: Path) -> dict[str, object]:
        """读取航线文件并返回 route 对象。注意：返回结构需兼容现有 config["route"]。"""

    @abstractmethod
    def save(self, path: Path, route: dict[str, object]) -> None:
        """把 route 对象写入航线文件。注意：用于后续界面导出或客户格式生成。"""
