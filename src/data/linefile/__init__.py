"""航线文件适配包。注意：对外只暴露工厂和管理器，避免调用方绑定具体格式。"""

from src.data.linefile.factory import LineFileStrategyFactory
from src.data.linefile.manager import LineFileManager

__all__ = ["LineFileManager", "LineFileStrategyFactory"]
