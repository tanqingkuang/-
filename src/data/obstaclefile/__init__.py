"""障碍文件适配包。注意：对外只暴露工厂和管理器，避免调用方绑定具体格式。"""

from src.data.obstaclefile.factory import ObstacleFileStrategyFactory
from src.data.obstaclefile.manager import ObstacleFileManager

__all__ = ["ObstacleFileManager", "ObstacleFileStrategyFactory"]
