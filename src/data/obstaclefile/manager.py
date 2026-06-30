"""障碍文件管理器。注意：负责 obstacles_file 相对路径解析和策略调度。"""

from __future__ import annotations

from pathlib import Path

from src.data.obstaclefile.factory import ObstacleFileStrategyFactory


class ObstacleFileManager:
    """障碍文件门面类。注意：控制器和 GUI 只通过本类解析/生成障碍文件。"""

    def __init__(self, factory: ObstacleFileStrategyFactory | None = None) -> None:
        """初始化障碍文件管理器。注意：可注入工厂以便单测或客户策略替换。"""
        self._factory = factory or ObstacleFileStrategyFactory()

    def load_obstacles(self, config_path: str | Path, obstacles_file: object) -> list[object]:
        """按主配置位置解析 obstacles_file 并读取 obstacles 数组。注意：相对路径不受 cwd 影响。"""
        obstacles_path = self.resolve_path(config_path, obstacles_file)
        # manager 只负责路径和策略调度，不解析单个障碍的 circle/rect 业务字段。
        # 路径解析和格式策略分开，后续客户格式只替换策略，不影响调用方。
        return self._factory.create(obstacles_path).load(obstacles_path)

    def save_obstacles(self, config_path: str | Path, obstacles_file: object, obstacles: list[object]) -> Path:
        """按主配置位置生成 obstacles_file。注意：返回实际写入路径，便于界面提示。"""
        obstacles_path = self.resolve_path(config_path, obstacles_file)
        # 生成路径和读取路径共用同一套解析规则，避免导入导出相对目录不一致。
        # save 与 load 使用同一个工厂，保证同一后缀读写策略一致。
        self._factory.create(obstacles_path).save(obstacles_path, obstacles)
        return obstacles_path

    @staticmethod
    def resolve_path(config_path: str | Path, obstacles_file: object) -> Path:
        """把 obstacles_file 转成实际文件路径。注意：相对路径以主配置文件所在目录为基准。"""
        if not isinstance(obstacles_file, str) or not obstacles_file.strip():
            raise ValueError("avoidance.obstacles_file must be a non-empty string")
        obstacles_path = Path(obstacles_file)
        # 绝对路径允许用于临时调试；正式配置仍建议使用相对路径。
        if not obstacles_path.is_absolute():
            # 这里接收主配置文件路径而不是目录，调用方不需要预先拆 parent。
            # 主配置是稳定参照点，避免从不同启动目录运行时解析到不同文件。
            obstacles_path = Path(config_path).parent / obstacles_path
        return obstacles_path
