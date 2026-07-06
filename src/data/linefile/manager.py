"""航线文件管理器。注意：负责 route_file 相对路径解析和策略调度。"""

from __future__ import annotations

from pathlib import Path

from src.data.linefile.factory import LineFileStrategyFactory


class LineFileManager:
    """航线文件门面类。注意：控制器和 GUI 只通过本类解析/生成航线文件。"""

    def __init__(self, factory: LineFileStrategyFactory | None = None) -> None:
        """初始化航线文件管理器。注意：可注入工厂以便单测或客户策略替换。"""
        self._factory = factory or LineFileStrategyFactory()

    def load_route(self, config_path: str | Path, route_file: object) -> dict[str, object]:
        """按主配置位置解析 route_file 并读取 route 对象。注意：相对路径不受 cwd 影响。"""
        route_path = self.resolve_path(config_path, route_file)
        # 路径解析和格式策略分开，后续客户格式只替换策略，不影响调用方。
        return self._factory.create(route_path).load(route_path)

    def save_route(self, config_path: str | Path, route_file: object, route: dict[str, object]) -> Path:
        """按主配置位置生成 route_file。注意：返回实际写入路径，便于界面提示。"""
        route_path = self.resolve_path(config_path, route_file)
        # save 与 load 使用同一个工厂，保证同一后缀读写策略一致。
        return self._factory.create(route_path).save(route_path, route)

    def default_output_filename(self, route_file: str | Path) -> str:
        """按 route_file 对应策略返回建议输出文件名。注意：只看格式，不读取文件内容。"""
        return self._factory.create(route_file).default_output_filename()

    @staticmethod
    def resolve_path(config_path: str | Path, route_file: object) -> Path:
        """把 route_file 转成实际文件路径。注意：相对路径以主配置文件所在目录为基准。"""
        if not isinstance(route_file, str) or not route_file.strip():
            raise ValueError("route_file must be a non-empty string")
        route_path = Path(route_file)
        # 绝对路径允许用于临时调试；正式配置仍建议使用相对路径。
        if not route_path.is_absolute():
            # 主配置是稳定参照点，避免从不同启动目录运行时解析到不同文件。
            route_path = Path(config_path).parent / route_path
        return route_path
