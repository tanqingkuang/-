"""配置文件加载辅助模块。注意：只处理通用文件引用，不做仿真语义校验。"""

from __future__ import annotations

import copy
from pathlib import Path

from src.data.linefile import LineFileManager
from src.data.obstaclefile import ObstacleFileManager
from src.data.geo_config import obstacles_to_internal, route_to_internal


_LINE_FILE_MANAGER = LineFileManager()
_OBSTACLE_FILE_MANAGER = ObstacleFileManager()


def resolve_config_references(config: dict[str, object], config_path: str | Path) -> dict[str, object]:
    """展开配置中的外部航线和障碍引用，返回不修改入参的配置副本。

    支持字段：
    - 顶层 route_file：指向完整 route 对象。
    - 顶层 rally_route_file：指向完整 rally_route 对象。
    - avoidance.obstacles_file：指向障碍数组，或包含 obstacles 字段的对象。
    相对路径均按主配置文件所在目录解析。
    """
    base_path = Path(config_path)
    resolved = copy.deepcopy(config)
    route_origin = None

    route_file = resolved.get("route_file")
    if route_file is not None:
        # route_file 交给独立航线文件管理器，便于后续按客户格式扩展策略。
        resolved["route"] = _LINE_FILE_MANAGER.load_route(base_path, route_file)
    route = resolved.get("route")
    if isinstance(route, dict):
        # 外部航线可使用经纬高；进入控制器前统一展开为内部 ENU。
        resolved["route"], route_origin = route_to_internal(route)

    rally_route_file = resolved.get("rally_route_file")
    if rally_route_file is not None:
        # 集结航线复用同一航线文件管理器，避免后续客户格式适配重复实现。
        resolved["rally_route"] = _LINE_FILE_MANAGER.load_route(base_path, rally_route_file)
    rally_route = resolved.get("rally_route")
    if isinstance(rally_route, dict):
        # 集结航线和任务航线共享同一个 origin；无任务航线时才退回自身首点。
        resolved["rally_route"], rally_origin = route_to_internal(rally_route, route_origin)
        route_origin = route_origin or rally_origin

    avoidance = resolved.get("avoidance")
    if isinstance(avoidance, dict):
        obstacles_file = avoidance.get("obstacles_file")
        if obstacles_file is not None:
            # obstacles_file 交给独立障碍文件管理器，便于后续按客户格式扩展策略。
            avoidance["obstacles"] = _OBSTACLE_FILE_MANAGER.load_obstacles(base_path, obstacles_file)
        obstacles = avoidance.get("obstacles")
        if isinstance(obstacles, list):
            # 障碍经纬度转换依赖上层注入的航线 origin，障碍文件策略本身不读取航线。
            avoidance["obstacles"] = obstacles_to_internal(obstacles, route_origin)

    return resolved

