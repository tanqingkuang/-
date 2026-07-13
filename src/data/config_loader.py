"""配置文件加载辅助模块。注意：只处理通用文件引用，不做仿真语义校验。"""

from __future__ import annotations

import copy
import json
from pathlib import Path

from src.data.linefile import LineFileManager
from src.data.obstaclefile import ObstacleFileManager
from src.data.geo_config import obstacles_to_internal, route_to_internal


_LINE_FILE_MANAGER = LineFileManager()
_OBSTACLE_FILE_MANAGER = ObstacleFileManager()
_LEGACY_FORMATION_KEYS = ("pattern", "slots", "formations")


def resolve_config_references(config: dict[str, object], config_path: str | Path) -> dict[str, object]:
    """展开配置中的外部航线、队形和障碍引用，返回不修改入参的配置副本。

    支持字段：
    - 顶层 route_file：指向完整 route 对象。
    - formation.formation_files：指向一个或多个单队形对象。
    - avoidance.obstacles_file：指向障碍数组，或包含 obstacles 字段的对象。
    相对路径均按主配置文件所在目录解析。
    """
    base_path = Path(config_path)
    resolved = copy.deepcopy(config)
    route_origin = None

    # 旧字段不能静默忽略，否则场景会在未知集结点上运行。
    for removed_field in ("rally_route_file", "rally_route"):
        if removed_field in resolved:
            raise ValueError(f"{removed_field} 已移除，请统一使用 route_file")

    _resolve_formation_files(resolved, base_path)

    route_file = resolved.get("route_file")
    if route_file is not None:
        # route_file 交给独立航线文件管理器，便于后续按客户格式扩展策略。
        resolved["route"] = _LINE_FILE_MANAGER.load_route(base_path, route_file)
    route = resolved.get("route")
    if isinstance(route, dict):
        # 外部航线可使用经纬高；进入控制器前统一展开为内部 ENU。
        resolved["route"], route_origin = route_to_internal(route)

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


def _resolve_formation_files(config: dict[str, object], config_path: Path) -> None:
    """展开 formation.formation_files。注意：文件入口不再接受旧的内联队形写法。"""
    formation = config.get("formation")
    if formation is None:
        return None
    if not isinstance(formation, dict):
        raise ValueError("formation must be an object")

    legacy_keys = [key for key in _LEGACY_FORMATION_KEYS if key in formation]
    formation_files = formation.get("formation_files")
    if formation_files is None:
        if legacy_keys:
            raise ValueError(
                "formation must use formation.formation_files; inline formation pattern/slots/formations "
                "is no longer supported"
            )
        return None
    if legacy_keys:
        raise ValueError("formation.formation_files cannot be combined with inline pattern/slots/formations")
    if not isinstance(formation_files, list) or not formation_files:
        raise ValueError("formation.formation_files must be a non-empty list")

    formations: list[object] = []
    for index, item in enumerate(formation_files):
        formation_path = _resolve_formation_file_path(config_path, item, index)
        data = json.loads(formation_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"formation.formation_files[{index}] must point to a formation object")
        if "formations" in data:
            raise ValueError(f"formation.formation_files[{index}] must point to one formation, not a list wrapper")
        if "slots" not in data:
            raise ValueError(f"formation.formation_files[{index}] object must contain slots")
        formations.append(data)
    formation["formations"] = formations
    return None


def _resolve_formation_file_path(config_path: Path, formation_file: object, index: int) -> Path:
    """解析单个队形文件路径。注意：相对路径以主配置文件所在目录为基准。"""
    if not isinstance(formation_file, str) or not formation_file.strip():
        raise ValueError(f"formation.formation_files[{index}] must be a non-empty string")
    formation_path = Path(formation_file)
    if not formation_path.is_absolute():
        # 主配置位置是唯一稳定参照点，避免 GUI 和 CLI 从不同 cwd 启动时解析出不同文件。
        formation_path = config_path.parent / formation_path
    return formation_path
