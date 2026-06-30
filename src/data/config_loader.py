"""配置文件加载辅助模块。注意：只处理通用文件引用，不做仿真语义校验。"""

from __future__ import annotations

import copy
import json
from pathlib import Path

from src.ui.gui.linefile import LineFileManager


_LINE_FILE_MANAGER = LineFileManager()


def load_config(path: str) -> dict[str, object]:
    """读取 JSON 仿真配置并展开外部元素引用。注意：文件路径由调用方保证存在且可读。"""
    config_path = Path(path)
    # 顶层加载器只负责根配置；外部元素由 resolve_config_references 统一展开。
    data = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("config root must be an object")
    return resolve_config_references(data, config_path)


def resolve_config_references(config: dict[str, object], config_path: str | Path) -> dict[str, object]:
    """展开配置中的外部航线和障碍引用，返回不修改入参的配置副本。

    支持字段：
    - 顶层 route_file：指向完整 route 对象。
    - avoidance.obstacles_file：指向障碍数组，或包含 obstacles 字段的对象。
    相对路径均按主配置文件所在目录解析。
    """
    base_path = Path(config_path)
    resolved = copy.deepcopy(config)

    route_file = resolved.get("route_file")
    if route_file is not None:
        # route_file 交给独立航线文件管理器，便于后续按客户格式扩展策略。
        resolved["route"] = _LINE_FILE_MANAGER.load_route(base_path, route_file)

    avoidance = resolved.get("avoidance")
    # 障碍文件引用暂时仍是通用 JSON，后续可按同样模式抽成 obstaclefile 策略。
    if isinstance(avoidance, dict):
        # 障碍库允许独立维护；展开后保持旧的 avoidance.obstacles 数组契约。
        obstacles_file = avoidance.get("obstacles_file")
        if obstacles_file is not None:
            obstacles_data = _load_referenced_json(base_path, obstacles_file, "avoidance.obstacles_file")
            if isinstance(obstacles_data, dict):
                # 兼容将来把障碍数组包进对象并附带说明字段的写法。
                obstacles_data = obstacles_data.get("obstacles", [])
            if not isinstance(obstacles_data, list):
                raise ValueError("avoidance.obstacles_file must point to an obstacles array")
            avoidance["obstacles"] = obstacles_data

    return resolved


def _load_referenced_json(base_path: Path, raw_path: object, field_name: str) -> object:
    """读取相对主配置的 JSON 引用。注意：非法路径直接按配置错误抛出。"""
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise ValueError(f"{field_name} must be a non-empty string")
    element_path = Path(raw_path)
    if not element_path.is_absolute():
        # 相对路径只相对主配置文件目录，避免受当前工作目录影响。
        element_path = base_path.parent / element_path
    return json.loads(element_path.read_text(encoding="utf-8"))

