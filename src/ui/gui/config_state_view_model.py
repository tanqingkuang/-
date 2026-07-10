"""配置路径与 ini 记忆决策 ViewModel。注意：本模块只含纯 Python 规则，不依赖 Qt。"""

from __future__ import annotations

import os
from pathlib import Path


def relative_config_path(path: Path, project_root: Path) -> str | None:
    """计算配置相对项目根的路径。注意：跨盘符或结果仍为绝对路径时返回 None。"""

    try:
        # Windows 上跨盘符无法相对化会抛 ValueError。
        relative_path = os.path.relpath(path.resolve(), project_root)
    except ValueError:
        return None
    # relpath 仍返回绝对路径（如不同盘）则视为不可相对化。
    if os.path.isabs(relative_path):
        return None
    try:
        # 统一用正斜杠存储，跨平台读取同一份 ini 时保持一致。
        return Path(relative_path).as_posix()
    except ValueError:
        return None


def display_config_path(path: Path, project_root: Path) -> str:
    """生成配置路径显示文本。注意：无法相对化时只显示文件名。"""

    # 界面优先显示可移植的相对路径，跨盘符配置退回简短文件名。
    relative_path = relative_config_path(path, project_root)
    return relative_path if relative_path is not None else path.name


def dialog_start_dir(last_relative: str | None, project_root: Path) -> Path:
    """决定配置对话框起始目录。注意：本函数会查询候选父目录是否存在。"""

    # 首次运行没有记忆记录，直接从项目根目录开始。
    if last_relative is None:
        return project_root
    config_path = (project_root / last_relative).resolve()
    candidate = config_path.parent
    # 目录已不存在则退回项目根，避免对话框打开到无效位置。
    return candidate if candidate.exists() else project_root


def parse_last_config_value(raw: str) -> str | None:
    """解析 ini 中的上次配置值。注意：空串和纯空白统一返回 None。"""

    # ConfigParser 可能返回带缩进的值，记忆策略只保留首尾去白后的路径。
    value = raw.strip()
    return value or None
