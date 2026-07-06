"""GUI 功能档位解析。注意：构建脚本和源码运行共用同一套档位名。"""

from __future__ import annotations

import os
from collections.abc import Mapping

FEATURE_PROFILE_ENV = "SIMU_GUI_FEATURE_PROFILE"
FULL_PROFILE = "full"
LITE_PROFILE = "lite"

_VALID_PROFILES = {FULL_PROFILE, LITE_PROFILE}


def normalize_feature_profile(value: str | None) -> str:
    """规整 GUI 功能档位。注意：空值保持全量版，未知值直接报错。"""

    # 默认全量版，保证源码运行和既有测试不需要额外环境变量。
    if value is None or not value.strip():
        return FULL_PROFILE
    profile = value.strip().lower()
    if profile not in _VALID_PROFILES:
        supported = ", ".join(sorted(_VALID_PROFILES))
        raise ValueError(f"未知 GUI 功能档位：{value!r}，支持：{supported}")
    return profile


def load_feature_profile(environ: Mapping[str, str] | None = None) -> str:
    """从环境变量读取 GUI 功能档位。注意：PyInstaller runtime hook 也走此入口。"""

    source = os.environ if environ is None else environ
    return normalize_feature_profile(source.get(FEATURE_PROFILE_ENV))
