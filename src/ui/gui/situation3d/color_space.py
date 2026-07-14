"""Qt Quick 3D 顶点色使用的颜色空间转换。"""

from __future__ import annotations

import numpy as np


def srgb_to_linear(value: float | np.ndarray) -> float | np.ndarray:
    """把 sRGB 标量或数组转换为线性空间，并保持数组形状。"""

    array = np.asarray(value, dtype=np.float64)
    # 幂函数分支只接收非负颜色；最大值可避免 np.where 预计算未选分支时产生警告。
    high = np.power((np.maximum(array, 0.0) + 0.055) / 1.055, 2.4)
    converted = np.where(array <= 0.04045, array / 12.92, high)
    if array.ndim == 0:
        return float(converted)
    return converted
