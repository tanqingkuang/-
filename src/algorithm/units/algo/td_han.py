"""韩京清跟踪微分器(TD)：对指令信号做时间最优限加速平滑，输出平滑值 x1 与其导数 x2。

用途：软化"相对槽位偏移"在队形重构瞬间的阶跃——x1 是平滑后的参考位置，x2 天然给出参考速度前馈。
特点(相对线性预滤波)：加速度被 r 限住且是时间最优(fhan)，**小偏差近似透传、大阶跃按 r 配速**，
因此不会把小偏差也一并拖慢。状态 (x1,x2) 可在(重)挂载时按需播种，把参考对齐到当前实际量避免起步大阶跃。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TdHanInitS:
    """Han TD 初始化参数。注意：r 为参考加速度上界(越大越快)，h 为控制周期，h0 为滤波因子(<=0 取 h)。"""

    r: float = 0.0  # 速度因子：参考加速度上界，单位 m/s^2
    h: float = 0.0  # 积分步长(=控制周期)，单位 s
    h0: float = 0.0  # 滤波因子，单位 s；<=0 时取 h。输入干净(配置驱动)时取 h 即可
    vMax: float = 0.0  # 参考速度上界，单位 m/s；<=0 表示不限。夹到通道速度权限内，防大阶跃参考跑飞回路跟不上


def _fhan(x1: float, x2: float, r: float, h0: float) -> float:
    """韩京清 fhan 时间最优控制：把 (x1, x2) 以加速度上界 r 拉向原点。注意：x1 传入的是 (跟踪值-目标)。"""
    d = r * h0
    d0 = h0 * d
    y = x1 + h0 * x2
    a0 = math.sqrt(d * d + 8.0 * r * abs(y))
    if abs(y) <= d0:
        a = x2 + y / h0
    else:
        a = x2 + 0.5 * (a0 - d) * (1.0 if y > 0.0 else -1.0)
    if abs(a) <= d:
        return -r * a / d
    return -r * (1.0 if a > 0.0 else -1.0)


class TdHan:
    """标量 Han 跟踪微分器。注意：无状态数学在 _fhan，本类只持有 (x1,x2) 两个状态并逐拍推进。"""

    def __init__(self) -> None:
        """初始化 TdHan 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = TdHanInitS()
        self._h0 = 0.0
        self._x1 = 0.0
        self._x2 = 0.0

    def init(self, cfg: TdHanInitS) -> None:
        """按配置初始化 TdHan。注意：r/h 必须为正，h0<=0 时回退到 h。"""
        if cfg.r <= 0.0:
            raise ValueError("r must be > 0")
        if cfg.h <= 0.0:
            raise ValueError("h must be > 0")
        self._cfg = cfg
        self._h0 = cfg.h0 if cfg.h0 > 0.0 else cfg.h
        self.reset()

    def seed(self, x1: float, x2: float = 0.0) -> None:
        """设置状态初值(种子)。注意：(重)挂载首拍用当前实际量播种，使参考起点贴合飞机、避免起步大阶跃。"""
        self._x1 = x1
        self._x2 = x2

    def step(self, target: float) -> tuple[float, float]:
        """推进一拍，把 x1 以加速度上界 r 时间最优逼近 target，返回推进后的 (x1, x2)。"""
        cfg = self._cfg
        fh = _fhan(self._x1 - target, self._x2, cfg.r, self._h0)
        self._x1 += cfg.h * self._x2
        self._x2 += cfg.h * fh
        # 速度上界：把参考速度夹到通道权限内，得到"加速度 r + 速度 vMax"双限的可行参考(梯形速度剖面)。
        if cfg.vMax > 0.0:
            if self._x2 > cfg.vMax:
                self._x2 = cfg.vMax
            elif self._x2 < -cfg.vMax:
                self._x2 = -cfg.vMax
        return self._x1, self._x2

    @property
    def x1(self) -> float:
        """当前平滑参考值。"""
        return self._x1

    @property
    def x2(self) -> float:
        """当前参考导数(速度前馈)。"""
        return self._x2

    def reset(self) -> None:
        """复位 TdHan 的动态状态。注意：清零 (x1,x2)，保留构造期配置。"""
        self._x1 = 0.0
        self._x2 = 0.0
