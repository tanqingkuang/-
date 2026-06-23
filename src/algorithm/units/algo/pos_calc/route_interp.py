"""长机实体的航线插值目标计算。注意：当前按直线航段投影和延拓。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, WayLineS
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS


@dataclass
class RouteInterpInitS(PosCalcInitS):
    pass


@dataclass
class RouteInterpInputS(PosCalcInputS):
    wayLine: WayLineS | None = None


class RouteInterp(PosCalcBase):
    def init(self, cfg: PosCalcInitS) -> None:
        """按配置初始化 RouteInterp。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: RouteInterpInputS, y: PosCalcOutputS) -> None:
        """推进 RouteInterp 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.selfState is None or u.wayLine is None or y.selfCmd is None:
            raise ValueError("RouteInterp ports must be bound")
        line = u.wayLine
        if line.radius != 0.0:
            raise NotImplementedError("curve route interpolation is not implemented")
        start = line.start.pos
        end = line.end.pos
        dx = end.east - start.east
        dy = end.north - start.north
        dz = end.h - start.h
        length2 = dx * dx + dy * dy + dz * dz
        if length2 <= 0.0:
            raise ValueError("wayLine start and end must be different")

        relx = u.selfState.pos.east - start.east
        rely = u.selfState.pos.north - start.north
        relz = u.selfState.pos.h - start.h
        t = max((relx * dx + rely * dy + relz * dz) / length2, 0.0)
        y.selfCmd.pos.east = start.east + t * dx
        y.selfCmd.pos.north = start.north + t * dy
        y.selfCmd.pos.h = start.h + t * dz

        length = math.sqrt(length2)
        y.selfCmd.vd.vEast = line.vdCmd * dx / length
        y.selfCmd.vd.vNorth = line.vdCmd * dy / length
        y.selfCmd.vd.vUp = line.vdCmd * dz / length
        y.selfCmd.vd.vd = math.hypot(y.selfCmd.vd.vEast, y.selfCmd.vd.vNorth)
        y.selfCmd.vd.vTheta = math.atan2(y.selfCmd.vd.vUp, y.selfCmd.vd.vd) if line.vdCmd else 0.0
        y.selfCmd.vd.vPsi = math.atan2(y.selfCmd.vd.vNorth, y.selfCmd.vd.vEast) if line.vdCmd else 0.0

    def reset(self) -> None:
        """复位 RouteInterp 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
