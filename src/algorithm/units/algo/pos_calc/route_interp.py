"""长机实体的航线插值目标计算。注意：当前按直线航段投影和延拓。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, WayLineS
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS


@dataclass
class RouteInterpInitS(PosCalcInitS):
    """航线插值初始化参数。注意：当前实现无额外字段，仅保留统一接口。"""

    pass


@dataclass
class RouteInterpInputS(PosCalcInputS):
    """航线插值输入端口。注意：wayLine 必须绑定当前需要跟踪的航段。"""

    wayLine: WayLineS | None = None


class RouteInterp(PosCalcBase):
    """长机航线插值目标计算器。注意：当前只支持直线航段，曲线航段会显式报错。"""

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

        # vdCmd 是地速（水平速度），故按水平投影长度分解：hypot(vEast,vNorth) 恰为 vdCmd，
        # vUp 由航迹角自然带出。固定翼不存在纯垂直航段，水平长度为零视为非法航线。
        hlen = math.hypot(dx, dy)
        if hlen <= 0.0:
            raise ValueError("wayLine must have non-zero horizontal length")
        y.selfCmd.v.vEast = line.vdCmd * dx / hlen
        y.selfCmd.v.vNorth = line.vdCmd * dy / hlen
        y.selfCmd.v.vUp = line.vdCmd * dz / hlen
        y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vTheta = math.atan2(y.selfCmd.v.vUp, y.selfCmd.v.vd) if line.vdCmd else 0.0
        y.selfCmd.v.vPsi = math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast) if line.vdCmd else 0.0

    def reset(self) -> None:
        """复位 RouteInterp 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
