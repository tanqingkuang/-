"""长机实体的航线插值目标计算。注意：支持直线投影/延拓与圆弧投影，并做曲率前馈。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, WayLineS
from src.algorithm.units.algo import arc_path
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS


@dataclass
class RouteInterpInitS(PosCalcInitS):
    """航线插值初始化参数。注意：lookAheadDistance 用于直线 L1 前视点，leadTimeS 用于曲率前馈前瞻。"""

    lookAheadDistance: float = 0.0  # 直线 L1 前视距离(米)，仅作用于直线段目标点
    leadTimeS: float = 0.0  # 曲率前馈前瞻时间 σ(秒)，前瞻窗长 L2=σ·vd；0 表示关闭前馈


class RouteInterp(PosCalcBase):
    """长机航线插值目标计算器。注意：直线段投影/延拓，圆弧段投影到弧；曲率经 σ 前瞻前馈。"""

    def __init__(self) -> None:
        """初始化 RouteInterp 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._look_ahead_distance = 0.0
        self._lead_time_s = 0.0

    def init(self, cfg: PosCalcInitS) -> None:
        """按配置初始化 RouteInterp。注意：调用方需先准备好必要依赖和输入数据。"""
        self._look_ahead_distance = cfg.lookAheadDistance if isinstance(cfg, RouteInterpInitS) else 0.0
        self._lead_time_s = cfg.leadTimeS if isinstance(cfg, RouteInterpInitS) else 0.0
        if self._look_ahead_distance < 0.0:
            raise ValueError("lookAheadDistance must be >= 0")
        if self._lead_time_s < 0.0:
            raise ValueError("leadTimeS must be >= 0")

    def step(self, u: PosCalcInputS, y: PosCalcOutputS) -> None:
        """推进 RouteInterp 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.selfState is None or u.wayLine is None or y.selfCmd is None:
            raise ValueError("RouteInterp ports must be bound")
        line = u.wayLine
        if line.start.turnSign != 0.0:
            self._interp_arc(line, u.selfState, y.selfCmd)
        else:
            self._interp_straight(line, u.selfState, y.selfCmd)
        # 曲率前馈(直线/圆弧通用)：dVPsi = vd·κ_ff，κ_ff 为 σ 前瞻窗内的平均曲率(航向差/窗长)。
        y.selfCmd.v.dVPsi = self._curvature_ff(u)

    def _interp_straight(self, line: WayLineS, self_state: MotionProfS, self_cmd: MotionProfS) -> None:
        """直线航段：把本体投影到航段并按 L1 延拓，给出目标位置与沿航段速度。注意：行为与历史一致。"""
        start = line.start.pos
        end = line.end.pos
        dx = end.east - start.east
        dy = end.north - start.north
        dz = end.h - start.h
        length2 = dx * dx + dy * dy + dz * dz
        if length2 <= 0.0:
            raise ValueError("wayLine start and end must be different")

        relx = self_state.pos.east - start.east
        rely = self_state.pos.north - start.north
        relz = self_state.pos.h - start.h
        t = max((relx * dx + rely * dy + relz * dz) / length2, 0.0)
        if self._look_ahead_distance > 0.0:
            t += self._look_ahead_distance / math.sqrt(length2)
        self_cmd.pos.east = start.east + t * dx
        self_cmd.pos.north = start.north + t * dy
        self_cmd.pos.h = start.h + t * dz

        # vdCmd 是地速（水平速度），故按水平投影长度分解：hypot(vEast,vNorth) 恰为 vdCmd，
        # vUp 由航迹角自然带出。固定翼不存在纯垂直航段，水平长度为零视为非法航线。
        hlen = math.hypot(dx, dy)
        if hlen <= 0.0:
            raise ValueError("wayLine must have non-zero horizontal length")
        self_cmd.v.vEast = line.start.vdCmd * dx / hlen
        self_cmd.v.vNorth = line.start.vdCmd * dy / hlen
        self_cmd.v.vUp = line.start.vdCmd * dz / hlen
        self_cmd.v.vd = math.hypot(self_cmd.v.vEast, self_cmd.v.vNorth)
        self_cmd.v.vTheta = math.atan2(self_cmd.v.vUp, self_cmd.v.vd) if line.start.vdCmd else 0.0
        self_cmd.v.vPsi = math.atan2(self_cmd.v.vNorth, self_cmd.v.vEast) if line.start.vdCmd else 0.0

    def _interp_arc(self, line: WayLineS, self_state: MotionProfS, self_cmd: MotionProfS) -> None:
        """圆弧航段：把本体投影到弧上(目标点=投影点，避免前视弓高)，给出切向速度。"""
        proj, _s, progress, heading = arc_path.project_arc(line, self_state.pos.east, self_state.pos.north)
        self_cmd.pos.east = proj.east
        self_cmd.pos.north = proj.north
        self_cmd.pos.h = proj.h
        # 垂向按弧首末高度沿弧长线性变化(水平转弯时通常为 0)。
        seg_len = arc_path.segment_length(line)
        slope = (line.end.pos.h - line.start.pos.h) / seg_len if seg_len > 0.0 else 0.0
        self_cmd.v.vEast = line.start.vdCmd * math.cos(heading)
        self_cmd.v.vNorth = line.start.vdCmd * math.sin(heading)
        self_cmd.v.vUp = line.start.vdCmd * slope
        self_cmd.v.vd = math.hypot(self_cmd.v.vEast, self_cmd.v.vNorth)
        self_cmd.v.vTheta = math.atan2(self_cmd.v.vUp, self_cmd.v.vd) if line.start.vdCmd else 0.0
        self_cmd.v.vPsi = heading if line.start.vdCmd else 0.0
        del progress  # 进度仅用于内部高度插值，已在 project_arc 内处理

    def _curvature_ff(self, u: PosCalcInputS) -> float:
        """σ 前瞻曲率前馈：dVPsi = vd·κ_ff，κ_ff=(前瞻航向−当前航向)/窗长。注意：只跨入圆弧下一段。"""
        vd = u.selfState.v.vd
        lead = self._lead_time_s * vd  # L2 = σ·vd
        if lead <= 1e-9:
            return 0.0
        line = u.wayLine
        s0 = _arclength_on(line, u.selfState.pos.east, u.selfState.pos.north)
        seg_len = arc_path.segment_length(line)
        psi0 = arc_path.heading_at_s(line, s0)
        remain = seg_len - s0
        if lead <= remain:
            psi1 = arc_path.heading_at_s(line, s0 + lead)
        elif u.nextWayLine is not None and u.nextWayLine.start.turnSign != 0.0:
            # 仅当下一段是圆弧才跨段前瞻；直线下一段(含尖角)不跨，避免在直线上凭空前馈。
            psi1 = arc_path.heading_at_s(u.nextWayLine, lead - remain)
        else:
            psi1 = arc_path.heading_at_s(line, seg_len)
        dpsi = math.atan2(math.sin(psi1 - psi0), math.cos(psi1 - psi0))  # wrap 到 (-pi,pi]
        return vd * (dpsi / lead)

    def reset(self) -> None:
        """复位 RouteInterp 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None


def _arclength_on(line: WayLineS, east: float, north: float) -> float:
    """求一点在航段上的投影弧长。注意：直线取水平投影并钳在段内，圆弧取弧投影。"""
    if line.start.turnSign != 0.0:
        _proj, s, _prog, _hdg = arc_path.project_arc(line, east, north)
        return s
    dx = line.end.pos.east - line.start.pos.east
    dy = line.end.pos.north - line.start.pos.north
    hlen2 = dx * dx + dy * dy
    if hlen2 <= 0.0:
        return 0.0
    t = ((east - line.start.pos.east) * dx + (north - line.start.pos.north) * dy) / hlen2
    return max(0.0, min(1.0, t)) * math.sqrt(hlen2)
