"""航线圆弧几何工具。注意：仅处理水平面(东/北)几何，高度由调用方按进度线性插值。

约定：转向符号 turn_sign：+1 左转(逆时针/CCW)、-1 右转(顺时针/CW)，与航迹偏航角速率 dVPsi 同号。
圆弧航段 WayLineS：start.pos=切入点、end.pos=切出点、start.turnSign=转向、start.center=圆心。
半径由 start.pos 与 start.center 的距离推导，不再作为顶层字段存储。
"""

from __future__ import annotations

import math

from src.algorithm.context.leaf_types import PosInEarthS, WayLineS


def _unit(dx: float, dy: float) -> tuple[float, float] | None:
    """求二维向量单位化结果。注意：零向量返回空值由调用方兜底。"""
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return dx / length, dy / length


def corner_arc(
    p_prev: PosInEarthS,
    p_corner: PosInEarthS,
    p_next: PosInEarthS,
    radius: float,
) -> tuple[PosInEarthS, PosInEarthS, PosInEarthS, float] | None:
    """由相邻三航点和半径求与两条腿相切的圆弧。

    返回 (T1 切入点, T2 切出点, center 圆心, turn_sign)；半径<=0、共线或近 U 形掉头返回 None。
    切点距拐点 d = R·tan(Δψ/2)；调用方需另行校验 d 不超过两条腿长度。
    """
    if radius <= 0.0:
        return None
    u_in = _unit(p_corner.east - p_prev.east, p_corner.north - p_prev.north)
    u_out = _unit(p_next.east - p_corner.east, p_next.north - p_corner.north)
    if u_in is None or u_out is None:
        return None
    # 转角 Δψ：cross 定号(左正)、dot 定余弦。
    cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
    dot = max(-1.0, min(1.0, u_in[0] * u_out[0] + u_in[1] * u_out[1]))
    dpsi = math.atan2(cross, dot)
    if abs(dpsi) < 1e-6 or abs(dpsi) > math.pi - 1e-6:
        return None  # 直线或近掉头，不做圆弧
    turn_sign = 1.0 if cross > 0.0 else -1.0
    d = radius * math.tan(abs(dpsi) / 2.0)  # 切点到拐点距离
    t1 = PosInEarthS(
        p_corner.east - d * u_in[0],
        p_corner.north - d * u_in[1],
        p_corner.h,
    )
    t2 = PosInEarthS(
        p_corner.east + d * u_out[0],
        p_corner.north + d * u_out[1],
        p_corner.h,
    )
    # 圆心在切入点的转向侧法向上：左转(turn_sign+1)取 u_in 左法向 (-uy,ux)。
    nx = -u_in[1] * turn_sign
    ny = u_in[0] * turn_sign
    center = PosInEarthS(t1.east + radius * nx, t1.north + radius * ny, p_corner.h)
    return t1, t2, center, turn_sign


def arc_radius(line: WayLineS) -> float:
    """从 start.pos 到 start.center 距离推导圆弧半径。注意：仅对 start.turnSign!=0 的圆弧段有意义。"""
    return math.hypot(
        line.start.pos.east - line.start.center.east,
        line.start.pos.north - line.start.center.north,
    )


def arc_swept_rad(line: WayLineS) -> float:
    """求圆弧航段的扫掠角(带符号，左正)。注意：仅用于 start.turnSign!=0 的圆弧段。"""
    center = line.start.center
    a_start = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
    a_end = math.atan2(line.end.pos.north - center.north, line.end.pos.east - center.east)
    delta = math.atan2(math.sin(a_end - a_start), math.cos(a_end - a_start))  # wrap 到 (-pi,pi]
    # 取与 turnSign 同向的扫掠；若 wrap 后符号相反，补一圈到同向。
    if line.start.turnSign >= 0.0 and delta < 0.0:
        delta += 2.0 * math.pi
    elif line.start.turnSign < 0.0 and delta > 0.0:
        delta -= 2.0 * math.pi
    return delta


def segment_length(line: WayLineS) -> float:
    """求航段水平长度。注意：直线取首末水平距离，圆弧取 R·|扫掠角|。"""
    if line.start.turnSign != 0.0:
        return arc_radius(line) * abs(arc_swept_rad(line))
    return math.hypot(
        line.end.pos.east - line.start.pos.east,
        line.end.pos.north - line.start.pos.north,
    )


def heading_at_s(line: WayLineS, s: float) -> float:
    """求航段在距起点弧长 s 处的航迹航向(弧度)。注意：s 会被钳到 [0, 段长]。"""
    if line.start.turnSign != 0.0:
        r = arc_radius(line)
        swept = arc_swept_rad(line)
        total = r * abs(swept)
        s = max(0.0, min(total, s))
        sign = 1.0 if swept >= 0.0 else -1.0
        center = line.start.center
        a_start = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
        radial = a_start + sign * (s / r)
        return radial + sign * (math.pi / 2.0)  # 切向 = 径向转 90°(按转向)
    return math.atan2(
        line.end.pos.north - line.start.pos.north,
        line.end.pos.east - line.start.pos.east,
    )


def project_arc(line: WayLineS, east: float, north: float) -> tuple[PosInEarthS, float, float, float]:
    """把一点投影到圆弧航段。返回 (投影点, 弧长 s, 进度[0,1], 该处航向)。注意：投影钳在弧两端之间。"""
    r = arc_radius(line)
    center = line.start.center
    swept = arc_swept_rad(line)
    sign = 1.0 if swept >= 0.0 else -1.0
    a_start = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
    a_pt = math.atan2(north - center.north, east - center.east)
    # 相对起点的有向夹角，折算到转向方向并钳进弧内。
    delta = math.atan2(math.sin(a_pt - a_start), math.cos(a_pt - a_start))
    along = delta * sign  # 沿转向为正
    along = max(0.0, min(abs(swept), along))
    s = along * r
    radial = a_start + sign * along
    proj = PosInEarthS(
        center.east + r * math.cos(radial),
        center.north + r * math.sin(radial),
        _lerp_height(line, abs(swept) and along / abs(swept) or 0.0),
    )
    progress = (along / abs(swept)) if abs(swept) > 0.0 else 1.0
    heading = radial + sign * (math.pi / 2.0)
    return proj, s, progress, heading


def _lerp_height(line: WayLineS, progress: float) -> float:
    """按进度在圆弧首末点之间线性插值高度。注意：水平转弯通常首末等高。"""
    return line.start.pos.h + (line.end.pos.h - line.start.pos.h) * progress
