"""从注入航线中选择当前航段的规划器。注意：多航段切换由待飞距和提前量决定。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, PosInEarthS, RouteS, WayLineS, WayPointS, copy_wayline
from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS, TraPlanInputS, TraPlanOutputS

_GRAVITY_MPS2 = 9.80665
_TURN_BANK_DEG = 20.0


@dataclass
class LeaderRouteInitS(TraPlanInitS):
    route: RouteS | None = None


class LeaderRoute(TraPlanBase):
    def __init__(self) -> None:
        """初始化 LeaderRoute 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._route = _default_route()
        self._current_index = 0

    def init(self, cfg: TraPlanInitS | None) -> None:
        """按配置初始化 LeaderRoute。注意：调用方需先准备好必要依赖和输入数据。"""
        if isinstance(cfg, LeaderRouteInitS) and cfg.route is not None and cfg.route.lines:
            self._route = _clone_route(cfg.route)
        else:
            self._route = _default_route()
        self._current_index = 0

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        """推进 LeaderRoute 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.wayLine is None:
            raise ValueError("LeaderRoute output port must be bound")
        line = self._select_current_line(u.selfState)
        copy_wayline(line, y.wayLine)

    def reset(self) -> None:
        """复位 LeaderRoute 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._current_index = 0

    def _select_current_line(self, self_state: MotionProfS | None) -> WayLineS:
        """选择当前应跟踪的航段。注意：会按待飞距和转弯提前量切换到下一段。"""
        lines = self._route.lines
        if not lines:
            raise ValueError("route must contain at least one wayLine")
        if self_state is None:
            return lines[self._current_index]
        while self._current_index < len(lines) - 1 and _should_switch_to_next_line(
            lines[self._current_index],
            lines[self._current_index + 1],
            self_state,
        ):
            self._current_index += 1
        return lines[self._current_index]


def _default_route() -> RouteS:
    """生成默认航线对象。注意：仅在外部配置缺失时作为兜底。"""
    return RouteS(lines=[_default_line()])


def _default_line() -> WayLineS:
    """生成默认单航段。注意：起终点和速度需与默认场景保持一致。"""
    return WayLineS(
        idx=0,
        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
        end=WayPointS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0)),
        vdCmd=8.0,
        radius=0.0,
    )


def _clone_wayline(src: WayLineS) -> WayLineS:
    """复制单个航段。注意：避免算法直接持有配置对象引用。"""
    dst = WayLineS()
    copy_wayline(src, dst)
    return dst


def _clone_route(src: RouteS) -> RouteS:
    """复制整条航线。注意：每个航段都需要独立复制。"""
    return RouteS(lines=[_clone_wayline(line) for line in src.lines])


def _line_progress(line: WayLineS, self_state: MotionProfS) -> float:
    """计算飞机沿当前航段的归一化进度。注意：退化航段返回已完成。"""
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
    return (relx * dx + rely * dy + relz * dz) / length2


def _should_switch_to_next_line(line: WayLineS, next_line: WayLineS, self_state: MotionProfS) -> bool:
    """判断是否提前切换到下一航段。注意：切换距离使用转弯半径乘航向夹角半角正切。"""
    if _line_progress(line, self_state) >= 1.0:
        return True
    distance_to_go = _horizontal_distance_to_go(line, self_state)
    if distance_to_go is None:
        return False
    return distance_to_go <= _turn_switch_distance_m(line, next_line)


def _horizontal_distance_to_go(line: WayLineS, self_state: MotionProfS) -> float | None:
    """计算当前水平航段剩余待飞距。注意：退化航段返回空值。"""
    start = line.start.pos
    end = line.end.pos
    dx = end.east - start.east
    dy = end.north - start.north
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    track_x = dx / length
    track_y = dy / length
    return (end.east - self_state.pos.east) * track_x + (end.north - self_state.pos.north) * track_y


def _turn_radius_m(line: WayLineS) -> float:
    """按固定滚转角估算转弯半径。注意：速度为零时半径也为零。"""
    speed = max(0.0, line.vdCmd)
    return speed * speed / (_GRAVITY_MPS2 * math.tan(math.radians(_TURN_BANK_DEG)))


def _turn_switch_distance_m(line: WayLineS, next_line: WayLineS) -> float:
    """计算航段切换提前量。注意：公式为 R 乘 tan(delta_psi/2)。"""
    delta_psi = _heading_change_rad(line, next_line)
    return _turn_radius_m(line) * math.tan(delta_psi / 2.0)


def _heading_change_rad(line: WayLineS, next_line: WayLineS) -> float:
    """计算相邻航段水平航向夹角。注意：结果单位为弧度。"""
    current = _horizontal_unit_vector(line)
    next_track = _horizontal_unit_vector(next_line)
    if current is None or next_track is None:
        return 0.0
    dot = max(-1.0, min(1.0, current[0] * next_track[0] + current[1] * next_track[1]))
    cross = current[0] * next_track[1] - current[1] * next_track[0]
    return abs(math.atan2(cross, dot))


def _horizontal_unit_vector(line: WayLineS) -> tuple[float, float] | None:
    """计算航段水平单位方向向量。注意：水平长度过小时返回空值。"""
    dx = line.end.pos.east - line.start.pos.east
    dy = line.end.pos.north - line.start.pos.north
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return dx / length, dy / length
