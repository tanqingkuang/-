"""从注入航线中选择当前航段的规划器。注意：多航段切换由待飞距和提前量决定。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    MotionProfS,
    PosInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
    copy_motion,
    copy_wayline,
)
from src.algorithm.units.algo import arc_path
from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS

if TYPE_CHECKING:
    from src.algorithm.context.context import FormContextS
    from src.algorithm.entity.types import EntityRuntimeS

_GRAVITY_MPS2 = 9.80665  # 重力加速度，用于由速度和滚转角估算转弯半径
_TURN_BANK_DEG = 20.0  # 标称协调转弯滚转角，越大则转弯半径越小
_TURN_SWITCH_DISTANCE_SCALE = 1.2  # 切段提前量系数，用于给实际转弯响应留裕度


@dataclass
class LeaderRouteInitS(TraPlanInitS):
    """长机航路规划初始化配置。注意：route 为空时退化为默认单航段。"""

    route: list[WayLineS] | None = None  # 预置航线（已完成圆弧几何计算的航段序列）


@dataclass
class LeaderRouteInputS:
    """长机航线策略输入快照。注意：只包含航段推进所需的本机状态。"""

    selfState: MotionProfS = field(default_factory=MotionProfS)


@dataclass
class LeaderRouteOutputS:
    """长机航线策略输出快照。注意：计算成功后统一提交黑板。"""

    wayLine: WayLineS = field(default_factory=WayLineS)
    nextWayLine: WayLineS = field(default_factory=WayLineS)


class LeaderRoute(TraPlanBase):
    """长机航路规划器：维护当前航段索引，按本机位置在多航段间顺序推进。注意：切换不可回退，只单向递增到下一段。"""

    def __init__(self) -> None:
        """初始化 LeaderRoute 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._route = _default_route()
        self._current_index = 0  # 当前正在跟踪的航段下标
        self._cxt: FormContextS | None = None
        self._u = LeaderRouteInputS()
        self._y = LeaderRouteOutputS()

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体黑板。注意：运行期通过内部快照隔离计算和提交。"""
        self._cxt = runtime.context

    def init(self, cfg: TraPlanInitS | None) -> None:
        """按配置初始化 LeaderRoute。注意：调用方需先准备好必要依赖和输入数据。"""
        # 仅当配置类型正确且含非空航段时采用注入航线，否则兜底默认航线
        if isinstance(cfg, LeaderRouteInitS) and cfg.route is not None and cfg.route:
            self._route = _clone_route(cfg.route)  # 深拷贝，防止算法持有外部配置引用
        else:
            self._route = _default_route()
        self._current_index = 0

    def step(self) -> None:
        """推进航段选择并提交黑板。注意：异常时不写入半成品航段。"""
        if self._cxt is None:
            raise ValueError("LeaderRoute 尚未绑定黑板")
        copy_motion(self._cxt.selfState, self._u.selfState)
        index = self._select_current_index(self._u.selfState)  # 据本机位置选定当前航段下标
        lines = self._route
        copy_wayline(lines[index], self._y.wayLine)  # 拷出，避免下游改写内部航线数据
        # 同时给出下一航段(末段时退化为当前段)，供曲率前馈跨段前瞻采样。
        copy_wayline(lines[min(index + 1, len(lines) - 1)], self._y.nextWayLine)
        copy_wayline(self._y.wayLine, self._cxt.wayLine)
        copy_wayline(self._y.nextWayLine, self._cxt.nextWayLine)

    def get_route(self) -> list[WayLineS]:
        """返回内部航线副本，供外部初始显示使用。注意：只读，不应由外部修改。"""
        return self._route

    def reset(self) -> None:
        """复位 LeaderRoute 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._current_index = 0

    def _select_current_index(self, self_state: MotionProfS | None) -> int:
        """选择当前应跟踪的航段下标。注意：会按待飞距和转弯提前量切换到下一段。"""
        lines = self._route
        if not lines:
            raise ValueError("route must contain at least one wayLine")
        if self_state is None:
            return self._current_index  # 无状态反馈时保持当前段不切换
        # 可能一帧跨过多段：循环前推，直到当前段不再满足切换条件或已是末段
        while self._current_index < len(lines) - 1 and _should_switch_to_next_line(
            lines[self._current_index],
            lines[self._current_index + 1],
            self_state,
        ):
            self._current_index += 1
        return self._current_index


def waypoint_inputs_to_waylines(inputs: list[WayPointInputS]) -> list[WayLineS]:
    """将原始航点转换为内部 WayLineS 序列，并按需展开圆弧几何。

    情况 1：turnSign != 0 表示外部已算好的圆弧，直接映射。
    情况 2：内部拐点 r > 0 时用 corner_arc() 计算相切圆弧。
    默认：按普通折线直连。
    """
    if len(inputs) < 2:
        raise ValueError("at least 2 waypoints required")
    nodes: list[WayPointS] = []
    for i, wpi in enumerate(inputs):
        if wpi.turnSign != 0.0:
            nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd, turnSign=wpi.turnSign, center=wpi.center))
        elif 0 < i < len(inputs) - 1 and wpi.r > 0.0:
            arc = arc_path.corner_arc(inputs[i - 1].pos, wpi.pos, inputs[i + 1].pos, wpi.r)
            if arc is not None:
                t1, t2, center, turn_sign = arc
                # 圆弧切点必须仍落在两条原始航腿内，否则保留折线。
                in_leg = _horizontal_distance(inputs[i - 1].pos, wpi.pos)
                out_leg = _horizontal_distance(wpi.pos, inputs[i + 1].pos)
                tangent_in = _horizontal_distance(t1, wpi.pos)
                tangent_out = _horizontal_distance(t2, wpi.pos)
                if tangent_in <= in_leg + 1e-9 and tangent_out <= out_leg + 1e-9:
                    nodes.append(
                        WayPointS(idx=wpi.idx, pos=t1, vdCmd=inputs[i - 1].vdCmd, turnSign=turn_sign, center=center)
                    )
                    nodes.append(WayPointS(idx=wpi.idx, pos=t2, vdCmd=wpi.vdCmd))
                else:
                    nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
            else:
                nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
        else:
            nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
    return [WayLineS(idx=j, start=nodes[j], end=nodes[j + 1]) for j in range(len(nodes) - 1)]


def _horizontal_distance(a: PosInEarthS, b: PosInEarthS) -> float:
    """计算两点水平距离。注意：圆弧切点合法性只看东/北平面。"""
    return math.hypot(a.east - b.east, a.north - b.north)


def _default_route() -> list[WayLineS]:
    """生成默认航线对象。注意：仅在外部配置缺失时作为兜底。"""
    return [_default_line()]


def _default_line() -> WayLineS:
    """生成默认单航段。注意：起终点和速度需与默认场景保持一致。"""
    return WayLineS(
        idx=0,
        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=8.0),
        end=WayPointS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0)),
    )


def _clone_wayline(src: WayLineS) -> WayLineS:
    """复制单个航段。注意：避免算法直接持有配置对象引用。"""
    dst = WayLineS()
    copy_wayline(src, dst)
    return dst


def _clone_route(src: list[WayLineS]) -> list[WayLineS]:
    """复制整条航线。注意：每个航段都需要独立复制。"""
    return [_clone_wayline(line) for line in src]


def _line_progress(line: WayLineS, self_state: MotionProfS) -> float:
    """计算飞机沿当前航段的归一化进度。注意：退化航段返回已完成。"""
    start = line.start.pos
    end = line.end.pos
    # 航段方向向量(含高度)
    dx = end.east - start.east
    dy = end.north - start.north
    dz = end.h - start.h
    length2 = dx * dx + dy * dy + dz * dz  # 航段长度平方，作投影归一化分母
    if length2 <= 0.0:
        raise ValueError("wayLine start and end must be different")
    # 本机相对起点的位移向量
    relx = self_state.pos.east - start.east
    rely = self_state.pos.north - start.north
    relz = self_state.pos.h - start.h
    # 位移在航段方向上的投影占比：0 在起点、1 在终点、>1 已越过终点
    return (relx * dx + rely * dy + relz * dz) / length2


def _should_switch_to_next_line(line: WayLineS, next_line: WayLineS, self_state: MotionProfS) -> bool:
    """判断是否切换到下一航段。注意：圆弧段及"下一段为圆弧"时按到段末(进度>=1)切，不提前。"""
    # 圆弧段：扫掠进度到末端即切，不做提前量。
    if line.start.turnSign != 0.0:
        _, _, progress, _ = arc_path.project_arc(line, self_state.pos.east, self_state.pos.north)
        return progress >= 1.0
    # 直线段：越过终点立即切。
    if _line_progress(line, self_state) >= 1.0:
        return True
    # 下一段是圆弧：圆弧切入点已是提前量，到切点(进度>=1)再切，不再额外提前。
    if next_line.start.turnSign != 0.0:
        return False
    distance_to_go = _horizontal_distance_to_go(line, self_state)
    if distance_to_go is None:
        return False  # 退化航段无法判断剩余距离，保持不切换
    # 直线->直线：保留原提前切段逻辑，让转弯起点恰好对准航段拐点。
    return distance_to_go <= _turn_switch_distance_m(line, next_line)


def _horizontal_distance_to_go(line: WayLineS, self_state: MotionProfS) -> float | None:
    """计算当前水平航段剩余待飞距。注意：退化航段返回空值。"""
    start = line.start.pos
    end = line.end.pos
    dx = end.east - start.east
    dy = end.north - start.north
    length = math.hypot(dx, dy)  # 仅取水平长度，忽略高度
    if length <= 1e-9:
        return None  # 水平退化(垂直航段)无水平待飞距
    # 航段水平单位方向(航迹方向)
    track_x = dx / length
    track_y = dy / length
    # 把"本机到终点"的水平位移投影到航迹方向，得沿航迹的剩余待飞距
    return (end.east - self_state.pos.east) * track_x + (end.north - self_state.pos.north) * track_y


def _turn_radius_m(line: WayLineS) -> float:
    """按固定滚转角估算转弯半径。注意：速度为零时半径也为零。"""
    speed = max(0.0, line.start.vdCmd)  # 用航段指令速度，钳到非负
    # 协调转弯半径公式 R = v^2 / (g*tan(bank))，滚转角固定为标称值
    return speed * speed / (_GRAVITY_MPS2 * math.tan(math.radians(_TURN_BANK_DEG)))


def _turn_switch_distance_m(line: WayLineS, next_line: WayLineS) -> float:
    """计算航段切换提前量。注意：公式为 R 乘 tan(delta_psi/2)。"""
    delta_psi = _heading_change_rad(line, next_line)  # 相邻航段航向转角
    # 几何上圆弧切入点到拐点的距离 = R*tan(转角/2)，再乘裕度系数提前切段。
    return _TURN_SWITCH_DISTANCE_SCALE * _turn_radius_m(line) * math.tan(delta_psi / 2.0)


def _heading_change_rad(line: WayLineS, next_line: WayLineS) -> float:
    """计算相邻航段水平航向夹角。注意：结果单位为弧度。"""
    current = _horizontal_unit_vector(line)
    next_track = _horizontal_unit_vector(next_line)
    if current is None or next_track is None:
        return 0.0  # 任一航段水平退化则视为无转角
    dot = max(-1.0, min(1.0, current[0] * next_track[0] + current[1] * next_track[1]))  # 点积=cos，钳防浮点越界
    cross = current[0] * next_track[1] - current[1] * next_track[0]  # 叉积=sin，带符号
    # 用 atan2(sin,cos) 取夹角并取绝对值，得 [0,pi] 的转角大小
    return abs(math.atan2(cross, dot))


def _horizontal_unit_vector(line: WayLineS) -> tuple[float, float] | None:
    """计算航段水平单位方向向量。注意：水平长度过小时返回空值。"""
    dx = line.end.pos.east - line.start.pos.east
    dy = line.end.pos.north - line.start.pos.north
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return dx / length, dy / length
