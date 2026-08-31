"""僚机航线里程编队目标计算。注意：纵向槽位表示规划航线上的里程偏置。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    FormPosS,
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    PosCalcStatusS,
    PosInEarthS,
    RallyPhaseE,
    WayLineS,
)
from src.algorithm.units.algo import arc_path
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS
from src.algorithm.units.algo.td_han import TdHan, TdHanInitS
from src.common.coordinates import FurBasis, enu_to_fur, fur_basis_from_angles


_GRAVITY_MPS2 = 9.80665
_DEFAULT_R_FORWARD = 0.8 * 6.0
_DEFAULT_R_VERTICAL = 0.8 * 6.0
_DEFAULT_R_LATERAL = 0.8 * _GRAVITY_MPS2 * math.tan(math.radians(40.0))
_PROJECTION_DISTANCE_TOLERANCE_M = 2.0
_PROJECTION_BACKTRACK_TOLERANCE_M = 5.0
_TRANSITION_VELOCITY_FF_SCALE = 0.2


@dataclass
class RouteFormationInitS(PosCalcInitS):
    """航线里程编队初始化参数。注意：route 必须是已经展开圆弧的连续航段。"""

    selfId: str = ""
    formPat: list[str] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)
    route: list[WayLineS] = field(default_factory=list)
    control_period_s: float = 0.0
    rForward: float = _DEFAULT_R_FORWARD
    rVertical: float = _DEFAULT_R_VERTICAL
    rLateral: float = _DEFAULT_R_LATERAL
    vMaxForward: float = 0.0
    vMaxVertical: float = 0.0
    vMaxLateral: float = 0.0
    catchupAltitudeM: float | None = None


@dataclass
class RouteFormationInputS:
    """航线里程编队私有输入端口。"""

    selfState: MotionProfS = field(default_factory=MotionProfS)
    leaderState: MotionProfS = field(default_factory=MotionProfS)
    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)


@dataclass
class RouteFormationOutputS:
    """航线里程编队私有输出端口。"""

    selfCmd: MotionProfS = field(default_factory=MotionProfS)
    status: PosCalcStatusS = field(default_factory=PosCalcStatusS)


@dataclass(frozen=True)
class _RouteSample:
    """规划航线指定里程处的几何样本。"""

    pos: PosInEarthS
    heading_rad: float
    height_gradient_m_m: float
    curvature_rad_m: float


class RouteFormation(PosCalcBase):
    """按公共规划航线生成僚机目标。注意：不使用长机实际位置构造空间槽位。"""

    def __init__(self) -> None:
        """建立空策略实例。"""
        # self_id 只负责从当前队形行中找本机槽位，不承担实体角色判断。
        self._self_id = ""
        # 槽位表仍沿用统一的前/上/右定义；本策略只重新解释前向量为航线里程。
        self._form_pos: list[list[FormPosS]] = []
        # route 保存已经展开圆弧的航段，运行期不得再按原始航点重复构造几何。
        self._route: list[WayLineS] = []
        # cumulative_s 的长度始终比 route 多 1，首项为 0、末项为整条航线总长。
        self._cumulative_s: list[float] = []
        self._leader_s_m: float | None = None
        # 直接 HOLD 可能从任意初始队形接入，三路 TD 在航线前/上/右坐标中完成平滑重构。
        self._transition_enabled = False
        self._transition_seeded = False
        self._td_forward = TdHan()
        self._td_vertical = TdHan()
        self._td_lateral = TdHan()
        self._catchup_altitude_m: float | None = None
        # 输入输出端口只持有黑板叶节点，避免运行期回读完整 Context。
        self._u = RouteFormationInputS()
        self._y = RouteFormationOutputS()
        self._bound = False

    def bind(self, cxt: FormContextS) -> None:
        """绑定本策略实际读取和写入的黑板端口。"""
        self._u = RouteFormationInputS(
            selfState=cxt.selfState,
            leaderState=cxt.leaderState,
            cmd=cxt.cmd,
        )
        self._y = RouteFormationOutputS(selfCmd=cxt.selfCmd, status=cxt.posCalcStatus)
        self._bound = True

    def init(self, cfg: PosCalcInitS) -> None:
        """保存槽位和规划航线。注意：航线不足一段时直接拒绝初始化。"""
        # 使用专属配置类型能阻止 Manager 漏传航线时静默退化成零目标。
        if not isinstance(cfg, RouteFormationInitS):
            raise ValueError("RouteFormation 必须使用 RouteFormationInitS 初始化")
        if not cfg.route:
            raise ValueError("RouteFormation route 不得为空")
        # 槽位行复制列表外壳，隔离运行期队形选择与外部配置容器变更。
        self._self_id = cfg.selfId
        self._form_pos = [list(row) for row in cfg.formPos]
        # WayLineS 在策略内只读，位置和圆心对象无需为每架僚机继续深拷贝。
        self._route = list(cfg.route)
        self._cumulative_s = [0.0]
        for line in self._route:
            # 统一使用水平航线里程：直线取水平长度，圆弧取半径乘扫掠角。
            length = arc_path.segment_length(line)
            if length <= 1e-9:
                raise ValueError("RouteFormation route 包含退化航段")
            # 累计表把跨航段偏置转换成一次有序区间查询。
            self._cumulative_s.append(self._cumulative_s[-1] + length)
        self._transition_enabled = cfg.control_period_s > 0.0
        self._catchup_altitude_m = cfg.catchupAltitudeM
        if self._transition_enabled:
            self._td_forward.init(
                TdHanInitS(
                    r=cfg.rForward,
                    h=cfg.control_period_s,
                    vMax=cfg.vMaxForward,
                )
            )
            self._td_vertical.init(
                TdHanInitS(
                    r=cfg.rVertical,
                    h=cfg.control_period_s,
                    vMax=cfg.vMaxVertical,
                )
            )
            self._td_lateral.init(
                TdHanInitS(
                    r=cfg.rLateral,
                    h=cfg.control_period_s,
                    vMax=cfg.vMaxLateral,
                )
            )
        self.reset()

    def step(self) -> None:
        """按长机实际位置对应的连续航线里程推进一次目标解算。"""
        if not self._bound:
            raise ValueError("RouteFormation 尚未绑定端口")
        # 第一步只解析队形配置，本拍不会修改或缩放标称槽位。
        slot = self._resolve_slot(self._u.cmd.pattern)
        # 只从长机实际位置提取一维航线进度；横向误差不会平移给僚机目标。
        leader_s = self._project_global_s(self._u.leaderState.pos)
        leader_progress_speed = self._project_progress_speed(
            self._u.leaderState,
            leader_s,
        )
        # 接入过渡只平滑航线坐标系中的槽位，不混用世界坐标轴的控制权限。
        slot_forward, slot_up, slot_right, v_forward, v_up, v_right = self._smooth_slot(slot)
        # 前向槽位跨越航段边界时由全局里程自然落到相邻航段。
        sample = self._sample_global_s(leader_s + slot_forward)
        self._write_command(
            sample,
            slot_up,
            slot_right,
            leader_progress_speed,
            self._y.selfCmd,
        )
        if (
            self._catchup_altitude_m is not None
            and self._u.cmd.stage == FormStageE.RALLY
            and self._u.cmd.step == RallyPhaseE.CATCHUP
        ):
            self._y.selfCmd.pos.h = self._catchup_altitude_m
        self._add_transition_velocity(
            sample,
            slot_up,
            slot_right,
            v_forward,
            v_up,
            v_right,
        )

    def reset(self) -> None:
        """复位连续里程和接入过渡，保留航线及 TD 配置。"""
        self._leader_s_m = None
        self._transition_seeded = False
        if self._transition_enabled:
            self._td_forward.reset()
            self._td_vertical.reset()
            self._td_lateral.reset()

    def _resolve_slot(self, pattern: int) -> FormPosS:
        """查找本机在当前队形中的槽位。"""
        row_index = int(pattern)
        if row_index < 0 or row_index >= len(self._form_pos):
            raise ValueError(f"formation pattern index out of range: {row_index}")
        slot = next((item for item in self._form_pos[row_index] if item.id == self._self_id), None)
        if slot is None:
            raise ValueError(f"missing slot for selfId: {self._self_id}")
        return slot

    def _project_global_s(self, point: PosInEarthS) -> float:
        """把长机实际位置投影成连续、单调的全局航线里程。"""
        candidates = self._projection_candidates(point)
        previous = self._leader_s_m
        if previous is None:
            # 闭合航线首尾重合时必须从最小里程起步，不能直接跳到任务终点。
            selected = _select_near_projection(candidates)
        else:
            # 排除明显回退的航段，再在近似等距候选中选择离上一拍最近的里程。
            forward = [
                item
                for item in candidates
                if item[1] >= previous - _PROJECTION_BACKTRACK_TOLERANCE_M
            ]
            selected = _select_near_projection(forward or candidates, reference_s=previous)
            selected = max(previous, selected)
        self._leader_s_m = selected
        return selected

    def _projection_candidates(self, point: PosInEarthS) -> list[tuple[float, float]]:
        """生成点对有限航段和首尾切线延拓的全部投影候选。"""
        candidates: list[tuple[float, float]] = []
        for index, line in enumerate(self._route):
            # 候选同时保存横向距离和全局里程，不能只比较各段局部进度。
            local_s, distance = _project_segment(line, point)
            candidates.append((distance, self._cumulative_s[index] + local_s))
        # 长机规划器允许越过任务首尾继续沿切线飞行，公共里程必须采用相同延拓语义。
        before_start = _project_tangent_extension(self._route[0], point, from_start=True)
        if before_start is not None:
            candidates.append(before_start)
        after_end = _project_tangent_extension(self._route[-1], point, from_start=False)
        if after_end is not None:
            distance, extra_s = after_end
            candidates.append((distance, self._cumulative_s[-1] + extra_s))
        return candidates

    def _project_progress_speed(self, state: MotionProfS, global_s: float) -> float:
        """计算实际运动映射到规划航线投影点后的水平里程速度。"""
        sample = self._sample_global_s(global_s)
        tangent_speed = (
            state.v.vEast * math.cos(sample.heading_rad)
            + state.v.vNorth * math.sin(sample.heading_rad)
        )
        total = self._cumulative_s[-1]
        if global_s < 0.0 or global_s > total:
            return max(0.0, tangent_speed)
        index = self._segment_index(global_s)
        line = self._route[index]
        if line.turnSign == 0.0:
            return max(0.0, tangent_speed)
        # 圆弧投影由实际极角决定：ds/dt=R·dθ/dt=R/rho·v_tangent。
        # 长机偏离规划半径时不能直接使用实际切向速度，否则目标位置与速度前馈不一致。
        actual_radius = math.hypot(
            state.pos.east - line.center.east,
            state.pos.north - line.center.north,
        )
        if actual_radius <= 1e-9:
            return 0.0
        return max(0.0, tangent_speed * arc_path.arc_radius(line) / actual_radius)

    def _smooth_slot(self, slot: FormPosS) -> tuple[float, float, float, float, float, float]:
        """平滑航线前/上/右槽位，返回槽位及其三轴变化率。"""
        if not self._transition_enabled:
            return slot.x, slot.y, slot.z, 0.0, 0.0, 0.0
        if not self._transition_seeded:
            leader_s = self._leader_s_m if self._leader_s_m is not None else 0.0
            self_s = _select_near_projection(
                self._projection_candidates(self._u.selfState.pos),
                reference_s=leader_s,
            )
            # 水平投影会把 FUR 上法向的前向分量误计入航线里程；沿三维前轴迭代消除该分量。
            for _ in range(3):
                self_sample = self._sample_global_s(self_s)
                basis = _sample_fur_basis(self_sample)
                rel = (
                    self._u.selfState.pos.east - self_sample.pos.east,
                    self._u.selfState.pos.north - self_sample.pos.north,
                    self._u.selfState.pos.h - self_sample.pos.h,
                )
                rel_forward, _rel_up, _rel_right = enu_to_fur(rel, basis)
                horizontal_forward = math.hypot(basis[0][0], basis[0][1])
                correction = rel_forward * horizontal_forward
                self_s += correction
                if abs(correction) <= 1e-9:
                    break
            self_sample = self._sample_global_s(self_s)
            rel = (
                self._u.selfState.pos.east - self_sample.pos.east,
                self._u.selfState.pos.north - self_sample.pos.north,
                self._u.selfState.pos.h - self_sample.pos.h,
            )
            _rel_forward, seed_up, seed_right = enu_to_fur(
                rel,
                _sample_fur_basis(self_sample),
            )
            # 前向初值必须使用本机与长机的航线里程差；切线弦长在圆弧上并不等于弧长。
            seed_forward = self_s - leader_s
            self._td_forward.seed(seed_forward, 0.0)
            # 上向和右向初值必须与正常目标使用同一三维 FUR，避免接管首拍坐标轴跳变。
            self._td_vertical.seed(seed_up, 0.0)
            self._td_lateral.seed(seed_right, 0.0)
            self._transition_seeded = True
        smooth_forward, v_forward = self._td_forward.step(slot.x)
        smooth_up, v_up = self._td_vertical.step(slot.y)
        smooth_right, v_right = self._td_lateral.step(slot.z)
        return smooth_forward, smooth_up, smooth_right, v_forward, v_up, v_right

    def _add_transition_velocity(
        self,
        sample: _RouteSample,
        slot_up: float,
        slot_right: float,
        v_forward: float,
        v_up: float,
        v_right: float,
    ) -> None:
        """把槽位变化率按航线切向、上向和右向叠加到速度前馈。"""
        if not self._transition_enabled:
            return
        output = self._y.selfCmd
        forward_rate = _TRANSITION_VELOCITY_FF_SCALE * v_forward
        up_rate = _TRANSITION_VELOCITY_FF_SCALE * v_up
        right_rate = _TRANSITION_VELOCITY_FF_SCALE * v_right
        basis = _sample_fur_basis(sample)
        up_axis = basis[1]
        sin_theta = basis[0][2]
        tangent_e = math.cos(sample.heading_rad)
        tangent_n = math.sin(sample.heading_rad)
        right_e = math.sin(sample.heading_rad)
        right_n = -math.cos(sample.heading_rad)
        # 前向槽位变化等价于增加公共航线里程速度；横向偏置曲线还需乘 1+κz。
        forward_speed = forward_rate * (1.0 + sample.curvature_rad_m * slot_right)
        # 爬升转弯时，上法向的水平分量随航向旋转，沿里程变化会产生额外右向速度。
        forward_right_speed = (
            forward_rate * sample.curvature_rad_m * slot_up * sin_theta
        )
        total_right_speed = right_rate + forward_right_speed
        output.v.vEast += (
            forward_speed * tangent_e
            + total_right_speed * right_e
            + up_rate * up_axis[0]
        )
        output.v.vNorth += (
            forward_speed * tangent_n
            + total_right_speed * right_n
            + up_rate * up_axis[1]
        )
        # 航线里程变化沿坡度推进，slot.y 的变化率沿三维 FUR 上法向叠加。
        output.v.vUp += (
            forward_rate * sample.height_gradient_m_m + up_rate * up_axis[2]
        )
        output.v.vd = math.hypot(output.v.vEast, output.v.vNorth)
        output.v.vPsi = math.atan2(output.v.vNorth, output.v.vEast) if output.v.vd > 0.0 else 0.0
        output.v.dVPsi += sample.curvature_rad_m * forward_rate

    def _sample_global_s(self, global_s: float) -> _RouteSample:
        """读取整条航线指定里程处的几何；首尾以端点切线延拓。"""
        # 后方槽位在任务初始阶段可能落到航线起点之前，按首段切线连续延拓。
        if global_s < 0.0:
            return _extended_sample(self._route[0], global_s, from_start=True)
        total = self._cumulative_s[-1]
        # 前方槽位越过任务终点时同样沿末段切线延拓，避免目标突然钳死在终点。
        if global_s > total:
            return _extended_sample(self._route[-1], global_s - total, from_start=False)
        # 边界默认归上一段；下一拍超过边界后自然进入下一段，不产生位置跳变。
        index = self._segment_index(global_s)
        return _sample_segment(self._route[index], global_s - self._cumulative_s[index])

    def _segment_index(self, global_s: float) -> int:
        """返回闭区间全局里程所属航段；边界默认归前一段。"""
        for candidate in range(len(self._route)):
            if global_s <= self._cumulative_s[candidate + 1] + 1e-9:
                return candidate
        return len(self._route) - 1

    @staticmethod
    def _write_command(
        sample: _RouteSample,
        slot_up: float,
        slot_right: float,
        progress_speed_mps: float,
        output: MotionProfS,
    ) -> None:
        """把航线样本和横/垂槽位写成自洽的位置速度指令。"""
        basis = _sample_fur_basis(sample)
        up_axis = basis[1]
        right_axis = basis[2]
        output.pos.east = (
            sample.pos.east + slot_up * up_axis[0] + slot_right * right_axis[0]
        )
        output.pos.north = (
            sample.pos.north + slot_up * up_axis[1] + slot_right * right_axis[1]
        )
        output.pos.h = sample.pos.h + slot_up * up_axis[2]
        # 平行偏置曲线在相同公共里程相位下需要按 1+κz 配平物理速度；本报告的一字纵队 z=0。
        speed_scale = 1.0 + sample.curvature_rad_m * slot_right
        if speed_scale <= 0.0:
            # 1+κz<=0 表示偏置线到达或越过瞬时曲率中心，几何上不可作为同向平行航线。
            raise ValueError("RouteFormation 横向槽位越过曲率中心")
        # 目标里程由长机实际进度驱动，因此速度前馈也必须使用同一个里程速度；
        # 不能改用目标点所在航段的局部标称速度，否则跨变速边界时位置和速度指令互相矛盾。
        physical_speed = progress_speed_mps * speed_scale
        # 爬升转弯时，上法向的水平分量随航向旋转，固定 slot.y 也会产生右向速度。
        up_turn_speed = (
            progress_speed_mps
            * sample.curvature_rad_m
            * slot_up
            * basis[0][2]
        )
        output.v.vEast = (
            physical_speed * math.cos(sample.heading_rad)
            + up_turn_speed * right_axis[0]
        )
        output.v.vNorth = (
            physical_speed * math.sin(sample.heading_rad)
            + up_turn_speed * right_axis[1]
        )
        output.v.vUp = progress_speed_mps * sample.height_gradient_m_m
        output.v.vd = math.hypot(output.v.vEast, output.v.vNorth)
        # 水平速度非零时直接采用航线切向，避免再次由带垂向速度的三维量反解航向。
        output.v.vPsi = sample.heading_rad if output.v.vd > 0.0 else 0.0
        # 平行偏置轨迹的曲率为 κ/(1+κz)，与配平速度相乘后角速率仍为 κV。
        output.v.dVPsi = sample.curvature_rad_m * progress_speed_mps


def _sample_fur_basis(sample: _RouteSample) -> FurBasis:
    """按航线水平航向和高度坡度建立完整三维制导 FUR。"""
    return fur_basis_from_angles(
        math.atan(sample.height_gradient_m_m),
        sample.heading_rad,
    )


def _select_near_projection(
    candidates: list[tuple[float, float]],
    reference_s: float | None = None,
) -> float:
    """在近似等距候选中选择最小里程或最接近参考里程的投影。"""
    min_distance = min(distance for distance, _s in candidates)
    near = [
        item
        for item in candidates
        if item[0] <= min_distance + _PROJECTION_DISTANCE_TOLERANCE_M
    ]
    if reference_s is None:
        return min(near, key=lambda item: item[1])[1]
    return min(near, key=lambda item: abs(item[1] - reference_s))[1]


def _project_segment(line: WayLineS, point: PosInEarthS) -> tuple[float, float]:
    """返回点在单航段上的投影里程和水平投影距离。"""
    if line.turnSign != 0.0:
        # 圆弧工具会把角度投影钳在有限扫掠范围内，并按转向输出正向弧长。
        projected, local_s, _progress, _heading = arc_path.project_arc(
            line, point.east, point.north
        )
    else:
        # 直线段用水平点积求投影；高度不参与公共航线里程的归属判断。
        dx = line.end.east - line.start.east
        dy = line.end.north - line.start.north
        length2 = dx * dx + dy * dy
        if length2 <= 1e-18:
            raise ValueError("RouteFormation route 包含退化航段")
        progress = max(
            0.0,
            min(1.0, ((point.east - line.start.east) * dx + (point.north - line.start.north) * dy) / length2),
        )
        # 投影钳在有限航段上，段外点由相邻航段候选负责竞争。
        projected = PosInEarthS(
            line.start.east + progress * dx,
            line.start.north + progress * dy,
            line.start.h + progress * (line.end.h - line.start.h),
        )
        local_s = progress * math.sqrt(length2)
    return local_s, math.hypot(point.east - projected.east, point.north - projected.north)


def _project_tangent_extension(
    line: WayLineS,
    point: PosInEarthS,
    *,
    from_start: bool,
) -> tuple[float, float] | None:
    """把点投影到首端向后或末端向前的切线延长线。"""
    local_s = 0.0 if from_start else arc_path.segment_length(line)
    endpoint = _sample_segment(line, local_s)
    rel_e = point.east - endpoint.pos.east
    rel_n = point.north - endpoint.pos.north
    tangent_e = math.cos(endpoint.heading_rad)
    tangent_n = math.sin(endpoint.heading_rad)
    along = rel_e * tangent_e + rel_n * tangent_n
    if (from_start and along >= 0.0) or (not from_start and along <= 0.0):
        return None
    lateral = abs(rel_e * tangent_n - rel_n * tangent_e)
    return lateral, along


def _sample_segment(line: WayLineS, local_s: float) -> _RouteSample:
    """读取单航段指定水平里程处的位置、切向速度和曲率。"""
    length = arc_path.segment_length(line)
    # 调用方处理整条航线首尾延拓，段内采样只允许落在闭区间。
    local_s = max(0.0, min(length, local_s))
    progress = local_s / length
    heading = arc_path.heading_at_s(line, local_s)
    if line.turnSign != 0.0:
        # 航向是圆弧切向，按转向旋回 90°即可恢复对应径向角。
        radius = arc_path.arc_radius(line)
        sign = 1.0 if line.turnSign > 0.0 else -1.0
        radial = heading - sign * math.pi / 2.0
        pos = PosInEarthS(
            line.center.east + radius * math.cos(radial),
            line.center.north + radius * math.sin(radial),
            line.start.h + progress * (line.end.h - line.start.h),
        )
        # 曲率符号与项目偏航角速率约定一致：左转为正、右转为负。
        curvature = sign / radius
    else:
        # 直线位置和高度都按同一个水平里程进度线性插值。
        pos = PosInEarthS(
            line.start.east + progress * (line.end.east - line.start.east),
            line.start.north + progress * (line.end.north - line.start.north),
            line.start.h + progress * (line.end.h - line.start.h),
        )
        curvature = 0.0
    # 高度坡度独立于速度保存，僚机垂向前馈随后统一乘长机实际里程速度。
    height_gradient = (line.end.h - line.start.h) / length
    return _RouteSample(pos, heading, height_gradient, curvature)


def _extended_sample(line: WayLineS, extra_s: float, *, from_start: bool) -> _RouteSample:
    """沿首段起点或末段终点的切线延拓航线样本。"""
    # 起点延拓接收负 extra_s，终点延拓接收正 extra_s，两者共用同一切线表达式。
    local_s = 0.0 if from_start else arc_path.segment_length(line)
    endpoint = _sample_segment(line, local_s)
    # 延拓区不再声称具有原圆弧曲率，防止越过任务端点后继续输出转弯前馈。
    return _RouteSample(
        PosInEarthS(
            endpoint.pos.east + extra_s * math.cos(endpoint.heading_rad),
            endpoint.pos.north + extra_s * math.sin(endpoint.heading_rad),
            endpoint.pos.h + extra_s * endpoint.height_gradient_m_m,
        ),
        endpoint.heading_rad,
        endpoint.height_gradient_m_m,
        0.0,
    )
