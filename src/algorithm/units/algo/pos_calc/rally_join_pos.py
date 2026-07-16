"""集结汇合位置解算：待命盘旋 → 公切线转移 → 集结圆盘旋 → 切出。

每架飞机独立运行此模块，无长机/僚机之分。
四个阶段：
  STANDBY   — 在本机当前位置按当前航向反推本地待命圆，沿圆盘旋等待外部开始集结指令
  FLYING    — 开始集结时锁存待命圆到集结圆的 CCW 公切线和基础航程；先沿待命圆飞到切出窗口，
               再沿公切线直飞集结圆切入点 T，并随阶段推进更新到下一次 M_i 的剩余航程
  LOITERING — 沿盘旋圆做 CCW 圆弧飞行；每次路过松散点 M_i（圆上固定的切出点）时评估是否切出
  EXITED    — 从松散点沿任务航向直飞，交由编队控制接管

盘旋圆的圆心和切出点 M_i 在 init 时就按任务航向定死：M_i 是松散槽位，圆心摆在任务航向左侧 R 处，
使 M_i 处的 CCW 切线方向恒等于任务航向——不管飞机从哪个方向飞来，只要沿圆弧飞到 M_i 就必然对齐任务航向，
切出瞬间的指令不会因为到达方向不同而发生跳变（这也是为什么切入点 T 要专门算一条切线，
而不是像旧版那样直飞 M_i 再原地盘旋：直飞 M_i 时的到达航向是任意的，会让盘旋圆摆歪）。
圆半径固定（loiter_radius_m），通过调整盘旋速度改变盘旋周期以匹配 T_ref；
固定计划为零圈时，飞机完成远区布防并第一次真实越过 M_i 后即可切出，不再依赖时间阈值。

LOITERING 阶段的位置指令是"期望半径圆上、飞机当前角度处的投影点"（不是圆心），向心加速度前馈也用
期望半径而非实时半径——这样飞机的实际盘旋半径才会收敛到 loiter_radius_m 本身；若指令目标点是圆心，
位置误差（侧偏）恒等于飞机此刻的实际半径，跟期望半径无关，控制律没有把半径拉回期望值的趋势，
实测会在很宽的范围内漂移（同一场景下从 23m 到 222m 都出现过，期望是 200m）。

公切线无解时退化为开始集结那一拍的当前位置到集结圆切线；若该位置落在集结圆内部或圆上，
点到圆切线同样无解，则继续退化为直飞 M_i。待命圆与集结圆圆心近零时直接进入 LOITERING。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FormCommInitS,
    FormPosS,
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    PosCalcStatusS,
    PosCalcStrategyE,
    PosInEarthS,
    PosTrackCommandE,
    PosTrackCommandS,
    RallyPlanS,
    WayPointInputS,
    copy_motion,
    copy_snapshot,
)
from src.algorithm.units.algo.arc_path import common_tangent
from src.algorithm.units.algo.formation_math import clamp, horizontal_track_vector_to_enu
from src.algorithm.units.algo.pos_calc.base import (
    PosCalcBase,
    PosCalcInitS,
    copy_pos_calc_status,
    reset_pos_calc_status,
)

if TYPE_CHECKING:
    from src.algorithm.entity.types import VelCmdLimitS

_TWO_PI = 2.0 * math.pi
_MIN_FIRST_SEGMENT_HORIZ_M = 1e-6
_DEFAULT_LOITER_SPEED_MIN_MPS = 14.0
_DEFAULT_LOITER_SPEED_MAX_MPS = 25.0

RALLY_STATE_FLYING = "FLYING"
RALLY_STATE_LOITERING = "LOITERING"
RALLY_STATE_EXITED = "EXITED"
RALLY_STATE_STANDBY = "STANDBY"

_EPSILON_HORIZ = 0.5  # 水平近零距离阈值，米
_SLOT_ANG_NEAR = 0.35  # ≈20°：确认已进入 M_i 点前近区的角度窗口（不依赖轨道半径）
_SLOT_ANG_AWAY = 1.05  # ≈60°：判定"已远离 M_i"的角度阈值
# 切入圆弧（FLYING→LOITERING）触发半径 d 与航向跳变角 ψ 的几何关系是 ψ = atan(d/R)（R=loiter_radius_m，
# 见 _compute_arc_capture_radius_m 推导），所以固定距离上限只对某一个 R 好使——R 越小，同样的 d 换算出的
# ψ 越大。改成按 R 反解 d = R·tan(ψ_max)，跳变角上限跟 R 无关，恒定在 ψ_max 附近。
_MAX_ARC_CAPTURE_HEADING_JUMP_RAD = math.radians(5.0)  # 允许的 FLYING→LOITERING 指令航向跳变上限
_MIN_ARC_CAPTURE_RADIUS_M = 0.5  # 触发半径下限，避免 loiter_radius_m 很小时算出不现实的亚米级容差
# 触发半径至少要能跨过这么多个控制周期的飞行距离，否则离散步进可能整拍跨过捕获窗口、错过 T 而
# 永远飞不进 LOITERING（FLYING 阶段直飞 T 是匀速直线，d_3d 每拍减少约 flying_speed*control_period_s）。
_MIN_ARC_CAPTURE_STEP_MARGIN = 3.0
# 待命圆提前 10° 切向公切线，把切出航向差限制在约 10°；离散跨越另由上一拍剩余角补判。
_LOCAL_TANGENT_CAPTURE_ANGLE_RAD = math.radians(10.0)
_TRANSIT_ARC_TO_TANGENT = "ARC_TO_TANGENT"
_TRANSIT_LINE_TO_RALLY_ENTRY = "LINE_TO_RALLY_ENTRY"


def loiter_speed_bounds(vel_cmd_limit: VelCmdLimitS) -> tuple[float, float]:
    """从速度权限推导盘旋速度上下限。注意：只配置单侧造成反序时明确拒绝。"""
    fwd_min = vel_cmd_limit.forwardMin
    fwd_max = vel_cmd_limit.forwardMax
    loiter_min = fwd_min if math.isfinite(fwd_min) and fwd_min > 0 else _DEFAULT_LOITER_SPEED_MIN_MPS
    loiter_max = fwd_max if math.isfinite(fwd_max) and fwd_max > 0 else _DEFAULT_LOITER_SPEED_MAX_MPS
    if loiter_max <= loiter_min:
        raise ValueError(
            f"loiter_speed_bounds: 推导出的盘旋速度上下限非法（min={loiter_min}, max={loiter_max}）："
            "velCmdLimit.forwardMin/forwardMax 只显式配置一侧、另一侧退回默认值（14/25 m/s）时，"
            "两者可能反序；请同时显式配置一对自洽的 forwardMin/forwardMax，或都不配置以使用默认值"
        )
    return loiter_min, loiter_max


def route_heading_rad(route: list[WayPointInputS]) -> float:
    """由航线第一航段计算集结航向。注意：水平退化航段不能定义航向。"""
    a = route[0].pos
    a1 = route[1].pos
    d_e = a1.east - a.east
    d_n = a1.north - a.north
    if math.hypot(d_e, d_n) < _MIN_FIRST_SEGMENT_HORIZ_M:
        raise ValueError(
            "route 第一航段水平长度退化为零（A/A1 水平坐标重合，仅高度不同也算）："
            "无法据此推导集结航向，请检查 route 前两个航点"
        )
    return math.atan2(d_n, d_e)


def rally_loose_target(route_start: PosInEarthS, heading_rad: float, scale: float, slot: FormPosS) -> PosInEarthS:
    """计算本机松散集结点。注意：高度差不随 looseScale 放大。"""
    east_off, north_off = horizontal_track_vector_to_enu(
        (slot.x, slot.z),
        (math.cos(heading_rad), math.sin(heading_rad)),
    )
    return PosInEarthS(
        east=route_start.east + scale * east_off,
        north=route_start.north + scale * north_off,
        h=route_start.h + slot.y,
    )


def resolve_formation_slot(comm_init: FormCommInitS, target_pattern: int, node_id: str) -> FormPosS | None:
    """按队形索引和节点标识查找槽位。注意：索引越界或缺项时返回 None。"""
    if not 0 <= target_pattern < len(comm_init.formPos):
        return None
    return next((slot for slot in comm_init.formPos[target_pattern] if slot.id == node_id), None)


def validate_capture_geometry(
    *,
    loiter_radius_m: float,
    arrival_radius_m: float,
    approach_speed_mps: float,
    loiter_speed_min_mps: float,
    control_period_s: float,
) -> None:
    """校验切入圆弧（FLYING→LOITERING）相关几何/时序参数的合法性。不合法抛 ValueError。

    供 `RallyJoinPos.init()` 和 `_ConfigLoader.validate()` 共用，避免"配置校验阶段查不出、
    要等到实际构造实体才报错"的分叉（同一套参数应该在两处得到同一个结论）。

    运行时实际生效的触发半径是 `min(arrival_radius_m, arc_capture_radius_m)`
    （`arc_capture_radius_m = max(_MIN_ARC_CAPTURE_RADIUS_M, loiter_radius_m·tan(ψ_max))`，
    见 `_step_flying` 触发条件），只校验 `loiter_radius_m` 反解出的 `arc_capture_radius_m` 不够——
    `arrival_radius_m` 配得比它还小时，实际生效的仍是更小的 `arrival_radius_m`，同样会被压穿。
    """
    if loiter_radius_m <= 0:
        raise ValueError("loiter_radius_m must be > 0")
    if arrival_radius_m <= 0:
        raise ValueError("arrival_radius_m must be > 0")
    if approach_speed_mps <= 0:
        raise ValueError("approach_speed_mps must be > 0")
    if control_period_s <= 0:
        raise ValueError("control_period_s must be > 0")

    # FLYING 阶段的下限速度是 max(approach_speed_mps, loiter_speed_min_mps)：近场按 slow_radius_m
    # 减速时地板是 loiter_speed_min_mps；若配置里 loiter_speed_min_mps 比 approach_speed_mps 还大，
    # 真实步进距离由它决定，只用 approach_speed_mps 算会低估最坏情况的单拍步进距离。
    worst_case_speed_mps = max(approach_speed_mps, loiter_speed_min_mps)
    required_capture_radius_m = max(
        _MIN_ARC_CAPTURE_RADIUS_M,
        _MIN_ARC_CAPTURE_STEP_MARGIN * worst_case_speed_mps * control_period_s,
    )

    # 1) loiter_radius_m 下限：保证 arc_capture_radius_m（若它是生效值）同时满足跳变角上限和步进安全。
    min_loiter_radius_m = required_capture_radius_m / math.tan(_MAX_ARC_CAPTURE_HEADING_JUMP_RAD)
    if loiter_radius_m < min_loiter_radius_m:
        raise ValueError(
            f"loiter_radius_m={loiter_radius_m:.2f} 太小：按 approach_speed_mps={approach_speed_mps}、"
            f"loiter_speed_min_mps={loiter_speed_min_mps}、control_period_s={control_period_s} 反推，"
            f"至少需要 {min_loiter_radius_m:.2f}m，否则切入圆弧的触发半径会被地板值或离散步进距离压过 "
            "5° 航向跳变角上限，或触发窗口比每步飞行距离还窄，可能整拍跨过捕获区而错过切入"
        )

    # 2) arrival_radius_m 下限：即便 loiter_radius_m 本身合法，arrival_radius_m 配得更小时，
    #    min(arrival_radius_m, arc_capture_radius_m) 的生效值仍会被它压到不安全的宽度。
    #    （生效值更小不会让跳变角超标——距离越小、跳变角越小，隐患只在离散步进安全这一侧。）
    if arrival_radius_m < required_capture_radius_m:
        raise ValueError(
            f"arrival_radius_m={arrival_radius_m:.2f} 太小：按 approach_speed_mps={approach_speed_mps}、"
            f"loiter_speed_min_mps={loiter_speed_min_mps}、control_period_s={control_period_s} 反推，"
            f"至少需要 {required_capture_radius_m:.2f}m，否则触发窗口比每步飞行距离还窄——"
            "飞机可能整拍跨过捕获区，冻结在'离 T 一点点但始终不够近'的位置上，永远进不了 LOITERING"
        )


@dataclass
class RallyJoinPosInitS(PosCalcInitS):
    """RallyJoinPos 初始化参数。"""

    self_id: str = ""  # 本机节点 ID，用于从公共计划圈数映射读取本机分配
    loose_slot: PosInEarthS = field(default_factory=PosInEarthS)  # 本机固定松散目标点 M_i（ENU 全局坐标），同时是盘旋圆上的切出点
    approach_speed_mps: float = 20.0  # 飞向切入点时的速度
    slow_radius_m: float = 0.0  # 近场降速半径；>0 时在此范围内线性减速
    arrival_radius_m: float = 100.0  # 到达切入点、转入圆弧飞行的触发距离
    loiter_radius_m: float = 200.0  # 盘旋圆半径（固定）
    loiter_speed_min_mps: float = 14.0  # 最小盘旋速度（固定翼速度下限）
    loiter_speed_max_mps: float = 25.0  # 最大盘旋速度
    mission_heading_rad: float = 0.0  # 切出后的飞行方向（弧度，从东向起算）
    mission_speed_mps: float = 20.0  # 切出后的飞行速度
    v_up_min_mps: float = -3.0  # 天向速度下限（来自 velCmdLimit.verticalMin，兜底 −3 m/s）
    v_up_max_mps: float = 3.0   # 天向速度上限（来自 velCmdLimit.verticalMax，兜底 +3 m/s）
    control_period_s: float = 0.05  # 控制周期，用于校验 loiter_radius_m 下限（避免离散步进跨过捕获窗口）
    standby_altitude_m: float | None = None  # 本地待命目标高度；None 表示保持进入待命首帧高度


@dataclass
class RallyJoinPosInputS:
    """集结位置解算内部输入快照。注意：只包含本策略实际读取的数据。"""

    selfState: MotionProfS = field(default_factory=MotionProfS)
    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)
    clock: AlgorithmClockS = field(default_factory=AlgorithmClockS)
    rallyPlan: RallyPlanS = field(default_factory=RallyPlanS)


@dataclass
class RallyJoinPosOutputS:
    """集结位置解算内部输出快照。注意：计算成功后统一提交到黑板。"""

    selfCmd: MotionProfS = field(default_factory=MotionProfS)
    status: PosCalcStatusS = field(default_factory=PosCalcStatusS)
    posTrackCommand: PosTrackCommandS = field(default_factory=PosTrackCommandS)


class RallyJoinPos(PosCalcBase):
    """集结汇合位置解算器，提供锁存的基础航程及其当前剩余值。"""

    def __init__(self) -> None:
        """建立内部快照。注意：具体算法配置仍由 init 完成。"""
        # 黑板引用只允许在读取和提交边界使用。
        self._cxt: FormContextS | None = None
        # 输入输出对象跨帧复用，运行期不产生端口临时对象。
        self._u = RallyJoinPosInputS()
        self._y = RallyJoinPosOutputS()
        self._empty_cmd = MotionProfS()

    def bind(self, cxt: FormContextS) -> None:
        """绑定黑板。注意：运行时只通过读取和提交函数访问黑板。"""
        # Manager只负责转交引用，不了解本策略读取哪些字段。
        self._cxt = cxt

    def init(self, cfg: RallyJoinPosInitS) -> None:
        """按配置初始化 RallyJoinPos。"""
        if cfg.approach_speed_mps <= 0:
            raise ValueError("approach_speed_mps must be > 0")
        if cfg.loiter_speed_min_mps <= 0 or cfg.loiter_speed_max_mps <= cfg.loiter_speed_min_mps:
            raise ValueError("loiter speed limits invalid")
        validate_capture_geometry(
            loiter_radius_m=cfg.loiter_radius_m,
            arrival_radius_m=cfg.arrival_radius_m,
            approach_speed_mps=cfg.approach_speed_mps,
            loiter_speed_min_mps=cfg.loiter_speed_min_mps,
            control_period_s=cfg.control_period_s,
        )

        self._slot = cfg.loose_slot
        self._self_id = cfg.self_id
        self._approach_speed = cfg.approach_speed_mps
        self._slow_radius_m = cfg.slow_radius_m
        self._arrival_radius_m = cfg.arrival_radius_m
        self._loiter_radius = cfg.loiter_radius_m
        self._speed_min = cfg.loiter_speed_min_mps
        self._speed_max = cfg.loiter_speed_max_mps
        self._mission_heading = cfg.mission_heading_rad
        self._mission_speed = cfg.mission_speed_mps
        self._v_up_min = cfg.v_up_min_mps
        self._v_up_max = cfg.v_up_max_mps
        self._standby_altitude_m = cfg.standby_altitude_m
        # 切入圆弧触发半径按 loiter_radius_m 反解，保证 FLYING→LOITERING 航向跳变角恒定在
        # _MAX_ARC_CAPTURE_HEADING_JUMP_RAD 附近，不随 loiter_radius_m 大小变化（见模块常量注释）；
        # 上面的下限校验已保证这里恒被 R·tan(ψ_max) 一项主导，max() 只是防御性兜底。
        self._arc_capture_radius_m = max(
            _MIN_ARC_CAPTURE_RADIUS_M,
            self._loiter_radius * math.tan(_MAX_ARC_CAPTURE_HEADING_JUMP_RAD),
        )

        # 盘旋圆几何在 init 时按任务航向定死：M_i 在圆上，圆心摆在任务航向左侧 R 处（CCW 盘旋），
        # 使 M_i 处的切线方向恒等于任务航向，与飞机到达方向无关。
        r = self._loiter_radius
        self._loiter_center_e = self._slot.east - r * math.sin(self._mission_heading)
        self._loiter_center_n = self._slot.north + r * math.cos(self._mission_heading)
        d_e = self._slot.east - self._loiter_center_e
        d_n = self._slot.north - self._loiter_center_n
        self._theta_slot = math.atan2(d_n, d_e)  # M_i 在盘旋圆上的角度（弧度），固定不变

        self._state: str = RALLY_STATE_FLYING
        self._planned_path_length_m: float = -1.0
        self._remaining_path_length_m: float = -1.0
        self._plan_applied: bool = False
        self._assigned_loops: int = 0
        self._remaining_loops: int = 0
        self._loiter_speed: float = cfg.approach_speed_mps
        self._standby_speed: float = cfg.approach_speed_mps
        self._standby_center_e: float | None = None
        self._standby_center_n: float | None = None
        self._standby_target_h: float | None = None
        # 公切线计划只在离开待命时生成一次，之后均读取锁存几何，避免目标随实时位置漂移。
        self._transit_phase: str | None = None
        self._local_exit_point: PosInEarthS | None = None
        self._theta_local_exit: float = 0.0
        self._tangent_length_m: float = 0.0
        # 剩余角用于识别离散步进是否跨过 0° 切点，防止漏过窗口后多绕一圈。
        self._last_local_remaining_angle: float | None = None
        self._away_from_slot: bool = False  # 盘旋后是否已远离松散点（防止立即切出）
        # 点前近窗只负责 armed，上一拍有向剩余角负责确认下一拍是否真正回绕。
        self._last_slot_remaining_angle: float | None = None
        self._slot_near_window_armed: bool = False
        self._entry_point: PosInEarthS | None = None  # 集结圆切入点 T；开始集结时一次性规划并锁存
        self._theta_entry: float = 0.0  # 切入点在盘旋圆上的角度，供基础航程计算
        self._reached_slot_once: bool = False  # 是否已至少一次路过 M_i，保留为汇合过程诊断量

    @property
    def state(self) -> str:
        """返回当前汇合子状态。"""
        return self._state

    @property
    def planned_path_length_m(self) -> float:
        """返回开始集结时锁存的不含额外整圈的基础水平航程。"""
        return self._planned_path_length_m

    @property
    def remaining_path_length_m(self) -> float:
        """返回沿锁存路线到下一次 M_i 的水平剩余航程。"""
        return self._remaining_path_length_m

    @property
    def remaining_loops(self) -> int:
        """返回固定计划尚未消耗的整圈数量。"""
        return self._remaining_loops

    @property
    def reached_slot_once(self) -> bool:
        """返回是否已至少一次路过松散点 M_i，供汇合过程诊断。"""
        return self._reached_slot_once

    def step(
        self,
        u: RallyJoinPosInputS | None = None,
        y: RallyJoinPosOutputS | None = None,
    ) -> None:
        """推进集结解算。注意：无参模式使用内部快照，显式端口仅兼容既有低层调用。"""
        if u is None and y is None:
            # 新实体使用事务式读取、计算、提交路径。
            self._read_context()
            # 输出先恢复默认值，避免复杂状态分支漏写时沿用上一拍。
            self._reset_output()
            self._calculate(self._u, self._y)
            self._write_context()
            return
        if u is None or y is None:
            # 显式兼容调用同样必须保持输入输出成对。
            raise ValueError("RallyJoinPos 输入输出端口必须同时提供")
        self._calculate(u, y)
        self._fill_common_output(y)

    def _calculate(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """使用内部端口完成算法计算。注意：本方法不访问黑板。"""
        if u.selfState is None or u.cmd is None or y.selfCmd is None:
            raise ValueError("RallyJoinPos ports must be bound")
        # 从这里到返回只允许访问策略输入输出和内部跨帧状态。
        # 实体会在固定计划生效后每拍重复发送同一圈数；这里只接受首个有效值，
        # 避免飞机已经消耗的圈数被后续重复报文恢复。
        plan = u.rallyPlan
        clock = u.clock
        plan_enabled = u.cmd.stage != FormStageE.STANDBY and plan is not None and plan.valid
        if plan_enabled and not self._plan_applied:
            # 首次锁存必须先完成全部值域校验，任何失败都不能留下半套计划状态。
            if clock is None or not math.isfinite(plan.t_ref) or not math.isfinite(clock.now_s):
                raise ValueError("首次有效计划的 t_ref 和 now_s 必须为有限数")
            assigned_loops = plan.loop_counts.get(self._self_id, 0)
            # bool 是 int 的子类，必须单独排除；浮点圈数不得通过转换静默截断。
            if (
                not isinstance(assigned_loops, int)
                or isinstance(assigned_loops, bool)
                or assigned_loops < 0
            ):
                raise ValueError("assigned_loops 必须为非 bool 的非负整数")
            self._assigned_loops = assigned_loops
            self._remaining_loops = self._assigned_loops
            self._plan_applied = True
        if u.cmd.stage == FormStageE.STANDBY:
            if self._state != RALLY_STATE_STANDBY:
                self._enter_standby(u)
            self._step_standby(u, y)
        else:
            if self._state == RALLY_STATE_STANDBY:
                self._leave_standby(u)
            if self._state == RALLY_STATE_FLYING:
                self._step_flying(u, y)
            elif self._state == RALLY_STATE_LOITERING:
                self._step_loitering(u, y)
            else:
                self._step_exited(u, y)
        # 指令阶段可能在本拍切换；已有锁存规划时统一在阶段推进后读取当前路线。
        if self._planned_path_length_m >= 0.0:
            self._remaining_path_length_m = self._remaining_base_path_m(u.selfState.pos)

    def _read_context(self) -> None:
        """从黑板生成本拍输入快照。"""
        if self._cxt is None:
            raise ValueError("RallyJoinPos 尚未绑定黑板")
        # 运动状态和任务指令按值复制，算法无法反向修改黑板输入。
        copy_motion(self._cxt.selfState, self._u.selfState)
        copy_snapshot(self._cxt.cmd, self._u.cmd)
        self._u.clock.now_s = self._cxt.clock.now_s
        self._u.rallyPlan.t_ref = self._cxt.rallyPlan.t_ref
        self._u.rallyPlan.valid = self._cxt.rallyPlan.valid
        # 圈数映射是可变对象，必须清空后复制而不是共享字典引用。
        self._u.rallyPlan.loop_counts.clear()
        self._u.rallyPlan.loop_counts.update(self._cxt.rallyPlan.loop_counts)

    def _reset_output(self) -> None:
        """清理内部输出，避免未覆盖字段沿用上一拍。"""
        # 默认运动剖面在构造期建立，避免每拍重新分配MotionProfS。
        copy_motion(self._empty_cmd, self._y.selfCmd)
        self._fill_common_output(self._y)

    def _fill_common_output(self, y: RallyJoinPosOutputS) -> None:
        """完整填写集结策略公共及专有状态。"""
        if y.status is not None:
            # Rally拥有全部集结专有字段，因此每拍完整覆盖对应诊断。
            reset_pos_calc_status(y.status, PosCalcStrategyE.RALLY_JOIN)
            self._write_rally_status(y.status)
        if y.posTrackCommand is not None:
            y.posTrackCommand.mode = PosTrackCommandE.SPEED_TRACK

    def _write_context(self) -> None:
        """把完整计算结果原地提交到黑板。"""
        assert self._cxt is not None
        # 仅当本拍所有几何和时序计算成功后才提交结果。
        self._fill_common_output(self._y)
        # 写回采用原地复制，不能替换PosTrack和Outbound已绑定的对象。
        copy_motion(self._y.selfCmd, self._cxt.selfCmd)
        copy_pos_calc_status(self._y.status, self._cxt.posCalcStatus)
        self._cxt.posTrackCommand.mode = self._y.posTrackCommand.mode

    def reset(self) -> None:
        """复位 RallyJoinPos 的动态状态。注意：盘旋圆几何（圆心/切出点）由 init 时的任务航向定死，reset 不清除。"""
        self._state = RALLY_STATE_FLYING
        self._planned_path_length_m = -1.0
        self._remaining_path_length_m = -1.0
        self._plan_applied = False
        self._assigned_loops = 0
        self._remaining_loops = 0
        self._loiter_speed = self._approach_speed
        self._standby_speed = self._approach_speed
        self._standby_center_e = None
        self._standby_center_n = None
        self._standby_target_h = None
        self._transit_phase = None
        self._local_exit_point = None
        self._theta_local_exit = 0.0
        self._tangent_length_m = 0.0
        self._last_local_remaining_angle = None
        self._away_from_slot = False
        # 过点检测跨拍保存状态，复位时必须和圈数计划一起清空。
        self._last_slot_remaining_angle = None
        self._slot_near_window_armed = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False
        if self._cxt is not None:
            # NONE边沿由Manager触发reset，专有诊断同步回到初始FLYING语义。
            self._write_rally_status(self._cxt.posCalcStatus)

    def _write_rally_status(self, status: PosCalcStatusS) -> None:
        """写入集结专有诊断。注意：不修改当前活动策略。"""
        # active_strategy由当前实际运行的子类写入，reset不能抢占该字段。
        status.rally_state = self._state
        status.planned_path_length_m = self._planned_path_length_m
        status.remaining_path_length_m = self._remaining_path_length_m
        status.remaining_loops = self._remaining_loops
        status.reached_slot_once = self._reached_slot_once
        status.join_exited = self._state == RALLY_STATE_EXITED

    # ------------------------------------------------------------------ #
    # 内部阶段实现
    # ------------------------------------------------------------------ #

    def _enter_standby(self, u: RallyJoinPosInputS) -> None:
        """进入本地待命盘旋。注意：待命圆按进入待命这一拍的本机位置和航向反推。"""
        assert u.selfState is not None
        heading = u.selfState.v.vPsi
        self._standby_center_e = u.selfState.pos.east - self._loiter_radius * math.sin(heading)
        self._standby_center_n = u.selfState.pos.north + self._loiter_radius * math.cos(heading)
        self._standby_target_h = (
            self._standby_altitude_m
            if self._standby_altitude_m is not None
            else u.selfState.pos.h
        )
        if math.isfinite(u.selfState.v.vd) and u.selfState.v.vd > 1.0:
            standby_speed = u.selfState.v.vd
        else:
            standby_speed = self._approach_speed
        self._standby_speed = clamp(standby_speed, self._speed_min, self._speed_max)
        self._planned_path_length_m = -1.0
        self._remaining_path_length_m = -1.0
        self._transit_phase = None
        self._local_exit_point = None
        self._theta_local_exit = 0.0
        self._tangent_length_m = 0.0
        self._last_local_remaining_angle = None
        self._away_from_slot = False
        # 新待命圆不继承上一轮集结圆的近窗布防状态。
        self._last_slot_remaining_angle = None
        self._slot_near_window_armed = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False
        self._state = RALLY_STATE_STANDBY

    def _leave_standby(self, u: RallyJoinPosInputS) -> None:
        """离开本地待命并按本拍位置一次性规划两圆转移路径。"""
        assert self._standby_center_e is not None and self._standby_center_n is not None
        self._state = RALLY_STATE_FLYING
        self._planned_path_length_m = -1.0
        self._remaining_path_length_m = -1.0
        self._loiter_speed = self._approach_speed
        self._away_from_slot = False
        # 离开待命重新规划几何时，从未进入集结圆的状态开始记录有向角。
        self._last_slot_remaining_angle = None
        self._slot_near_window_armed = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False
        center_distance = math.hypot(
            self._loiter_center_e - self._standby_center_e,
            self._loiter_center_n - self._standby_center_n,
        )
        # 两个名义圆重合时，本机已经在集结圆上，继续直飞只会制造无意义的绕行。
        if center_distance <= _EPSILON_HORIZ:
            # 以当前径向角作为进圆角，保证后续到 M_i 的 CCW 弧长估算仍然连续。
            self._theta_entry = math.atan2(
                u.selfState.pos.north - self._loiter_center_n,
                u.selfState.pos.east - self._loiter_center_e,
            )
            # 重合圆没有转移线段，基础航程就是当前位置到松散点的集结圆弧。
            self._planned_path_length_m = self._rally_arc_to_slot_m(self._theta_entry)
            self._enter_arc()
            return
        self._transit_phase = self._plan_transit(u.selfState.pos)
        # 规划已锁存，后续剩余航程只随当前位置推进，不随实时几何重算。
        self._planned_path_length_m = self._remaining_base_path_m(u.selfState.pos)

    def _plan_transit(self, current_pos: PosInEarthS) -> str:
        """锁存待命圆到集结圆的 CCW 公切线；无解时退回当前点到集结圆切线。"""
        assert self._standby_center_e is not None and self._standby_center_n is not None
        local_center = PosInEarthS(
            east=self._standby_center_e,
            north=self._standby_center_n,
            h=current_pos.h,
        )
        rally_center = PosInEarthS(
            east=self._loiter_center_e,
            north=self._loiter_center_n,
            h=self._slot.h,
        )
        # 两圆均为 CCW，正转向参数让 common_tangent 选择前进方向一致的外公切线。
        tangent = common_tangent(
            local_center,
            self._loiter_radius,
            1.0,
            rally_center,
            self._loiter_radius,
            1.0,
        )
        if tangent is None:
            # 保留旧的点到圆切入能力；该方法还会在点位于圆内时继续退化到 M_i。
            self._entry_point = self._compute_entry_point(current_pos)
            return _TRANSIT_LINE_TO_RALLY_ENTRY

        # 两端切点和角度必须一起锁存，控制、切换判据与基础航程才能使用同一份几何。
        (local_e, local_n), (entry_e, entry_n) = tangent
        self._local_exit_point = PosInEarthS(east=local_e, north=local_n, h=self._slot.h)
        self._entry_point = PosInEarthS(east=entry_e, north=entry_n, h=self._slot.h)
        self._theta_local_exit = math.atan2(
            local_n - self._standby_center_n,
            local_e - self._standby_center_e,
        )
        self._theta_entry = math.atan2(
            entry_n - self._loiter_center_n,
            entry_e - self._loiter_center_e,
        )
        self._tangent_length_m = math.hypot(entry_e - local_e, entry_n - local_n)
        return _TRANSIT_ARC_TO_TANGENT

    def _step_standby(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """在本地待命阶段输出沿本机待命圆的 CCW 盘旋指令。"""
        assert u.selfState is not None
        assert self._standby_center_e is not None and self._standby_center_n is not None
        target_h = self._standby_target_h if self._standby_target_h is not None else u.selfState.pos.h
        self._write_ccw_circle_cmd(
            u,
            y,
            center_e=self._standby_center_e,
            center_n=self._standby_center_n,
            speed=self._standby_speed,
            target_h=target_h,
        )

    def _write_ccw_circle_cmd(
        self,
        u: RallyJoinPosInputS,
        y: RallyJoinPosOutputS,
        *,
        center_e: float,
        center_n: float,
        speed: float,
        target_h: float,
    ) -> None:
        """按本机当前径向角写入期望圆投影点与 CCW 切向运动指令。"""
        theta = math.atan2(
            u.selfState.pos.north - center_n,
            u.selfState.pos.east - center_e,
        )
        y.selfCmd.pos.east = center_e + self._loiter_radius * math.cos(theta)
        y.selfCmd.pos.north = center_n + self._loiter_radius * math.sin(theta)
        y.selfCmd.pos.h = target_h
        v_e = -speed * math.sin(theta)
        v_n = speed * math.cos(theta)
        y.selfCmd.v.vEast = v_e
        y.selfCmd.v.vNorth = v_n
        y.selfCmd.v.vPsi = math.atan2(v_n, v_e)
        y.selfCmd.v.vd = speed
        d_h = target_h - u.selfState.pos.h
        y.selfCmd.v.vUp = clamp(d_h * 0.3, self._v_up_min, self._v_up_max)
        y.selfCmd.v.dVPsi = speed / self._loiter_radius
        y.selfCmd.v.vTheta = 0.0

    def _step_flying(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """在直飞阶段生成指向盘旋圆切入点 T 的指令，到达 T 附近后转入圆弧飞行。"""
        if self._transit_phase == _TRANSIT_ARC_TO_TANGENT:
            self._step_arc_to_tangent(u, y)
            return
        if self._entry_point is None:
            self._entry_point = self._compute_entry_point(u.selfState.pos)
        target = self._entry_point

        d_e = target.east - u.selfState.pos.east
        d_n = target.north - u.selfState.pos.north
        d_h = target.h - u.selfState.pos.h
        d_horiz = math.sqrt(d_e * d_e + d_n * d_n)
        d_3d = math.sqrt(d_horiz * d_horiz + d_h * d_h)

        if self._plan_applied:
            speed = self._coordinated_speed(u)
        else:
            speed = self._flying_speed_for_distance(d_horiz)

        y.selfCmd.pos.east = target.east
        y.selfCmd.pos.north = target.north
        y.selfCmd.pos.h = target.h
        if d_horiz >= _EPSILON_HORIZ:
            y.selfCmd.v.vPsi = math.atan2(d_n, d_e)
            y.selfCmd.v.vEast = speed * d_e / d_horiz
            y.selfCmd.v.vNorth = speed * d_n / d_horiz
            y.selfCmd.v.vd = speed
            v_up_raw = speed * d_h / max(d_horiz, 1.0)
            y.selfCmd.v.vUp = clamp(v_up_raw, self._v_up_min, self._v_up_max)
        else:
            y.selfCmd.v.vPsi = u.selfState.v.vPsi
            y.selfCmd.v.vEast = 0.0
            y.selfCmd.v.vNorth = 0.0
            y.selfCmd.v.vd = 0.0
            y.selfCmd.v.vUp = clamp(d_h * 0.5, self._v_up_min, self._v_up_max)
        y.selfCmd.v.dVPsi = 0.0
        y.selfCmd.v.vTheta = 0.0

        # 触发半径夹到 self._arc_capture_radius_m（按 loiter_radius_m 反解，见 init）：T 是圆上固定点，
        # 容差越大，切到 LOITERING 那一拍的切向航向（按飞机此刻实际角度算）跟 FLYING 直飞航向（按 T 处
        # 角度算）的偏差就越大——用 100m 默认 arrival_radius_m 实测过约 26° 的指令航向跳变；且 T 是
        # 静止点，不像旧版直飞 M_i 那样需要较大容差防止绕不拢，收紧没有副作用。
        if d_3d < min(self._arrival_radius_m, self._arc_capture_radius_m):
            self._enter_arc()
            # 同一拍内顺势推进圆弧阶段：既让"起点恰好落在 M_i 上"的退化场景无延迟切出，
            # 也避免刚到 T 这一拍还输出已经过期的直飞指令。
            self._step_loitering(u, y)

    def _step_arc_to_tangent(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """沿待命圆接近锁存切出点，进入角度窗口后切换到公切线直飞。"""
        assert self._standby_center_e is not None and self._standby_center_n is not None
        theta = math.atan2(
            u.selfState.pos.north - self._standby_center_n,
            u.selfState.pos.east - self._standby_center_e,
        )
        remaining = (self._theta_local_exit - theta) % _TWO_PI
        # 正常 CCW 推进时 remaining 单调减小；跨过切点后会从接近 0 突然回绕到接近 2π。
        crossed_exit = (
            self._last_local_remaining_angle is not None
            and remaining > self._last_local_remaining_angle + math.pi
        )
        # 进入提前切出窗口或确认已跨点时，都在本拍直接改发锁存公切线指令。
        if remaining <= _LOCAL_TANGENT_CAPTURE_ANGLE_RAD or crossed_exit:
            self._transit_phase = _TRANSIT_LINE_TO_RALLY_ENTRY
            self._step_flying(u, y)
            return
        self._last_local_remaining_angle = remaining
        target_h = self._standby_target_h if self._standby_target_h is not None else u.selfState.pos.h
        self._write_ccw_circle_cmd(
            u,
            y,
            center_e=self._standby_center_e,
            center_n=self._standby_center_n,
            speed=self._coordinated_speed(u) if self._plan_applied else self._standby_speed,
            target_h=target_h,
        )

    def _flying_speed_for_distance(self, distance_m: float) -> float:
        """按直线段剩余水平距离计算近场降速后的有效速度。"""
        speed = self._approach_speed
        if self._slow_radius_m > 0.0:
            speed *= min(1.0, max(0.0, distance_m / self._slow_radius_m))
        return max(self._speed_min, speed)

    def _compute_entry_point(self, pos: PosInEarthS) -> PosInEarthS:
        """按当前位置求切入盘旋圆、能顺势接上 CCW 弧线的切点 T。

        注意：起点已落在圆内/圆上时无切线可求（已知限制，见模块文档），退化为直飞 M_i。
        """
        tangent = _ccw_entry_tangent(pos.east, pos.north, self._loiter_center_e, self._loiter_center_n, self._loiter_radius)
        if tangent is None:
            self._theta_entry = self._theta_slot
            return self._slot
        t_e, t_n, theta_t = tangent
        self._theta_entry = theta_t
        return PosInEarthS(east=t_e, north=t_n, h=self._slot.h)

    def _enter_arc(self) -> None:
        """从切入点顺势转入圆弧飞行（CCW），未执行计划时沿用最低盘旋速度。"""
        self._loiter_speed = self._speed_min
        entry_arc_angle = (self._theta_slot - self._theta_entry) % _TWO_PI
        # 点前且位于 away 阈值内时，锁存切入几何已提供方向明确的跨零前态；近窗外只预置 away。
        # 只有点前近窗或恰在 M_i 才预置 armed；点后近窗接近 2π，两项都保持 False。
        entry_can_start_away = entry_arc_angle <= _SLOT_ANG_AWAY
        entry_can_arm_crossing = entry_arc_angle < _SLOT_ANG_NEAR
        self._away_from_slot = entry_can_start_away
        self._last_slot_remaining_angle = entry_arc_angle if entry_can_start_away else None
        self._slot_near_window_armed = entry_can_arm_crossing
        self._reached_slot_once = False
        self._state = RALLY_STATE_LOITERING

    def _rally_arc_to_slot_m(self, theta: float) -> float:
        """返回集结圆上从指定角度逆时针飞到松散点的弧长。"""
        return self._loiter_radius * ((self._theta_slot - theta) % _TWO_PI)

    def _remaining_base_path_m(self, pos: PosInEarthS) -> float:
        """按当前阶段返回锁存基础路线到下一次松散点的剩余水平航程。"""
        if self._transit_phase == _TRANSIT_ARC_TO_TANGENT:
            theta = math.atan2(pos.north - self._standby_center_n, pos.east - self._standby_center_e)
            local_arc = self._loiter_radius * ((self._theta_local_exit - theta) % _TWO_PI)
            return local_arc + self._tangent_length_m + self._rally_arc_to_slot_m(self._theta_entry)
        if self._state == RALLY_STATE_FLYING and self._entry_point is not None:
            line = math.hypot(self._entry_point.east - pos.east, self._entry_point.north - pos.north)
            return line + self._rally_arc_to_slot_m(self._theta_entry)
        if self._state == RALLY_STATE_LOITERING:
            theta = math.atan2(pos.north - self._loiter_center_n, pos.east - self._loiter_center_e)
            return self._rally_arc_to_slot_m(theta)
        return 0.0

    def _coordinated_speed(self, u: RallyJoinPosInputS) -> float:
        """按完整剩余航程和固定计划剩余时间计算水平协调速度。"""
        if not self._plan_applied:
            return self._approach_speed
        # 该函数会在计划生效后的每个 JOINING 子阶段调用，不能只依赖首次计划校验。
        if (
            u.rallyPlan is None
            or u.clock is None
            or not math.isfinite(u.rallyPlan.t_ref)
            or not math.isfinite(u.clock.now_s)
        ):
            raise ValueError("协调调速的 t_ref 和 now_s 必须为有限数")
        # 基础航程沿锁存几何实时递减，整圈部分只由真实经过 M_i 的事件消费；
        # 两者相加后统一除以固定计划的剩余时间，三个子阶段不再各用一套速度口径。
        circumference = _TWO_PI * self._loiter_radius
        remaining_m = self._remaining_base_path_m(u.selfState.pos) + self._remaining_loops * circumference
        remaining_s = u.rallyPlan.t_ref - u.clock.now_s
        # 计划时刻已经到达或越过时直接采用上限，避免除零并尽快追赶固定计划。
        # 此分支只处理有限时间的非正差值，NaN/Inf 已在函数入口拒绝。
        if remaining_s <= 0.0:
            return self._speed_max
        return clamp(remaining_m / remaining_s, self._speed_min, self._speed_max)

    def _step_loitering(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """在盘旋阶段更新切出判定并生成圆周飞行指令。"""
        pos_e = u.selfState.pos.east
        pos_n = u.selfState.pos.north

        # 当前在圆上的角度
        d_e = pos_e - self._loiter_center_e
        d_n = pos_n - self._loiter_center_n
        theta = math.atan2(d_n, d_e)

        # 有向剩余角在点前趋近 0，越过 M_i 后回绕到接近 2π；实际轨道半径不参与判定。
        slot_remaining_angle = (self._theta_slot - theta) % _TWO_PI
        ang_dist = min(slot_remaining_angle, _TWO_PI - slot_remaining_angle)
        # 远区布防阻止刚切入圆弧或带噪声落在 M_i 附近时被当作一次完整经过。
        if not self._away_from_slot and ang_dist > _SLOT_ANG_AWAY:
            self._away_from_slot = True
        # 只在点前有向剩余角进入近窗时 armed；点后虽有相同对称弧距，但不会反向 armed。
        if self._away_from_slot and slot_remaining_angle < _SLOT_ANG_NEAR:
            self._slot_near_window_armed = True
        # 相邻拍按 CCW 前进时，有向剩余角递减；跨零后则从点前小角回绕到点后大角。
        # 前一拍位于 away 阈值内即可确认跨点来源，不再强制先采到更窄的 near 窗。
        previous_remaining = self._last_slot_remaining_angle
        forward_progress = (
            (previous_remaining - slot_remaining_angle) % _TWO_PI
            if previous_remaining is not None
            else 0.0
        )
        crossing_candidate = (
            self._away_from_slot
            and previous_remaining is not None
            and previous_remaining <= _SLOT_ANG_AWAY
            and previous_remaining < math.pi
            and slot_remaining_angle > math.pi
            and 0.0 < forward_progress <= math.pi
        )
        # 小于水平近零阈值的跨零位移按位置抖动处理，并保留点前样本等待后续真实推进。
        crossed_slot = crossing_candidate and forward_progress * self._loiter_radius > _EPSILON_HORIZ
        hold_previous_for_noise = crossing_candidate and not crossed_slot
        if crossed_slot:
            # reached、圈数消费和 EXITED 共用同一个真实越点事件，避免三套判据漂移。
            self._reached_slot_once = True
            # 越点后同时撤销远区和近窗布防，下一圈必须重新完整经过两个门槛。
            self._away_from_slot = False
            self._slot_near_window_armed = False
            if self._plan_applied and self._remaining_loops > 0:
                self._remaining_loops -= 1
            elif self._plan_applied:
                self._state = RALLY_STATE_EXITED
                self._set_exit_cmd(u, y)
                return
        # 消费后保存点后大角度，下一拍不会重复满足回绕；噪声候选则保留点前样本。
        if not hold_previous_for_noise:
            self._last_slot_remaining_angle = slot_remaining_angle

        # 过点事件先更新圈数，再按回绕后的基础弧长计算速度，保证完整剩余航程跨点连续。
        if self._plan_applied:
            self._loiter_speed = self._coordinated_speed(u)

        # 复用待命阶段的圆上投影、切向速度与曲率前馈口径，避免两套 CCW 圆周公式漂移。
        self._write_ccw_circle_cmd(
            u,
            y,
            center_e=self._loiter_center_e,
            center_n=self._loiter_center_n,
            speed=self._loiter_speed,
            target_h=self._slot.h,
        )

    def _step_exited(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """在已切出阶段持续输出沿任务航向飞行的过渡指令。"""
        self._set_exit_cmd(u, y)

    def _set_exit_cmd(self, u: RallyJoinPosInputS, y: RallyJoinPosOutputS) -> None:
        """写入从松散点沿任务航向直飞的目标位置与速度。"""
        vd = self._mission_speed
        heading = self._mission_heading
        # 目标点设在远前方保持持续前飞（编队控制接管前的过渡）
        y.selfCmd.pos.east = self._slot.east + 5000.0 * math.cos(heading)
        y.selfCmd.pos.north = self._slot.north + 5000.0 * math.sin(heading)
        y.selfCmd.pos.h = self._slot.h
        y.selfCmd.v.vEast = vd * math.cos(heading)
        y.selfCmd.v.vNorth = vd * math.sin(heading)
        y.selfCmd.v.vPsi = heading
        y.selfCmd.v.vd = vd
        d_h = self._slot.h - u.selfState.pos.h
        y.selfCmd.v.vUp = clamp(d_h * 0.3, self._v_up_min, self._v_up_max)
        y.selfCmd.v.dVPsi = 0.0
        y.selfCmd.v.vTheta = 0.0

def _ccw_entry_tangent(px: float, py: float, cx: float, cy: float, r: float) -> tuple[float, float, float] | None:
    """求外部点 (px,py) 到圆 (cx,cy,r) 上、能顺势接上 CCW 弧线的切点。

    注意：外部点到圆一般有两条切线/两个切点，只有一条切线的直飞方向能在切点处顺势接上 CCW 切向
    （另一条接的是 CW，方向不对，不能用）——用直飞方向和切点处 CCW 切向方向是否同向来筛选。
    起点已在圆内/圆上（距圆心 <= r，无切线）时返回 None，交由调用方兜底。
    """
    dx, dy = px - cx, py - cy
    d = math.hypot(dx, dy)
    if d <= r:
        return None
    angle_cp = math.atan2(dy, dx)
    phi = math.acos(clamp(r / d, -1.0, 1.0))
    for sign in (1.0, -1.0):
        theta_t = angle_cp + sign * phi
        t_e = cx + r * math.cos(theta_t)
        t_n = cy + r * math.sin(theta_t)
        approach_e, approach_n = t_e - px, t_n - py
        approach_norm = math.hypot(approach_e, approach_n)
        if approach_norm < 1e-9:
            continue
        # 切点处 CCW 切向方向 = theta_t + 90°
        tangent_dir_e, tangent_dir_n = -math.sin(theta_t), math.cos(theta_t)
        cos_align = (approach_e * tangent_dir_e + approach_n * tangent_dir_n) / approach_norm
        if cos_align > 0.0:
            return t_e, t_n, theta_t
    return None
