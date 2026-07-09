"""集结汇合位置解算：待命盘旋 → 平等飞行 → 切入盘旋圆 → 圆弧盘旋 → 切出。

每架飞机独立运行此模块，无长机/僚机之分。
四个阶段：
  STANDBY   — 在本机当前位置按当前航向反推本地待命圆，沿圆盘旋等待外部开始集结指令
  FLYING    — 直飞盘旋圆的 CCW 切入点 T（由当前位置对盘旋圆作切线求得，FLYING 第一拍算一次后固定），
               到达 T 附近后顺势转入圆弧飞行，广播 ETA
  LOITERING — 沿盘旋圆做 CCW 圆弧飞行；每次路过松散点 M_i（圆上固定的切出点）时评估是否切出
  EXITED    — 从松散点沿任务航向直飞，交由编队控制接管

盘旋圆的圆心和切出点 M_i 在 init 时就按任务航向定死：M_i 是松散槽位，圆心摆在任务航向左侧 R 处，
使 M_i 处的 CCW 切线方向恒等于任务航向——不管飞机从哪个方向飞来，只要沿圆弧飞到 M_i 就必然对齐任务航向，
切出瞬间的指令不会因为到达方向不同而发生跳变（这也是为什么切入点 T 要专门算一条切线，
而不是像旧版那样直飞 M_i 再原地盘旋：直飞 M_i 时的到达航向是任意的，会让盘旋圆摆歪）。
圆半径固定（loiter_radius_m），通过调整盘旋速度改变盘旋周期以匹配 T_ref；
如果本机恰好是"最后到达"的一架，绕圆弧第一次路过 M_i 时就会满足切出条件，不需要先转一整圈。

LOITERING 阶段的位置指令是"期望半径圆上、飞机当前角度处的投影点"（不是圆心），向心加速度前馈也用
期望半径而非实时半径——这样飞机的实际盘旋半径才会收敛到 loiter_radius_m 本身；若指令目标点是圆心，
位置误差（侧偏）恒等于飞机此刻的实际半径，跟期望半径无关，控制律没有把半径拉回期望值的趋势，
实测会在很宽的范围内漂移（同一场景下从 23m 到 222m 都出现过，期望是 200m）。

已知限制：起始位置落在盘旋圆内部（无法作切线）时暂未处理，退化为直飞 M_i（等价于旧版行为），
留待后续按需补齐。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import PosInEarthS
from src.algorithm.units.algo.formation_math import clamp
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS

_TWO_PI = 2.0 * math.pi

RALLY_STATE_FLYING = "FLYING"
RALLY_STATE_LOITERING = "LOITERING"
RALLY_STATE_EXITED = "EXITED"
RALLY_STATE_STANDBY = "STANDBY"

_EPSILON_HORIZ = 0.5  # 水平近零距离阈值，米
_SLOT_ANG_NEAR = 0.35  # ≈20°：判定"经过 M_i"的角度窗口（不依赖轨道半径）
_SLOT_ANG_AWAY = 1.05  # ≈60°：判定"已远离 M_i"的角度阈值
# 切入圆弧（FLYING→LOITERING）触发半径 d 与航向跳变角 ψ 的几何关系是 ψ = atan(d/R)（R=loiter_radius_m，
# 见 _compute_arc_capture_radius_m 推导），所以固定距离上限只对某一个 R 好使——R 越小，同样的 d 换算出的
# ψ 越大。改成按 R 反解 d = R·tan(ψ_max)，跳变角上限跟 R 无关，恒定在 ψ_max 附近。
_MAX_ARC_CAPTURE_HEADING_JUMP_RAD = math.radians(5.0)  # 允许的 FLYING→LOITERING 指令航向跳变上限
_MIN_ARC_CAPTURE_RADIUS_M = 0.5  # 触发半径下限，避免 loiter_radius_m 很小时算出不现实的亚米级容差
# 触发半径至少要能跨过这么多个控制周期的飞行距离，否则离散步进可能整拍跨过捕获窗口、错过 T 而
# 永远飞不进 LOITERING（FLYING 阶段直飞 T 是匀速直线，d_3d 每拍减少约 flying_speed*control_period_s）。
_MIN_ARC_CAPTURE_STEP_MARGIN = 3.0


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
class RallyJoinPosInputS(PosCalcInputS):
    """RallyJoinPos 输入端口。"""

    # 继承 selfState: MotionProfS
    t_ref: float = 0.0  # 集结基准时刻（长机广播的最晚 ETA）
    t_ref_valid: bool = False  # False 时只允许进入/保持盘旋，不允许切出
    t_now: float = 0.0  # 当前仿真时间
    standby: bool = False  # True 表示本拍保持本地待命盘旋，不进入集结圆汇合


class RallyJoinPos(PosCalcBase):
    """集结汇合位置解算器。注意：state 和 eta_s 属性供外部读取后广播。"""

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
        self._eta_s: float = 0.0
        self._loiter_speed: float = cfg.approach_speed_mps
        self._standby_speed: float = cfg.approach_speed_mps
        self._standby_center_e: float | None = None
        self._standby_center_n: float | None = None
        self._standby_target_h: float | None = None
        self._away_from_slot: bool = False  # 盘旋后是否已远离松散点（防止立即切出）
        self._entry_point: PosInEarthS | None = None  # 切入点 T；FLYING 第一拍按当前位置算一次后固定
        self._theta_entry: float = 0.0  # 切入点在盘旋圆上的角度，供 ETA 弧长估算
        self._reached_slot_once: bool = False  # 是否已至少一次路过 M_i；供 T_ref 聚合判断本机是否仍需被等待

    @property
    def state(self) -> str:
        """返回当前汇合子状态。"""
        return self._state

    @property
    def eta_s(self) -> float:
        """返回当前预计到达松散点的仿真时刻。"""
        return self._eta_s

    @property
    def reached_slot_once(self) -> bool:
        """返回是否已至少一次路过松散点 M_i。

        注意：切入点 T 到 M_i 之间可能还有很长一段弧要飞（见 FLYING/LOITERING 说明），从 FLYING 切到
        LOITERING 不代表已经"到过" M_i——T_ref 聚合（Rally 任务）需要这个更精确的信号来判断本机是否
        仍应计入基准时间：还没路过 M_i 一次时应计入（避免被过早剔除导致 T_ref 提前塌缩），已经路过至少
        一次后应排除（避免盘旋等待时每圈波动的 eta_s 反复推高 T_ref）。
        """
        return self._reached_slot_once

    def step(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """推进 RallyJoinPos 一个处理周期。"""
        if u.selfState is None or y.selfCmd is None:
            raise ValueError("RallyJoinPos ports must be bound")
        if u.standby:
            if self._state != RALLY_STATE_STANDBY:
                self._enter_standby(u)
            self._step_standby(u, y)
            return
        if self._state == RALLY_STATE_STANDBY:
            self._leave_standby()
        if self._state == RALLY_STATE_FLYING:
            self._step_flying(u, y)
        elif self._state == RALLY_STATE_LOITERING:
            self._step_loitering(u, y)
        else:
            self._step_exited(u, y)

    def reset(self) -> None:
        """复位 RallyJoinPos 的动态状态。注意：盘旋圆几何（圆心/切出点）由 init 时的任务航向定死，reset 不清除。"""
        self._state = RALLY_STATE_FLYING
        self._eta_s = 0.0
        self._loiter_speed = self._approach_speed
        self._standby_speed = self._approach_speed
        self._standby_center_e = None
        self._standby_center_n = None
        self._standby_target_h = None
        self._away_from_slot = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False

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
        self._eta_s = 0.0
        self._away_from_slot = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False
        self._state = RALLY_STATE_STANDBY

    def _leave_standby(self) -> None:
        """离开本地待命进入飞向集结圆阶段。注意：切入点必须按离开待命时的当前位置重新计算。"""
        self._state = RALLY_STATE_FLYING
        self._eta_s = 0.0
        self._loiter_speed = self._approach_speed
        self._away_from_slot = False
        self._entry_point = None
        self._theta_entry = 0.0
        self._reached_slot_once = False

    def _step_standby(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """在本地待命阶段输出沿本机待命圆的 CCW 盘旋指令。"""
        assert u.selfState is not None
        assert self._standby_center_e is not None and self._standby_center_n is not None
        target_h = self._standby_target_h if self._standby_target_h is not None else u.selfState.pos.h
        theta = math.atan2(
            u.selfState.pos.north - self._standby_center_n,
            u.selfState.pos.east - self._standby_center_e,
        )
        y.selfCmd.pos.east = self._standby_center_e + self._loiter_radius * math.cos(theta)
        y.selfCmd.pos.north = self._standby_center_n + self._loiter_radius * math.sin(theta)
        y.selfCmd.pos.h = target_h
        v_e = -self._standby_speed * math.sin(theta)
        v_n = self._standby_speed * math.cos(theta)
        y.selfCmd.v.vEast = v_e
        y.selfCmd.v.vNorth = v_n
        y.selfCmd.v.vPsi = math.atan2(v_n, v_e)
        y.selfCmd.v.vd = self._standby_speed
        d_h = target_h - u.selfState.pos.h
        y.selfCmd.v.vUp = clamp(d_h * 0.3, self._v_up_min, self._v_up_max)
        y.selfCmd.v.dVPsi = self._standby_speed / self._loiter_radius
        y.selfCmd.v.vTheta = 0.0
        self._eta_s = 0.0

    def _step_flying(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """在直飞阶段生成指向盘旋圆切入点 T 的指令，到达 T 附近后转入圆弧飞行。"""
        if self._entry_point is None:
            self._entry_point = self._compute_entry_point(u.selfState.pos)
        target = self._entry_point

        d_e = target.east - u.selfState.pos.east
        d_n = target.north - u.selfState.pos.north
        d_h = target.h - u.selfState.pos.h
        d_horiz = math.sqrt(d_e * d_e + d_n * d_n)
        d_3d = math.sqrt(d_horiz * d_horiz + d_h * d_h)

        speed = self._approach_speed
        if self._slow_radius_m > 0.0:
            speed *= min(1.0, max(0.0, d_horiz / self._slow_radius_m))
        speed = max(self._speed_min, speed)

        # ETA = 直飞 T 的时间 + 沿圆弧从 T 飞到 M_i 的时间（弧长按名义 approach_speed 估算，
        # 盘旋阶段命中 T_ref 后会用实际调速的 loiter_speed 重新估算，这里只是首次进场的粗估）。
        arc_angle = (self._theta_slot - self._theta_entry) % _TWO_PI
        arc_len = self._loiter_radius * arc_angle
        self._eta_s = u.t_now + d_3d / max(speed, 0.1) + arc_len / max(self._approach_speed, 0.1)

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
            remaining = (u.t_ref - u.t_now) if u.t_ref_valid else float("inf")
            self._enter_arc(remaining)
            # 同一拍内顺势推进圆弧阶段：既让"起点恰好落在 M_i 上"的退化场景无延迟切出，
            # 也避免刚到 T 这一拍还输出已经过期的直飞指令。
            self._step_loitering(u, y)

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

    def _enter_arc(self, remaining: float) -> None:
        """从切入点顺势转入圆弧飞行（CCW）。注意：圆心/切出点已在 init 时定死，这里只切换状态和盘旋速度。"""
        self._loiter_speed = self._adjusted_speed(remaining)
        # ang_dist 是对称弧距，分不清"再飞一小段就到 M_i"和"刚绕过 M_i、实际还要飞近一整圈"——
        # 只有切入点弧长（CCW，从 T 到 M_i 的真实剩余角度）本身就很小时，才说明确实快到了，
        # 允许首次路过就评估切出（"最后到达"的飞机场景）；否则必须按标准流程先飞过"远离"窗口，
        # 避免把"刚越过 M_i"误判成"已到达"，在没有真正沿圆弧飞完一圈的情况下就切出。
        entry_arc_angle = (self._theta_slot - self._theta_entry) % _TWO_PI
        self._away_from_slot = entry_arc_angle < _SLOT_ANG_NEAR
        self._reached_slot_once = self._away_from_slot  # 同一个"真实弧长很小"判据，切入点本来就快到 M_i
        self._state = RALLY_STATE_LOITERING

    def _step_loitering(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """在盘旋阶段更新到点时间、切出判定和圆周飞行指令。"""
        pos_e = u.selfState.pos.east
        pos_n = u.selfState.pos.north

        # 当前在圆上的角度
        d_e = pos_e - self._loiter_center_e
        d_n = pos_n - self._loiter_center_n
        theta = math.atan2(d_n, d_e)

        # CCW 弧长至松散点（用实际半径让 ETA 更准确）
        arc_angle = (self._theta_slot - theta) % _TWO_PI
        if arc_angle < 1e-6:
            arc_angle = _TWO_PI
        actual_r = math.sqrt(d_e * d_e + d_n * d_n)
        arc_len = max(actual_r, 1.0) * arc_angle
        t_to_slot = arc_len / max(self._loiter_speed, 0.1)
        self._eta_s = u.t_now + t_to_slot

        # 角度判断"远离 / 经过 M_i"：不依赖实际轨道半径，轨道偏差也能可靠检测
        ang_dist = min(arc_angle, _TWO_PI - arc_angle)  # 到 M_i 的最短弧（0 = 正在经过）
        if not self._away_from_slot and ang_dist > _SLOT_ANG_AWAY:
            self._away_from_slot = True
        # 不依赖 t_ref_valid：只要几何上真正路过 M_i 附近就算数，供 T_ref 聚合判断本机是否仍需被等待。
        # 必须同时要求 self._away_from_slot：ang_dist 是对称弧距，分不清"快到 M_i"（arc_angle 小）和
        # "刚越过 M_i、其实还要绕近一整圈"（arc_angle 接近 2π，ang_dist 同样很小）——刚进弧那一拍如果
        # entry_arc_angle 很大（_enter_arc 已正确把 away_from_slot/reached_slot_once 都设为 False），
        # 这里若只看 ang_dist 会在同一拍内立刻把 reached_slot_once 错误地翻回 True，抵消 _enter_arc 的判断。
        # away_from_slot 已经是"先远离过、再靠近"的正确判据（切出评估复用的就是它），一并复用即可。
        if self._away_from_slot and ang_dist < _SLOT_ANG_NEAR:
            self._reached_slot_once = True

        # 每次经过松散点时做切出评估
        if self._away_from_slot and ang_dist < _SLOT_ANG_NEAR and u.t_ref_valid:
            remaining = u.t_ref - u.t_now
            if self._should_exit(remaining):
                self._state = RALLY_STATE_EXITED
                self._eta_s = u.t_now
                self._set_exit_cmd(u, y)
                return
            # 调整下一圈盘旋速度
            self._loiter_speed = self._adjusted_speed(remaining)
            self._away_from_slot = False  # 重置，等下次远离后再判断

        # 飞盘旋圆：selfCmd.pos = 期望半径圆上、飞机当前角度处的投影点（不是圆心），
        # 这样侧偏才是"离期望圆多远"的半径误差，能收敛到 loiter_radius_m；
        # 若仍用圆心当目标，侧偏恒等于飞机此刻的实际半径本身，跟期望半径无关，收敛不到目标值。
        y.selfCmd.pos.east = self._loiter_center_e + self._loiter_radius * math.cos(theta)
        y.selfCmd.pos.north = self._loiter_center_n + self._loiter_radius * math.sin(theta)
        y.selfCmd.pos.h = self._slot.h
        v_e = -self._loiter_speed * math.sin(theta)
        v_n = self._loiter_speed * math.cos(theta)
        y.selfCmd.v.vEast = v_e
        y.selfCmd.v.vNorth = v_n
        y.selfCmd.v.vPsi = math.atan2(v_n, v_e)
        y.selfCmd.v.vd = self._loiter_speed
        d_h = self._slot.h - u.selfState.pos.h
        y.selfCmd.v.vUp = clamp(d_h * 0.3, self._v_up_min, self._v_up_max)
        # CCW 向心前馈：用期望半径而非实时半径，让前馈本身就以 loiter_radius_m 为平衡点
        # （用实时半径的前馈只会维持"当前飞多大就多大"，没有把半径拉回期望值的趋势）。
        y.selfCmd.v.dVPsi = self._loiter_speed / self._loiter_radius
        y.selfCmd.v.vTheta = 0.0

    def _step_exited(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """在已切出阶段持续输出沿任务航向飞行的过渡指令。"""
        self._eta_s = u.t_now
        self._set_exit_cmd(u, y)

    def _set_exit_cmd(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
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

    def _should_exit(self, remaining: float) -> bool:
        """经过松散点时：现在切出是否比再飞一圈更接近 T_ref。"""
        t_loop_min = _TWO_PI * self._loiter_radius / self._speed_max
        # 剩余时间小于最快半圈时，现在切出误差更小
        return remaining < t_loop_min / 2.0

    def _adjusted_speed(self, remaining: float) -> float:
        """计算下一圈目标盘旋速度，使盘旋周期尽量等于 remaining。"""
        t_loop_min = _TWO_PI * self._loiter_radius / self._speed_max
        t_loop_max = _TWO_PI * self._loiter_radius / self._speed_min
        if remaining <= 0.0 or remaining < t_loop_min:
            return self._speed_max
        if remaining > t_loop_max:
            return self._speed_min
        return _TWO_PI * self._loiter_radius / remaining


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
