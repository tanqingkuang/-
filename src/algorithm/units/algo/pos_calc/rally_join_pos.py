"""集结汇合位置解算：平等飞行 → 过松散点盘旋 → 切出。

每架飞机独立运行此模块，无长机/僚机之分。
三个阶段：
  FLYING   — 直飞固定松散槽位，广播 ETA
  LOITERING — 以松散点为圆上切出点做 CCW 盘旋；每次过松散点时评估是否切出
  EXITED   — 从松散点沿任务航向直飞，交由编队控制接管

盘旋圆半径固定（loiter_radius_m），通过调整盘旋速度改变盘旋周期以匹配 T_ref。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import PosInEarthS
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS

_TWO_PI = 2.0 * math.pi

RALLY_STATE_FLYING = "FLYING"
RALLY_STATE_LOITERING = "LOITERING"
RALLY_STATE_EXITED = "EXITED"

_EPSILON_HORIZ = 0.5  # 水平近零距离阈值，米
_SLOT_ANG_NEAR = 0.35  # ≈20°：判定"经过 M_i"的角度窗口（不依赖轨道半径）
_SLOT_ANG_AWAY = 1.05  # ≈60°：判定"已远离 M_i"的角度阈值


@dataclass
class RallyJoinPosInitS(PosCalcInitS):
    """RallyJoinPos 初始化参数。"""

    loose_slot: PosInEarthS = field(default_factory=PosInEarthS)  # 本机固定松散目标点（ENU 全局坐标）
    approach_speed_mps: float = 20.0  # 飞向松散点时的速度
    slow_radius_m: float = 0.0  # 近场降速半径；>0 时在此范围内线性减速
    arrival_radius_m: float = 100.0  # 进入盘旋的触发距离
    loiter_radius_m: float = 200.0  # 盘旋圆半径（固定）
    loiter_speed_min_mps: float = 14.0  # 最小盘旋速度（固定翼速度下限）
    loiter_speed_max_mps: float = 25.0  # 最大盘旋速度
    mission_heading_rad: float = 0.0  # 切出后的飞行方向（弧度，从东向起算）
    mission_speed_mps: float = 20.0  # 切出后的飞行速度
    last_arrival_threshold_s: float = 5.0  # 兼容保留；当前切出判定固定使用最快盘旋半圈


@dataclass
class RallyJoinPosInputS(PosCalcInputS):
    """RallyJoinPos 输入端口。"""

    # 继承 selfState: MotionProfS
    t_ref: float = 0.0  # 集结基准时刻（长机广播的最晚 ETA）
    t_ref_valid: bool = False  # False 时只允许进入/保持盘旋，不允许切出
    t_now: float = 0.0  # 当前仿真时间


class RallyJoinPos(PosCalcBase):
    """集结汇合位置解算器。注意：state 和 eta_s 属性供外部读取后广播。"""

    def init(self, cfg: RallyJoinPosInitS) -> None:
        """按配置初始化 RallyJoinPos。"""
        if cfg.approach_speed_mps <= 0:
            raise ValueError("approach_speed_mps must be > 0")
        if cfg.loiter_radius_m <= 0:
            raise ValueError("loiter_radius_m must be > 0")
        if cfg.loiter_speed_min_mps <= 0 or cfg.loiter_speed_max_mps <= cfg.loiter_speed_min_mps:
            raise ValueError("loiter speed limits invalid")

        self._slot = cfg.loose_slot
        self._approach_speed = cfg.approach_speed_mps
        self._slow_radius_m = cfg.slow_radius_m
        self._arrival_radius_m = cfg.arrival_radius_m
        self._loiter_radius = cfg.loiter_radius_m
        self._speed_min = cfg.loiter_speed_min_mps
        self._speed_max = cfg.loiter_speed_max_mps
        self._mission_heading = cfg.mission_heading_rad
        self._mission_speed = cfg.mission_speed_mps
        self._last_arrival_threshold_s = cfg.last_arrival_threshold_s  # 保留配置值，当前算法不读取

        self._state: str = RALLY_STATE_FLYING
        self._eta_s: float = 0.0
        self._loiter_center_e: float = 0.0
        self._loiter_center_n: float = 0.0
        self._loiter_speed: float = cfg.approach_speed_mps
        self._theta_slot: float = 0.0  # 松散点在盘旋圆上的角度（弧度）
        self._away_from_slot: bool = False  # 盘旋后是否已远离松散点（防止立即切出）

    @property
    def state(self) -> str:
        """返回当前汇合子状态。"""
        return self._state

    @property
    def eta_s(self) -> float:
        """返回当前预计到达松散点的仿真时刻。"""
        return self._eta_s

    def step(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """推进 RallyJoinPos 一个处理周期。"""
        if u.selfState is None or y.selfCmd is None:
            raise ValueError("RallyJoinPos ports must be bound")
        if self._state == RALLY_STATE_FLYING:
            self._step_flying(u, y)
        elif self._state == RALLY_STATE_LOITERING:
            self._step_loitering(u, y)
        else:
            self._step_exited(u, y)

    def reset(self) -> None:
        """复位 RallyJoinPos 的动态状态。"""
        self._state = RALLY_STATE_FLYING
        self._eta_s = 0.0
        self._loiter_speed = self._approach_speed
        self._loiter_center_e = 0.0
        self._loiter_center_n = 0.0
        self._away_from_slot = False

    # ------------------------------------------------------------------ #
    # 内部阶段实现
    # ------------------------------------------------------------------ #

    def _step_flying(self, u: RallyJoinPosInputS, y: PosCalcOutputS) -> None:
        """在直飞阶段生成指向松散点的指令，并按剩余时间决定盘旋或切出。"""
        d_e = self._slot.east - u.selfState.pos.east
        d_n = self._slot.north - u.selfState.pos.north
        d_h = self._slot.h - u.selfState.pos.h
        d_horiz = math.sqrt(d_e * d_e + d_n * d_n)
        d_3d = math.sqrt(d_horiz * d_horiz + d_h * d_h)

        speed = self._approach_speed
        if self._slow_radius_m > 0.0:
            speed *= min(1.0, max(0.0, d_horiz / self._slow_radius_m))
        speed = max(self._speed_min, speed)

        self._eta_s = u.t_now + d_3d / max(speed, 0.1)

        y.selfCmd.pos.east = self._slot.east
        y.selfCmd.pos.north = self._slot.north
        y.selfCmd.pos.h = self._slot.h
        if d_horiz >= _EPSILON_HORIZ:
            y.selfCmd.v.vPsi = math.atan2(d_n, d_e)
            y.selfCmd.v.vEast = speed * d_e / d_horiz
            y.selfCmd.v.vNorth = speed * d_n / d_horiz
            y.selfCmd.v.vd = speed
            v_up_raw = speed * d_h / max(d_horiz, 1.0)
            y.selfCmd.v.vUp = max(-3.0, min(3.0, v_up_raw))
        else:
            y.selfCmd.v.vPsi = u.selfState.v.vPsi
            y.selfCmd.v.vEast = 0.0
            y.selfCmd.v.vNorth = 0.0
            y.selfCmd.v.vd = 0.0
            y.selfCmd.v.vUp = max(-3.0, min(3.0, d_h * 0.5))
        y.selfCmd.v.dVPsi = 0.0
        y.selfCmd.v.vTheta = 0.0

        if d_3d < self._arrival_radius_m:
            if not u.t_ref_valid:
                # 冷启动尚未收齐参与者 ETA 时，早到机先以最低速度盘旋，禁止按默认零时刻切出。
                self._enter_loiter(u.selfState.v.vPsi, float("inf"))
                return
            # 与盘旋过槽位点时使用同一贪心逻辑：
            # remaining < t_loop_min/2 时，进入盘旋再切出不如直接切出误差更小
            remaining = u.t_ref - u.t_now
            if self._should_exit(remaining):
                self._state = RALLY_STATE_EXITED
                self._eta_s = u.t_now
            else:
                self._enter_loiter(u.selfState.v.vPsi, remaining)

    def _enter_loiter(self, arrival_psi: float, remaining: float) -> None:
        """进入盘旋：以松散点为圆上一点，圆心在到达时速度方向的左侧 R 处（CCW 盘旋）。"""
        r = self._loiter_radius
        # 速度方向 psi 的左侧垂直方向：(-sin(psi), cos(psi))
        self._loiter_center_e = self._slot.east - r * math.sin(arrival_psi)
        self._loiter_center_n = self._slot.north + r * math.cos(arrival_psi)
        # 松散点在盘旋圆上的角度
        d_e = self._slot.east - self._loiter_center_e
        d_n = self._slot.north - self._loiter_center_n
        self._theta_slot = math.atan2(d_n, d_e)
        self._loiter_speed = self._adjusted_speed(remaining)
        self._away_from_slot = False
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

        # 飞盘旋圆：selfCmd.pos = 圆心（提供向心位置误差），v = CCW 切线速度
        y.selfCmd.pos.east = self._loiter_center_e
        y.selfCmd.pos.north = self._loiter_center_n
        y.selfCmd.pos.h = self._slot.h
        v_e = -self._loiter_speed * math.sin(theta)
        v_n = self._loiter_speed * math.cos(theta)
        y.selfCmd.v.vEast = v_e
        y.selfCmd.v.vNorth = v_n
        y.selfCmd.v.vPsi = math.atan2(v_n, v_e)
        y.selfCmd.v.vd = self._loiter_speed
        d_h = self._slot.h - u.selfState.pos.h
        y.selfCmd.v.vUp = max(-3.0, min(3.0, d_h * 0.3))
        y.selfCmd.v.dVPsi = self._loiter_speed / max(actual_r, 1.0)  # CCW 向心前馈（用实际半径）
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
        y.selfCmd.v.vUp = max(-3.0, min(3.0, d_h * 0.3))
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
