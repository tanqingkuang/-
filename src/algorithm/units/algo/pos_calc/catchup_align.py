"""追赶对齐：切出后沿任务航向直线飞行，按沿航迹误差调速，使全机同步进入松散槽位。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormCommInitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.scaled_slot_geometry import ScaledSlotGeometry, ScaledSlotInitS, ScaledSlotInputS


@dataclass
class CatchupAlignInitS:
    """CatchupAlign 初始化参数。"""

    selfId: str = ""
    commInit: FormCommInitS = field(default_factory=FormCommInitS)
    mission_heading_rad: float = 0.0   # 切出后任务航向（弧度，从东向起算）
    mi_east: float = 0.0               # M_i 点 ENU 东坐标（用于锁定"杆"的横侧向位置）
    mi_north: float = 0.0              # M_i 点 ENU 北坐标
    nominal_speed_mps: float = 20.0    # 额定速度（与长机任务航线速度一致）
    kp_speed: float = 0.05             # 沿航迹误差增益（m/s per m）
    speed_min_mps: float = 14.0        # 速度下限
    speed_max_mps: float = 25.0        # 速度上限


class CatchupAlign:
    """追赶对齐单元：维持任务航向，通过速度调节闭合沿航迹方向的槽位误差。

    "珠子串在杆上"模型：
    - 杆 = 过 M_i 点、平行任务航向的直线（横航迹坐标 _mi_cross 在 init 时固定）
    - 沿航迹目标 slot_along = ScaledSlotGeometry 给出（随掌机位置动）
    - selfCmd.pos = 本机在杆上的正交投影（same along-track, _mi_cross）
      → track-frame 前向误差恒为 0，横向误差 = self.cross − _mi_cross，把飞机推回杆上
    - 前向：speed = clamp(v_nominal + kp × along_track_err, v_min, v_max)，仅调速不倒飞
    - 误差统计：pos_err_m = dist2d(self, slot)，供广播覆盖 posErr_m
    """

    #: 最近一帧到槽位目标的 2D 距离（供 follower._update_outbound 覆盖 posErr_m）
    pos_err_m: float = 0.0
    #: 真实槽位的 ENU 坐标（slot_along, _mi_cross 反算），供 GUI 显示用
    true_slot_east: float = 0.0
    true_slot_north: float = 0.0

    def init(self, cfg: CatchupAlignInitS) -> None:
        """按配置初始化追赶对齐几何、速度增益与固定横侧向基准。"""
        self._slot_geom = ScaledSlotGeometry()
        self._slot_geom.init(ScaledSlotInitS(selfId=cfg.selfId, commInit=cfg.commInit))
        self._cos_h = math.cos(cfg.mission_heading_rad)
        self._sin_h = math.sin(cfg.mission_heading_rad)
        self._heading = cfg.mission_heading_rad
        self._v_nominal = cfg.nominal_speed_mps
        self._kp = cfg.kp_speed
        self._v_min = cfg.speed_min_mps
        self._v_max = cfg.speed_max_mps
        # 杆的横侧向坐标：M_i 点在 track-frame 的 cross-track 坐标，初始化后固定
        self._mi_cross = -cfg.mi_east * self._sin_h + cfg.mi_north * self._cos_h

    def step(self, u: ScaledSlotInputS, y: PosCalcOutputS) -> None:
        """计算真实槽位误差，并输出沿任务航向调速、横向回杆的位置速度指令。"""
        if u.selfState is None or y.selfCmd is None:
            raise ValueError("CatchupAlign ports must be bound")

        # 从 ScaledSlotGeometry 取沿航迹目标（slot_along）和高度
        self._slot_geom.step(u, y)
        slot_e = y.selfCmd.pos.east
        slot_n = y.selfCmd.pos.north
        slot_h = y.selfCmd.pos.h

        self_e = u.selfState.pos.east
        self_n = u.selfState.pos.north

        # track 坐标分解
        self_along = self_e * self._cos_h + self_n * self._sin_h
        self_cross = -self_e * self._sin_h + self_n * self._cos_h
        slot_along = slot_e * self._cos_h + slot_n * self._sin_h

        along_track_err = slot_along - self_along      # 正=落后，负=超前
        cross_track_err = self._mi_cross - self_cross  # 横向偏差（推回杆）

        # 广播用指标：到槽位（slot_along, _mi_cross）的 2D 距离
        self.pos_err_m = math.hypot(along_track_err, cross_track_err)

        # 速度调制：超前减速，落后加速
        speed = max(self._v_min, min(self._v_max, self._v_nominal + self._kp * along_track_err))

        # 真实槽位（slot_along, _mi_cross）→ ENU，仅供 GUI 显示
        self.true_slot_east = slot_along * self._cos_h - self._mi_cross * self._sin_h
        self.true_slot_north = slot_along * self._sin_h + self._mi_cross * self._cos_h

        # 位置控制器目标：本机在"杆"上的正交投影（self_along, _mi_cross）
        # 前向误差=0，位置PID只产生横向修正；沿航迹收敛由速度调制承担
        proj_e = self_along * self._cos_h - self._mi_cross * self._sin_h
        proj_n = self_along * self._sin_h + self._mi_cross * self._cos_h
        y.selfCmd.pos.east = proj_e
        y.selfCmd.pos.north = proj_n
        y.selfCmd.pos.h = slot_h

        # 速度指令：锁定任务航向
        y.selfCmd.v.vEast = speed * self._cos_h
        y.selfCmd.v.vNorth = speed * self._sin_h
        y.selfCmd.v.vPsi = self._heading
        y.selfCmd.v.vd = speed
        d_h = slot_h - u.selfState.pos.h
        y.selfCmd.v.vUp = max(-3.0, min(3.0, d_h * 0.3))
        y.selfCmd.v.dVPsi = 0.0
        y.selfCmd.v.vTheta = 0.0

    def reset(self) -> None:
        """复位内部槽位几何及对外暴露的最近一帧诊断量。"""
        self._slot_geom.reset()
        self.pos_err_m = 0.0
        self.true_slot_east = 0.0
        self.true_slot_north = 0.0
