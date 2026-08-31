"""无非惯性补偿的位置跟踪产品。注意：用于与旋转补偿和完整补偿产品做消融对照。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import MotionProfS, copy_motion
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.common.coordinates import fur_basis_from_velocity

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


class NoInertialPidCompose(PidCompose):
    """复用 PID 组合控制律，但在控制前移除槽位旋转与向心前馈。"""

    def __init__(self) -> None:
        """初始化消融产品及独立目标副本。注意：不得原地修改 PosCalc 输出。"""
        super().__init__()
        self._source_cmd = MotionProfS()
        self._filtered_cmd = MotionProfS()
        self._leader_state = MotionProfS()
        self._leader_cmd = MotionProfS()

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定跟踪端口和掌机参考状态。注意：控制核心只读取过滤后的目标副本。"""
        super().bind(runtime)
        context = runtime.context
        self._source_cmd = context.selfCmd
        self._leader_state = context.leaderState
        self._leader_cmd = context.leaderCmd
        self._u.selfCmd = self._filtered_cmd

    def step(self) -> None:
        """移除两项非惯性前馈后推进原 PID 组合控制律。"""
        if not self._bound:
            raise ValueError("NoInertialPidCompose 尚未绑定端口")
        self._remove_non_inertial_compensation()
        self._calculate(self._u, self._y)

    def _remove_non_inertial_compensation(self) -> None:
        """扣除 omega×r 运输速度并清零 dVPsi。注意：保留 TD 重构速度等其他前馈。"""
        copy_motion(self._source_cmd, self._filtered_cmd)
        omega = _reference_yaw_rate(self._leader_cmd, self._leader_state)
        relative_east = self._source_cmd.pos.east - self._leader_state.pos.east
        relative_north = self._source_cmd.pos.north - self._leader_state.pos.north
        # ENU 中正偏航角速度沿天轴，omega×r=(-omega*r_north, omega*r_east, 0)。
        self._filtered_cmd.v.vEast += omega * relative_north
        self._filtered_cmd.v.vNorth -= omega * relative_east
        self._filtered_cmd.v.dVPsi = 0.0
        horizontal_speed = math.hypot(
            self._filtered_cmd.v.vEast,
            self._filtered_cmd.v.vNorth,
        )
        self._filtered_cmd.v.vd = horizontal_speed
        self._filtered_cmd.v.vPsi = (
            math.atan2(self._filtered_cmd.v.vNorth, self._filtered_cmd.v.vEast)
            if horizontal_speed > 0.0
            else 0.0
        )


def _reference_yaw_rate(leader_cmd: MotionProfS, leader_state: MotionProfS) -> float:
    """按 SlotGeometry 的帧选择规则取得角速度。注意：无有效指令航迹时回退实际状态。"""
    try:
        fur_basis_from_velocity(
            (
                leader_cmd.v.vEast,
                leader_cmd.v.vNorth,
                leader_cmd.v.vUp,
            )
        )
    except ValueError:
        return leader_state.v.dVPsi
    return leader_cmd.v.dVPsi
