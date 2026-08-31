"""完整运动学前馈的位置跟踪产品。注意：仅用于平动/角加速度补偿的受控验证。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import AccInEarthS, MotionProfS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


class FullInertialPidCompose(PidCompose):
    """在现有位置反馈上叠加槽位目标完整水平运动学加速度。"""

    def __init__(self) -> None:
        """建立未绑定参考量和角速度差分状态。"""
        super().__init__()
        self._source_cmd = MotionProfS()
        self._leader_state = MotionProfS()
        self._leader_acc_cmd = AccInEarthS()
        self._leader_clock = None
        self._previous_sample_time_s: float | None = None
        self._previous_yaw_rate = 0.0

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定目标、长机状态、长机加速度指令和统一时钟。"""
        super().bind(runtime)
        context = runtime.context
        self._source_cmd = context.selfCmd
        self._leader_state = context.leaderState
        self._leader_acc_cmd = context.leaderAccCmd
        self._leader_clock = context.leaderClock

    def step(self) -> None:
        """计算完整目标加速度并推进原位置反馈控制律。"""
        if not self._bound or self._leader_clock is None:
            raise ValueError("FullInertialPidCompose 尚未绑定端口")
        acceleration_ff = self._target_acceleration(self._leader_clock.now_s)
        self._calculate(self._u, self._y, acceleration_ff)

    def _target_acceleration(self, sample_time_s: float) -> tuple[float, float, float]:
        """计算 aL+alpha×r+omega×(omega×r)。注意：当前只展开水平偏航旋转。"""
        # SlotGeometry 已把实际采用的长机参考帧转率写入 selfCmd，必须复用同一基准。
        omega = self._source_cmd.v.dVPsi
        alpha = 0.0
        if self._previous_sample_time_s is None:
            self._previous_sample_time_s = sample_time_s
            self._previous_yaw_rate = omega
        elif sample_time_s > self._previous_sample_time_s:
            elapsed_s = sample_time_s - self._previous_sample_time_s
            alpha = (omega - self._previous_yaw_rate) / elapsed_s
            self._previous_sample_time_s = sample_time_s
            self._previous_yaw_rate = omega
        # 重复或更旧的报文不得推进差分基准；入站层也会拒绝旧快照。

        relative_east = self._source_cmd.pos.east - self._leader_state.pos.east
        relative_north = self._source_cmd.pos.north - self._leader_state.pos.north
        omega_sq = omega * omega
        return (
            self._leader_acc_cmd.accEast - alpha * relative_north - omega_sq * relative_east,
            self._leader_acc_cmd.accNorth + alpha * relative_east - omega_sq * relative_north,
            self._leader_acc_cmd.accUp,
        )

    def reset(self) -> None:
        """复位 PID 与角速度差分状态。"""
        super().reset()
        self._previous_sample_time_s = None
        self._previous_yaw_rate = 0.0
