"""PID 组合式位置跟踪。注意：前向速度和苏联式法向/侧向位置环分开处理。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.ctrl.pid import Pid
from src.algorithm.units.algo.formation_math import enu_to_track, track_to_enu
from src.algorithm.units.algo.pos_track.base import PosTrackBase, PosTrackInitS, PosTrackInputS, PosTrackOutputS


@dataclass
class PidComposeInitS(PosTrackInitS):
    """PID 组合跟踪初始化参数。注意：vMin 用于避免低速航迹系奇异。"""

    vMin: float = 0.5
    gainForward: CtrlInitS | None = None
    gainLateral: CtrlInitS | None = None
    gainVertical: CtrlInitS | None = None


class PidCompose(PosTrackBase):
    """组合式 PID 位置跟踪器。注意：前向只控速度，法向和侧向右按位置误差闭环。"""

    def __init__(self) -> None:
        """初始化 PidCompose 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._v_min = PidComposeInitS.vMin
        self._forward = Pid()
        self._lateral = Pid()
        self._vertical = Pid()

    def init(self, cfg: PidComposeInitS) -> None:
        """按配置初始化 PidCompose。注意：调用方需先准备好必要依赖和输入数据。"""
        self._v_min = cfg.vMin
        self._forward.init(cfg.gainForward or CtrlInitS())
        self._lateral.init(cfg.gainLateral or CtrlInitS())
        self._vertical.init(cfg.gainVertical or CtrlInitS())

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        """推进 PidCompose 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.selfCmd is None or u.selfState is None or y.accCmd is None:
            raise ValueError("PidCompose ports must be bound")
        if u.selfState.v.vd < self._v_min:
            raise ValueError(f"ground speed below vMin: {u.selfState.v.vd} < {self._v_min}")

        pos_err_enu = (
            u.selfCmd.pos.east - u.selfState.pos.east,
            u.selfCmd.pos.north - u.selfState.pos.north,
            u.selfCmd.pos.h - u.selfState.pos.h,
        )
        pos_err = enu_to_track(pos_err_enu, u.selfState)
        self_vel = enu_to_track(
            (u.selfState.v.vEast, u.selfState.v.vNorth, u.selfState.v.vUp),
            u.selfState,
        )
        trim_vel = enu_to_track(
            (u.selfCmd.v.vEast, u.selfCmd.v.vNorth, u.selfCmd.v.vUp),
            u.selfState,
        )
        vel_err = (
            u.selfCmd.v.vd - u.selfState.v.vd,
            trim_vel[1] - self_vel[1],
            trim_vel[2] - self_vel[2],
        )
        if y.diag is not None:
            y.diag.cmd_pos_east_m = u.selfCmd.pos.east
            y.diag.cmd_pos_north_m = u.selfCmd.pos.north
            y.diag.cmd_pos_h_m = u.selfCmd.pos.h
            y.diag.cmd_vel_east_mps = u.selfCmd.v.vEast
            y.diag.cmd_vel_north_mps = u.selfCmd.v.vNorth
            y.diag.cmd_vel_up_mps = u.selfCmd.v.vUp
            y.diag.pos_err_east_m = pos_err_enu[0]
            y.diag.pos_err_north_m = pos_err_enu[1]
            y.diag.pos_err_h_m = pos_err_enu[2]
            y.diag.vel_err_east_mps = u.selfCmd.v.vEast - u.selfState.v.vEast
            y.diag.vel_err_north_mps = u.selfCmd.v.vNorth - u.selfState.v.vNorth
            y.diag.vel_err_up_mps = u.selfCmd.v.vUp - u.selfState.v.vUp
            y.diag.track_pos_err_x_m = pos_err[0]
            y.diag.track_pos_err_y_m = pos_err[1]
            y.diag.track_pos_err_z_m = pos_err[2]
            y.diag.track_vel_err_x_mps = vel_err[0]
            y.diag.track_vel_err_y_mps = vel_err[1]
            y.diag.track_vel_err_z_mps = vel_err[2]

        # 航迹偏航角速率前馈(向心加速度)：在航迹系侧向直接补出维持转弯所需的 vd·dVPsi。
        # 本机航迹系第三轴(lateral_right)以右为正，而 dVPsi>0 为左转，故取负号；
        # 配合本机自身 vd，外/内侧僚机的半径与速度差异被自动吸收(v_S/R_S = dVPsi)。
        lateral_ff = -u.selfCmd.v.dVPsi * u.selfState.v.vd
        acc_track = (
            self._forward.step(vel_err[0], 0.0),
            self._vertical.step(pos_err[1], vel_err[1]),
            self._lateral.step(pos_err[2], vel_err[2]) + lateral_ff,
        )
        acc_enu = track_to_enu(acc_track, u.selfState)
        y.accCmd.accEast = acc_enu[0]
        y.accCmd.accNorth = acc_enu[1]
        y.accCmd.accUp = acc_enu[2]

    def reset(self) -> None:
        """复位 PidCompose 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._forward.reset()
        self._lateral.reset()
        self._vertical.reset()
