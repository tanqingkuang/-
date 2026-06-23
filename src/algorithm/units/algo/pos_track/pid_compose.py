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
        vel_err_enu = (
            u.selfCmd.v.vEast - u.selfState.v.vEast,
            u.selfCmd.v.vNorth - u.selfState.v.vNorth,
            u.selfCmd.v.vUp - u.selfState.v.vUp,
        )
        pos_err = enu_to_track(pos_err_enu, u.selfState)
        vel_err = enu_to_track(vel_err_enu, u.selfState)

        acc_track = (
            self._forward.step(0.0, vel_err[0]),
            self._vertical.step(pos_err[1], vel_err[1]),
            self._lateral.step(pos_err[2], vel_err[2]),
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
