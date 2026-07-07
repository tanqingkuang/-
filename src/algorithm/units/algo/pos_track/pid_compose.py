"""PID 组合式位置跟踪。注意：三轴共用双通道 PID，前向速度/位置环由增益切换，苏联式法向/侧向恒为位置环。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.units.algo.ctrl.base import CtrlBase, CtrlInitS
from src.algorithm.units.algo.ctrl.pid import Pid
from src.algorithm.units.algo.ctrl.ppi import PPI, PPIInitS
from src.algorithm.units.algo.formation_math import enu_to_track, track_to_enu
from src.algorithm.units.algo.pos_track.base import PosTrackBase, PosTrackInitS, PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.lateral_track_angle import LateralTrackAngle, LateralTrackAngleInitS


def _uses_position_error(cfg: CtrlInitS | PPIInitS | LateralTrackAngleInitS | None) -> bool:
    """判断控制配置是否会消费位置误差。注意：只服务诊断屏蔽，不改变控制计算。"""
    if isinstance(cfg, PPIInitS):
        # PPI 的位置误差只进外环 kpPos；kpVel/kiVel 属于速度内环，不能让位置诊断误报有效。
        return cfg.kpPos != 0.0
    if isinstance(cfg, LateralTrackAngleInitS):
        # 串级横侧向的外环消费横偏 dZ(vel_err_cmd=-kpPos·dZ)，kpPos 非零即启用位置通道。
        return cfg.kpPos != 0.0
    if isinstance(cfg, CtrlInitS):
        # 并联式 PID 中 kp/ki 是位置通道，kd/kiv 只说明速度误差通道启用。
        return cfg.kp != 0.0 or cfg.ki != 0.0
    return False


def _uses_velocity_error(cfg: CtrlInitS | PPIInitS | LateralTrackAngleInitS | None) -> bool:
    """判断控制配置是否会消费速度误差。注意：前馈速度本身不算速度误差闭环。"""
    if isinstance(cfg, PPIInitS):
        # PPI 的内环跟踪 vel_cmd-velActual，因此 kpVel/kiVel 任一非零都表示速度误差被使用。
        return cfg.kpVel != 0.0 or cfg.kiVel != 0.0
    if isinstance(cfg, LateralTrackAngleInitS):
        # 串级横侧向内环跟踪侧向速度误差(velErr-velErrCmd)，kpVel/kiVel 任一非零即启用速度通道。
        return cfg.kpVel != 0.0 or cfg.kiVel != 0.0
    if isinstance(cfg, CtrlInitS):
        # 并联式 PID 的速度误差只走 kd/kiv；位置环开启不代表速度诊断有效。
        return cfg.kd != 0.0 or cfg.kiv != 0.0
    return False


def _make_ctrl(cfg: CtrlInitS | PPIInitS | None) -> CtrlBase:
    """按配置类型选择控制律：PPIInitS->串级 P+PI，否则->并联式 Pid。注意：未提供配置时退化为零增益 Pid。"""
    if isinstance(cfg, PPIInitS):
        ctrl: CtrlBase = PPI()
        ctrl.init(cfg)
        return ctrl
    ctrl = Pid()
    ctrl.init(cfg or CtrlInitS())
    return ctrl


@dataclass
class PidComposeInitS(PosTrackInitS):
    """PID 组合跟踪初始化参数。注意：vMin 用于避免低速航迹系奇异；各轴增益给 PPIInitS 则走串级 P+PI(可限速)，给 CtrlInitS 则走并联式 Pid。"""

    vMin: float = 0.5
    gainForward: CtrlInitS | PPIInitS | None = None
    gainLateral: CtrlInitS | PPIInitS | LateralTrackAngleInitS | None = None
    gainVertical: CtrlInitS | PPIInitS | None = None


class PidCompose(PosTrackBase):
    """组合式 PID 位置跟踪器。注意：三轴统一用双通道 PID，前向走速度环(长机)或位置环(僚机)由增益决定，法向/侧向恒按位置误差闭环。"""

    def __init__(self) -> None:
        """初始化 PidCompose 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._v_min = PidComposeInitS.vMin
        self._forward: CtrlBase = Pid()
        self._lateral: CtrlBase = Pid()
        self._lateral_cascade: LateralTrackAngle | None = None  # 非 None 时横侧向走串级+变限幅
        self._vertical: CtrlBase = Pid()
        self._diag_pos_enabled = (False, False, False)
        self._diag_vel_enabled = (False, False, False)

    def init(self, cfg: PidComposeInitS) -> None:
        """按配置初始化 PidCompose。注意：调用方需先准备好必要依赖和输入数据。"""
        self._v_min = cfg.vMin
        self._forward = _make_ctrl(cfg.gainForward)
        # 横侧向：LateralTrackAngleInitS 走串级+航迹角变限幅(需本机地速，见 step)；否则退回并联/串级 Pid。
        if isinstance(cfg.gainLateral, LateralTrackAngleInitS):
            self._lateral_cascade = LateralTrackAngle()
            self._lateral_cascade.init(cfg.gainLateral)
            self._lateral = Pid()  # 占位，串级激活时不参与计算
        else:
            self._lateral_cascade = None
            self._lateral = _make_ctrl(cfg.gainLateral)
        self._vertical = _make_ctrl(cfg.gainVertical)
        # 诊断字段轴序固定为 x/y/z = 前向/垂向/右侧向，配置字段顺序则是 Forward/Vertical/Lateral。
        self._diag_pos_enabled = (
            _uses_position_error(cfg.gainForward),
            _uses_position_error(cfg.gainVertical),
            _uses_position_error(cfg.gainLateral),
        )
        self._diag_vel_enabled = (
            _uses_velocity_error(cfg.gainForward),
            _uses_velocity_error(cfg.gainVertical),
            _uses_velocity_error(cfg.gainLateral),
        )

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
        # 误差/速度一律投到"目标速度系"(selfCmd 的航迹系)：横偏相对目标航路度量，
        # 规避以本机航迹系度量时"目标落在机头后方→越滚越偏"的正反馈(转圈)根因；
        # 前向误差随之变为"沿目标航向的前后"，天然覆盖"飞机比目标点靠前→前向环减速后退"。
        # 目标水平速度过低(集结起步/悬停)时其航向无定义，退回本机航迹系兜底，避免建基奇异。
        frame = u.selfCmd
        if math.hypot(u.selfCmd.v.vEast, u.selfCmd.v.vNorth) < self._v_min:
            frame = u.selfState
        pos_err = enu_to_track(pos_err_enu, frame)
        self_vel = enu_to_track(
            (u.selfState.v.vEast, u.selfState.v.vNorth, u.selfState.v.vUp),
            frame,
        )
        trim_vel = enu_to_track(
            (u.selfCmd.v.vEast, u.selfCmd.v.vNorth, u.selfCmd.v.vUp),
            frame,
        )
        # 各轴速度前馈(原指令)与实测速度：前向用地速标量 vd，法向/侧向用航迹系分量。
        vel_ff = (u.selfCmd.v.vd, trim_vel[1], trim_vel[2])
        vel_actual = (u.selfState.v.vd, self_vel[1], self_vel[2])
        vel_err = (
            vel_ff[0] - vel_actual[0],
            vel_ff[1] - vel_actual[1],
            vel_ff[2] - vel_actual[2],
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
            # 航迹系诊断只报告真正进入控制律的误差信号；未启用的通道置 0，避免被离线分析误判。
            y.diag.track_pos_err_x_m = pos_err[0] if self._diag_pos_enabled[0] else 0.0
            y.diag.track_pos_err_y_m = pos_err[1] if self._diag_pos_enabled[1] else 0.0
            y.diag.track_pos_err_z_m = pos_err[2] if self._diag_pos_enabled[2] else 0.0
            y.diag.track_vel_err_x_mps = vel_err[0] if self._diag_vel_enabled[0] else 0.0
            y.diag.track_vel_err_y_mps = vel_err[1] if self._diag_vel_enabled[1] else 0.0
            y.diag.track_vel_err_z_mps = vel_err[2] if self._diag_vel_enabled[2] else 0.0

        # 航迹偏航角速率前馈(向心加速度)：在侧向轴直接补出维持转弯所需的 a_lat = vd·dVPsi。
        # 侧向轴(lateral_right)以右为正，而 dVPsi>0 为左转，故取负号。
        # 必须用**本机自身地速** selfState.v.vd：飞机偏航率 psi_dot=a_lat/V_self(见 model.py)，
        # 要用前馈产生角速率 dVPsi 就得按本机速度换算 a_lat=dVPsi·V_self；用目标速度会在
        # V_self≠V_cmd(加减速/速度未收敛)时给出错误前馈。外/内侧僚机半径速度差异经 dVPsi 自动吸收。
        lateral_ff = -u.selfCmd.v.dVPsi * u.selfState.v.vd
        # 前向/法向：step(位置误差, 速度前馈, 实测速度)——Pid 走并联式、PPI 走串级 P+PI。
        # 横侧向：LateralTrackAngle 走串级 + 航迹角变限幅(消除大侧偏持续滚转→转圈)，需本机地速；
        #        无该配置时退回并联/串级 Pid(旧行为)。两路均叠加向心前馈 lateral_ff。
        if self._lateral_cascade is not None:
            lateral_acc = self._lateral_cascade.step(pos_err[2], vel_err[2], u.selfState.v.vd) + lateral_ff
        else:
            lateral_acc = self._lateral.step(pos_err[2], vel_ff[2], vel_actual[2]) + lateral_ff
        acc_track = (
            self._forward.step(pos_err[0], vel_ff[0], vel_actual[0]),
            self._vertical.step(pos_err[1], vel_ff[1], vel_actual[1]),
            lateral_acc,
        )
        acc_enu = track_to_enu(acc_track, frame)
        y.accCmd.accEast = acc_enu[0]
        y.accCmd.accNorth = acc_enu[1]
        y.accCmd.accUp = acc_enu[2]

    def reset(self) -> None:
        """复位 PidCompose 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._forward.reset()
        self._lateral.reset()
        if self._lateral_cascade is not None:
            self._lateral_cascade.reset()
        self._vertical.reset()
