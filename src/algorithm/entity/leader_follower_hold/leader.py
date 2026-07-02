"""领航跟随保持场景的长机实体。注意：负责规划航线并广播状态。"""

from __future__ import annotations

import math

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    PosInEarthS,
    PosTrackDiagS,
    RemoteCmdS,
    WayLineS,
    WayPointInputS,
    WayPointS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import (
    DEFAULT_CONTROL_PERIOD_S,
    EntityInitS,
    EntityInputS,
    EntityOutputS,
    VelCmdLimitS,
)
from src.algorithm.units.algo.arc_path import corner_arc
from src.algorithm.units.algo.ctrl.ppi import PPIInitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS, RouteInterpInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.lateral_track_angle import LateralTrackAngleInitS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS
from src.algorithm.units.process.formation_task.base import FormationTaskInputS, FormationTaskOutputS
from src.algorithm.units.process.formation_task.hold import Hold
from src.algorithm.units.process.outbound.base import OutboundInputS, OutboundOutputS
from src.algorithm.units.process.outbound.leader_broadcast import LeaderBroadcast, OutboundInitS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute, LeaderRouteInitS

_LEADER_L1_DISTANCE_M = 0.0 # 关闭L1前瞻，直接按航段投影解算目标航迹。大侧偏限角保护已由横侧向变限幅(1.2)接管，L1 不再需要。
_LEADER_FF_LEAD_TIME_S = 0.5 # 曲率前馈前瞻时间 σ(秒)，前瞻窗长 L2=σ·vd；配为0则关闭曲率前馈。调参旋钮。


class LeaderEntity(EntityBase):
    """长机实体：串联任务编排、航路规划、位置解算、跟踪与广播五级处理链。注意：上下文 cxt 在各单元间共享，单元仅读写各自绑定的端口。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 LeaderEntity。注意：调用方需先准备好必要依赖和输入数据。"""
        # 共享黑板：所有处理单元通过它读写状态，避免逐级显式传参
        self.cxt = FormContextS()
        self._remote = RemoteCmdS()  # 缓存外部遥控指令，跨帧保留
        self._outbox = []  # 复用同一列表对象，避免每帧重新分配

        # 长机处理链：任务编排 -> 航路规划 -> 位置解算 -> 位置跟踪 -> 出站广播
        self._task = Hold()
        self._tra_plan = LeaderRoute()
        self._pos_calc = RouteInterp()
        self._pos_track = PidCompose()
        self._outbound = LeaderBroadcast()

        # 各单元一次性初始化；航路规划注入预置航线，广播注入本机 id 与拓扑
        self._task.init(None)
        route_lines = waypoint_inputs_to_waylines(cfg.route) if len(cfg.route) >= 2 else None
        self._tra_plan.init(LeaderRouteInitS(route_lines))
        self._pos_calc.init(RouteInterpInitS(lookAheadDistance=_LEADER_L1_DISTANCE_M, leadTimeS=_LEADER_FF_LEAD_TIME_S))
        self._pos_track.init(_default_tracker_init(cfg.control_period_s, cfg.velCmdLimit))
        self._outbound.init(OutboundInitS(cfg.selfInit.id, cfg.commInit.netWork))

        # 预先绑定各单元输入/输出端口到共享黑板字段，step 时直接复用、零拷贝串联
        self._task_u = FormationTaskInputS(remote=self._remote, cmd=self.cxt.cmd)
        self._task_y = FormationTaskOutputS(cmd=self.cxt.cmd)
        # 航路规划读编队指令与本机状态，输出当前航段写回黑板的 wayLine
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine, nextWayLine=self.cxt.nextWayLine)
        self._pos_calc_u = RouteInterpInputS(
            selfState=self.cxt.selfState, wayLine=self.cxt.wayLine, nextWayLine=self.cxt.nextWayLine
        )
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_diag = PosTrackDiagS()
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd, diag=self._pos_track_diag)
        self._outbound_u = OutboundInputS(cmd=self.cxt.cmd, selfState=self.cxt.selfState)
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 LeaderEntity 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        # 先把外部输入灌入黑板：状态反馈用深拷贝，遥控仅取目标阶段
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = u.remote.stage
        # 按固定顺序推进处理链，前级输出即后级输入（经黑板传递）
        self._task.step(self._task_u, self._task_y)
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)
        self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
        self._pos_track.step(self._pos_track_u, self._pos_track_y)
        self._outbound.step(self._outbound_u, self._outbound_y)
        # 回填加速度指令：调用方未提供容器则直接引用，否则逐字段写入避免改变其引用
        if y.selfAccCmd is None:
            y.selfAccCmd = self.cxt.selfAccCmd
        else:
            y.selfAccCmd.accEast = self.cxt.selfAccCmd.accEast
            y.selfAccCmd.accNorth = self.cxt.selfAccCmd.accNorth
            y.selfAccCmd.accUp = self.cxt.selfAccCmd.accUp
        if y.selfCmd is None:
            y.selfCmd = self.cxt.selfCmd
        else:
            copy_motion(self.cxt.selfCmd, y.selfCmd)
        if y.controlDiag is None:
            y.controlDiag = self._pos_track_diag
        else:
            copy_pos_track_diag(self._pos_track_diag, y.controlDiag)
        # 把本帧广播消息搬运到输出（先清空避免残留上一帧）
        y.outbox.clear()
        y.outbox.extend(self._outbox)

    def reset(self) -> None:
        """复位 LeaderEntity 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        # 原地清空黑板与各单元运行期状态，保留构造期注入的依赖
        reset_context(self.cxt)
        self._remote.stage = RemoteCmdS().stage  # 遥控阶段回到默认
        self._task.reset()
        self._tra_plan.reset()
        self._pos_calc.reset()
        self._pos_track.reset()
        self._outbound.reset()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._outbox.clear()

    def close(self) -> None:
        """释放 LeaderEntity 持有的资源。注意：关闭后不应继续调用运行接口。"""
        return None


def waypoint_inputs_to_waylines(inputs: list[WayPointInputS]) -> list[WayLineS]:
    """将原始航点转换为内部 WayLineS 序列，并按需展开圆弧几何。

    情况 1：turnSign != 0 表示外部已算好的圆弧，直接映射。
    情况 2：内部拐点 r > 0 时用 corner_arc() 计算相切圆弧。
    默认：按普通折线直连。
    """
    if len(inputs) < 2:
        raise ValueError("at least 2 waypoints required")
    nodes: list[WayPointS] = []
    for i, wpi in enumerate(inputs):
        if wpi.turnSign != 0.0:
            nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd, turnSign=wpi.turnSign, center=wpi.center))
        elif 0 < i < len(inputs) - 1 and wpi.r > 0.0:
            arc = corner_arc(inputs[i - 1].pos, wpi.pos, inputs[i + 1].pos, wpi.r)
            if arc is not None:
                t1, t2, center, turn_sign = arc
                # corner_arc 只负责几何求解，调用方需保证切点仍落在两条原始航腿内。
                in_leg = _horizontal_distance(inputs[i - 1].pos, wpi.pos)
                out_leg = _horizontal_distance(wpi.pos, inputs[i + 1].pos)
                tangent_in = _horizontal_distance(t1, wpi.pos)
                tangent_out = _horizontal_distance(t2, wpi.pos)
                if tangent_in <= in_leg + 1e-9 and tangent_out <= out_leg + 1e-9:
                    nodes.append(
                        WayPointS(idx=wpi.idx, pos=t1, vdCmd=inputs[i - 1].vdCmd, turnSign=turn_sign, center=center)
                    )
                    nodes.append(WayPointS(idx=wpi.idx, pos=t2, vdCmd=wpi.vdCmd))
                else:
                    # 半径过大时切点会越过相邻航点，保持折线比插入反向圆弧更安全。
                    nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
            else:
                nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
        else:
            nodes.append(WayPointS(idx=wpi.idx, pos=wpi.pos, vdCmd=wpi.vdCmd))
    return [WayLineS(idx=j, start=nodes[j], end=nodes[j + 1]) for j in range(len(nodes) - 1)]


def _horizontal_distance(a: PosInEarthS, b: PosInEarthS) -> float:
    """计算两点水平距离。注意：圆弧切点合法性只看东/北平面。"""
    return math.hypot(a.east - b.east, a.north - b.north)


def _tracker_init(control_period_s: float, gain_forward: PPIInitS, vel_limit: VelCmdLimitS) -> PidComposeInitS:
    """按给定前向增益生成位置跟踪器配置。注意：前向/垂向走串级 P+PI(可限速)，侧向恒为位置环 Pid，长机与僚机只在前向通道有别。"""
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    # 侧向(侧偏)改串级(P+PI)+航迹角变限幅：增益 kp/kd/ki 与旧并联式一一等价(无饱和+ki=0 时严格相等)，
    # 变限幅解决大侧偏"持续滚转→转圈"(见 lateral_track_angle 与 docs/横侧向点号切入问题)。两实体共用。
    gain_lateral = LateralTrackAngleInitS(kp=0.02, ki=0.0, kd=0.12, dt=control_period_s, outMax=4.0)
    # 垂向改串级 P+PI：等价旧 kp=0.2/kd=0.6(kpPos=kp/kd)，acc 限幅沿用 ±6；
    # 垂向速度限幅 vCmdMin/vCmdMax 由配置注入(默认 ±inf 不限)。
    gain_vertical = PPIInitS(
        kpPos=0.2 / 0.6,
        kpVel=0.6,
        kiVel=0.0,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.verticalMin,
        vCmdMax=vel_limit.verticalMax,
    )
    return PidComposeInitS(0.5, gain_forward, gain_lateral, gain_vertical)


def _default_tracker_init(
    control_period_s: float = DEFAULT_CONTROL_PERIOD_S, vel_limit: VelCmdLimitS | None = None
) -> PidComposeInitS:
    """生成长机默认位置跟踪器配置。注意：前向串级 P+PI 退化为纯速度环(外环 kpPos=0，速度比例走 kpVel)，内环积分 kiVel 预留默认 0。"""
    vel_limit = vel_limit or VelCmdLimitS()
    # 长机无前向位置基准，外环 kpPos=0 -> vel_cmd=velFf，纯内环 PI 速度环；前向速度限幅由配置注入。
    gain_forward = PPIInitS(
        kpPos=0.0,
        kpVel=1.0,
        kiVel=0.0,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.forwardMin,
        vCmdMax=vel_limit.forwardMax,
    )
    return _tracker_init(control_period_s, gain_forward, vel_limit)


def _follower_tracker_init(
    control_period_s: float = DEFAULT_CONTROL_PERIOD_S, vel_limit: VelCmdLimitS | None = None
) -> PidComposeInitS:
    """生成僚机默认位置跟踪器配置。注意：前向串级 P+PI，等价旧 kp=0.02/kd=0.12(kpPos=kp/kd)，增益待整定。"""
    vel_limit = vel_limit or VelCmdLimitS()
    # 前向速度限幅由配置注入；前向速度恒正，按需设 forwardMin>0、forwardMax 上限。
    gain_forward = PPIInitS(
        kpPos=0.02 / 0.12,
        kpVel=0.12,
        kiVel=0.0,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.forwardMin,
        vCmdMax=vel_limit.forwardMax,
    )
    return _tracker_init(control_period_s, gain_forward, vel_limit)
