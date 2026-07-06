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
from src.algorithm.units.process.formation_task.hold import Hold, HoldTaskInitS
from src.algorithm.units.process.outbound.base import OutboundInitS, OutboundOutputS
from src.algorithm.units.process.outbound.rally_leader_broadcast import RallyLeaderBroadcast, RallyLeaderBroadcastInputS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute, LeaderRouteInitS

_LEADER_L1_DISTANCE_M = 0.0 # 关闭L1前瞻，直接按航段投影解算目标航迹。大侧偏限角保护已由横侧向变限幅(1.2)接管，L1 不再需要。
_LEADER_FF_LEAD_TIME_S = 0.5 # 曲率前馈前瞻时间 σ(秒)，前瞻窗长 L2=σ·vd；配为0则关闭曲率前馈。调参旋钮。

# —— 横侧向限幅调参旋钮（长机僚机共用；见 lateral_track_angle.py 与 docs/横侧向点号切入问题）——
# 分两层：外环"航迹角变限幅"(拦截角)与执行层"滚转角限幅"。
# 变限幅半径 R = V² / (g·sin(_LATERAL_GAMMA_MAX_RAD)) · _LATERAL_R_MARGIN；据此把大侧偏拦截角限到 [地板, 90°]。
# 注意：以下值暂借/试定，**尚未按本项目慢速编队机整定**，是留给后续手动调参的旋钮；
# 改动只影响切入的快慢/陡缓与转弯出力，不影响小侧偏(无饱和)时与旧并联式的等价行为。
_LATERAL_ROLL_MAX_RAD = math.radians(40.0)   # 执行层滚转角限幅：侧向加速度上限 = g·tan(40°)≈8.2 m/s²(模型硬限 70°)
_LATERAL_GAMMA_MAX_RAD = math.radians(25.0)  # 定 R 的最大航迹角(转弯半径尺度)：越小→R 越大→垂直切入触发越晚、切入越缓
_LATERAL_FLOOR_RAD = math.radians(7.0)       # 航迹角限幅地板：中心线附近的最小拦截角，防近线残余大角引发震荡
_LATERAL_R_MARGIN = 1.2                       # R 余量系数(>1 更保守，向上留裕度)


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
        self._outbound = RallyLeaderBroadcast()

        # 各单元一次性初始化；航路规划注入预置航线，广播注入本机 id 与拓扑
        self._task.init(HoldTaskInitS(initialPattern=cfg.commInit.initialPattern))
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
        # slotScale/t_ref 端口必须绑定（RallyLeaderBroadcast 强制校验 slotScale 非 None）；
        # hold 场景广播默认集结字段：scale=1.0/scaleRate=0.0/t_ref_valid=False 恒定不变，
        # 僚机用 RallyLeaderFollower 统一解析后仍按普通保持编队执行。
        self._outbound_u = RallyLeaderBroadcastInputS(
            cmd=self.cxt.cmd,
            selfState=self.cxt.selfState,
            slotScale=self.cxt.slotScale,
            t_ref=self.cxt.rally_t_ref,
            t_ref_valid=self.cxt.rally_t_ref_valid,
        )
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
    # 侧向(侧偏)改串级(P+PI)+航迹角变限幅：结构上仍等价于并联式 kp·dZ+kd·velErr(无饱和+ki=0)。
    # 注意 kd 已从旧自身系并联的 0.12 上调到 0.30——误差改到"目标速度系"度量后，丢了自身航迹系
    # 随机头旋转带来的纯追踪前置阻尼，照搬旧增益会欠阻尼振荡(base.json 僚机切入持续摆动)，故需更高阻尼。
    # 变限幅解决大侧偏"持续滚转→转圈"(见 lateral_track_angle 与 docs/横侧向点号切入问题)。两实体共用。
    # rollMax/gammaMax/floor/margin 为待整定旋钮，见文件顶部 _LATERAL_* 常量。执行层限滚转角而非侧向加速度。
    gain_lateral = LateralTrackAngleInitS(
        kp=0.02, ki=0.0, kd=0.30, dt=control_period_s, rollMaxRad=_LATERAL_ROLL_MAX_RAD,
        gammaMaxRad=_LATERAL_GAMMA_MAX_RAD, floorRad=_LATERAL_FLOOR_RAD, margin=_LATERAL_R_MARGIN,
    )
    # 垂向改串级 P+PI：按错层队形切换探针整定为较高阻尼，压低高度阶跃越零超调；
    # 垂向速度限幅 vCmdMin/vCmdMax 由配置注入(默认 ±inf 不限)，acc 限幅沿用 ±6。
    gain_vertical = PPIInitS(
        kpPos=0.25,
        kpVel=0.65,
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
    """生成僚机默认位置跟踪器配置。注意：前向串级 P+PI 已按 change.json 队形切换整定，避免外侧僚机越零超调。"""
    vel_limit = vel_limit or VelCmdLimitS()
    # 前向速度限幅由配置注入；提高速度内环阻尼后，90m 槽位突变不再越过目标点十几米。
    gain_forward = PPIInitS(
        kpPos=0.12,
        kpVel=0.32,
        kiVel=0.0,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.forwardMin,
        vCmdMax=vel_limit.forwardMax,
    )
    return _tracker_init(control_period_s, gain_forward, vel_limit)
