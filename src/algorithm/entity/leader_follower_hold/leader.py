"""领航跟随保持场景的长机实体。注意：负责规划航线并广播状态。"""

from __future__ import annotations

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import RemoteCmdS, copy_motion
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import DEFAULT_CONTROL_PERIOD_S, EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS, RouteInterpInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS
from src.algorithm.units.process.formation_task.base import FormationTaskInputS, FormationTaskOutputS
from src.algorithm.units.process.formation_task.hold import Hold
from src.algorithm.units.process.outbound.base import OutboundInputS, OutboundOutputS
from src.algorithm.units.process.outbound.leader_broadcast import LeaderBroadcast, OutboundInitS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute, LeaderRouteInitS

_LEADER_L1_DISTANCE_M = 200.0 # 配置为0，则关闭L1前瞻航迹插值，直接按航段起点/终点位置解算航迹。


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
        self._tra_plan.init(LeaderRouteInitS(cfg.route))
        self._pos_calc.init(RouteInterpInitS(lookAheadDistance=_LEADER_L1_DISTANCE_M))
        self._pos_track.init(_default_tracker_init(cfg.control_period_s))
        self._outbound.init(OutboundInitS(cfg.selfInit.id, cfg.commInit.netWork))

        # 预先绑定各单元输入/输出端口到共享黑板字段，step 时直接复用、零拷贝串联
        self._task_u = FormationTaskInputS(remote=self._remote, cmd=self.cxt.cmd)
        self._task_y = FormationTaskOutputS(cmd=self.cxt.cmd)
        # 航路规划读编队指令与本机状态，输出当前航段写回黑板的 wayLine
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        self._pos_calc_u = RouteInterpInputS(selfState=self.cxt.selfState, wayLine=self.cxt.wayLine)
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd)
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
        self._outbox.clear()

    def close(self) -> None:
        """释放 LeaderEntity 持有的资源。注意：关闭后不应继续调用运行接口。"""
        return None


def _default_tracker_init(control_period_s: float = DEFAULT_CONTROL_PERIOD_S) -> PidComposeInitS:
    """生成长机默认位置跟踪器配置。注意：仅在外部未注入配置时使用。"""
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    # 三轴 PID 增益分别整定：纵向(前向)、横向(侧偏)、垂向(高度)，dt 使用上层注入的控制周期。
    gain_forward = CtrlInitS(kp=1.0, ki=0.0, kd=0.0, dt=control_period_s, outMax=6.0)
    gain_lateral = CtrlInitS(kp=0.02, ki=0.0, kd=0.12, dt=control_period_s, outMax=4.0)
    gain_vertical = CtrlInitS(kp=0.2, ki=0.0, kd=0.6, dt=control_period_s, outMax=6.0)
    return PidComposeInitS(0.5, gain_forward, gain_lateral, gain_vertical)
