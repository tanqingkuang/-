"""集结场景僚机实体：集结期间平等飞行 → 盘旋等待 → 切出，之后跟随松散/压缩编队。"""

from __future__ import annotations

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    FormStageE,
    MotionProfS,
    PosTrackDiagS,
    RallyPhaseE,
    copy_motion,
    copy_pos_track_diag,
    zero_acceleration,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.leader import _default_tracker_init, _follower_tracker_init
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc import PosCalcInputS, PosCalcManager, PosCalcOutputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound.base import InboundInputS
from src.algorithm.units.process.inbound.rally_leader_follower import RallyLeaderFollower, RallyLeaderFollowerOutputS
from src.algorithm.units.process.outbound.base import OutboundOutputS
from src.algorithm.units.process.outbound.follower_broadcast import FollowerBroadcast, FollowerBroadcastInitS, FollowerBroadcastInputS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.noop import Noop
from src.algorithm.entity.leader_follower_rally import fill_output


class RallyFollowerEntity(EntityBase):
    """集结僚机实体：JOINING 阶段平等飞行/盘旋，LOOSE/COMPRESS 阶段跟随松散槽位，HOLD 阶段维持编队。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyFollowerEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("RallyFollowerEntity: route 至少需要两个航点")
        if not isinstance(cfg.rally_cfg, RallyTaskInitS):
            raise ValueError("RallyFollowerEntity: rally_cfg must be RallyTaskInitS")

        self.cxt = FormContextS()
        self._inbox: list = []
        self._outbox: list = []
        self._leader_cmd = MotionProfS()
        self._self_id = cfg.selfInit.id

        # 单元实例
        self._inbound = RallyLeaderFollower()
        self._tra_plan = Noop()
        self._pos_track_joining = PidCompose()
        self._pos_track_formation = PidCompose()
        self._pos_track = self._pos_track_joining
        self._outbound = FollowerBroadcast()

        # 单元初始化
        self._inbound.init(None)
        self._tra_plan.init(None)
        # JOINING 的前向通道只跟踪时间协调速度；形成队形后恢复僚机前向位置闭环。
        self._pos_track_joining.init(_default_tracker_init(cfg.control_period_s, cfg.velCmdLimit))
        self._pos_track_formation.init(_follower_tracker_init(cfg.control_period_s, cfg.velCmdLimit))
        self._outbound.init(FollowerBroadcastInitS(
            selfId=cfg.selfInit.id,
            netWork=cfg.commInit.netWork,
            leaderId=cfg.rally_leader_id,
        ))

        # 绑定端口
        self._inbound_u = InboundInputS(inbox=self._inbox)
        self._inbound_y = RallyLeaderFollowerOutputS(
            leaderState=self.cxt.leaderState,
            leaderCmd=self._leader_cmd,
            cmd=self.cxt.cmd,
        )
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        self._pos_calc_u = PosCalcInputS(
            selfState=self.cxt.selfState,
            leaderState=self.cxt.leaderState,
            leaderCmd=self._leader_cmd,
            cmd=self.cxt.cmd,
            clock=self.cxt.clock,
            rallyPlan=self.cxt.rallyPlan,
        )
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd, status=self.cxt.posCalcStatus)
        self._pos_calc = PosCalcManager()
        self._pos_calc.init(cfg)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_diag = PosTrackDiagS()
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd, diag=self._pos_track_diag)
        self._outbound_u = FollowerBroadcastInputS(
            cmd=self.cxt.cmd,
            selfState=self.cxt.selfState,
            selfCmd=self.cxt.selfCmd,
        )
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 RallyFollowerEntity 一个处理周期。"""
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        self.cxt.clock.now_s = u.now_s
        remote_stage = u.remote.stage if u.remote is not None else None
        self._inbox.clear()
        self._inbox.extend(u.inbox)
        standby_requested = remote_stage == FormStageE.STANDBY

        # 通信槽位先正常解析长机广播，待命只在后续阶段选择覆盖本机位置解算。
        self._inbound.step(self._inbound_u, self._inbound_y)
        self.cxt.rally_t_ref = self._inbound_y.t_ref
        self.cxt.rally_loop_counts.clear()
        self.cxt.rally_loop_counts.update(self._inbound_y.loopCounts)
        has_assignment = self._self_id in self._inbound_y.loopCounts
        self.cxt.rally_t_ref_valid = self._inbound_y.t_ref_valid and has_assignment
        if standby_requested:
            # 本地远控阶段只决定本机 pos_calc 策略，不阻断长机广播解析。
            self.cxt.cmd.stage = FormStageE.STANDBY
            self.cxt.cmd.step = RallyPhaseE.JOINING
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)

        stage = self.cxt.cmd.stage
        joining_active = stage == FormStageE.STANDBY or (
            stage == FormStageE.RALLY and self.cxt.cmd.step == RallyPhaseE.JOINING
        )
        # 两个产品仅在 init 构造一次；运行期按任务指令切换引用，不重建控制器状态。
        self._pos_track = self._pos_track_joining if joining_active else self._pos_track_formation

        if stage == FormStageE.NONE:
            # NONE 是停控空策略，保留当前位置零速输出，和 STANDBY 本地盘旋分开处理。
            self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
            zero_acceleration(self.cxt.selfAccCmd)
            self._update_outbound()
            self._outbound.step(self._outbound_u, self._outbound_y)
            fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
            return

        self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
        self._pos_track.step(self._pos_track_u, self._pos_track_y)
        if stage == FormStageE.STANDBY:
            # STANDBY 仍走出站槽位，只把回报语义固定为本地待命。
            self._update_standby_outbound()
        else:
            self._update_outbound()
        self._outbound.step(self._outbound_u, self._outbound_y)
        fill_output(self.cxt, self._pos_track_diag, self._outbox, y)

    def reset(self) -> None:
        """复位 RallyFollowerEntity 的动态状态。"""
        reset_context(self.cxt)
        copy_motion(MotionProfS(), self._leader_cmd)
        self._inbound.reset()
        self._tra_plan.reset()
        self._pos_calc.reset()
        self._pos_track_joining.reset()
        self._pos_track_formation.reset()
        self._pos_track = self._pos_track_joining
        self._outbound.reset()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._inbox.clear()
        self._outbox.clear()

    def close(self) -> None:
        """释放 RallyFollowerEntity 持有的资源。"""
        return None

    def _update_outbound(self) -> None:
        """将 RallyJoinPos 状态同步到出站端口。"""
        status = self.cxt.posCalcStatus
        self._outbound_u.rally_state = status.rally_state
        self._outbound_u.planned_path_length_m = status.planned_path_length_m
        self._outbound_u.reached_slot_once = status.reached_slot_once
        self._outbound_u.selfArrived = 1 if status.join_exited else 0

    def _update_standby_outbound(self) -> None:
        """将本地待命盘旋状态同步到僚机回报端口。"""
        self._outbound_u.rally_state = self.cxt.posCalcStatus.rally_state
        self._outbound_u.planned_path_length_m = -1.0
        self._outbound_u.reached_slot_once = False
        self._outbound_u.selfArrived = 0
