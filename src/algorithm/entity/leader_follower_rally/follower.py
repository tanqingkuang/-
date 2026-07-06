"""集结场景僚机实体：集结期间平等飞行 → 盘旋等待 → 切出，之后跟随松散/压缩编队。"""

from __future__ import annotations

import math

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    FormStageE,
    PosTrackDiagS,
    RallyPhaseE,
    copy_motion,
    copy_pos_track_diag,
    copy_position,
    zero_acceleration,
    zero_velocity,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.leader import _follower_tracker_init
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RallyJoinPos,
    RallyJoinPosInitS,
    RallyJoinPosInputS,
)
from src.algorithm.units.algo.pos_calc.scaled_slot_geometry import ScaledSlotGeometry, ScaledSlotInitS, ScaledSlotInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound.base import InboundInputS
from src.algorithm.units.process.inbound.rally_leader_follower import RallyLeaderFollower, RallyLeaderFollowerOutputS
from src.algorithm.units.process.outbound.base import OutboundOutputS
from src.algorithm.units.process.outbound.follower_broadcast import FollowerBroadcast, FollowerBroadcastInitS, FollowerBroadcastInputS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.noop import Noop
from src.algorithm.entity.leader_follower_rally import (
    fill_output,
    loiter_speed_bounds,
    rally_loose_target,
    rally_route_heading_rad,
    resolve_formation_slot,
)


class RallyFollowerEntity(EntityBase):
    """集结僚机实体：JOINING 阶段平等飞行/盘旋，LOOSE/COMPRESS 阶段跟随松散槽位，HOLD 阶段维持编队。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyFollowerEntity。"""
        if not cfg.rally_route or len(cfg.rally_route) < 2:
            raise ValueError("RallyFollowerEntity: rally_route 至少需要两个航点")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("RallyFollowerEntity: rally_cfg must be RallyTaskInitS")

        # 从集结航线第一航段计算任务航向，再按目标队形槽位算本机松散目标点
        _A = cfg.rally_route[0].pos
        _heading = rally_route_heading_rad(cfg.rally_route)
        _slot = resolve_formation_slot(cfg.commInit, rally_cfg.targetPattern, cfg.selfInit.id)
        if _slot is None:
            raise ValueError(
                f"RallyFollowerEntity: 节点 {cfg.selfInit.id!r} 在目标队形 {rally_cfg.targetPattern!r} "
                "的槽位表中未找到对应条目（目标队形不在 formPat 中，或 formPos 缺少该队形/该节点）"
            )
        _rally_target = rally_loose_target(_A, _heading, rally_cfg.looseScale, _slot)

        self.cxt = FormContextS()
        self._inbox: list = []
        self._outbox: list = []

        loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)

        slow_radius_m = max(rally_cfg.arrival_radius_m * 3.0, 60.0)

        # 单元实例
        self._inbound = RallyLeaderFollower()
        self._tra_plan = Noop()
        self._rally_join = RallyJoinPos()
        self._pos_calc_slot = ScaledSlotGeometry()
        self._pos_track = PidCompose()
        self._outbound = FollowerBroadcast()

        # 单元初始化
        self._inbound.init(None)
        self._tra_plan.init(None)
        v_up_min = cfg.velCmdLimit.verticalMin if math.isfinite(cfg.velCmdLimit.verticalMin) else -3.0
        v_up_max = cfg.velCmdLimit.verticalMax if math.isfinite(cfg.velCmdLimit.verticalMax) else 3.0
        self._rally_join.init(RallyJoinPosInitS(
            loose_slot=_rally_target,
            approach_speed_mps=cfg.rally_approach_speed_mps,
            slow_radius_m=slow_radius_m,
            arrival_radius_m=rally_cfg.arrival_radius_m,
            loiter_radius_m=rally_cfg.loiter_radius_m,
            loiter_speed_min_mps=loiter_min,
            loiter_speed_max_mps=loiter_max,
            mission_heading_rad=_heading,
            mission_speed_mps=cfg.rally_approach_speed_mps,
            v_up_min_mps=v_up_min,
            v_up_max_mps=v_up_max,
            control_period_s=cfg.control_period_s,
        ))
        self._pos_calc_slot.init(ScaledSlotInitS(
            selfId=cfg.selfInit.id,
            commInit=cfg.commInit,
        ))
        self._pos_track.init(_follower_tracker_init(cfg.control_period_s, cfg.velCmdLimit))
        self._outbound.init(FollowerBroadcastInitS(
            selfId=cfg.selfInit.id,
            netWork=cfg.commInit.netWork,
            leaderId=cfg.rally_leader_id,
        ))

        # 绑定端口
        self._inbound_u = InboundInputS(inbox=self._inbox)
        self._inbound_y = RallyLeaderFollowerOutputS(
            leaderState=self.cxt.leaderState,
            cmd=self.cxt.cmd,
            slotScale=self.cxt.slotScale,
        )
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        self._rally_join_u = RallyJoinPosInputS(selfState=self.cxt.selfState)
        self._slot_u = ScaledSlotInputS(
            selfState=self.cxt.selfState,
            leaderState=self.cxt.leaderState,
            cmd=self.cxt.cmd,
            slotScale=self.cxt.slotScale,
        )
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
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
        previous_stage = self.cxt.cmd.stage
        self._inbox.clear()
        self._inbox.extend(u.inbox)

        # 入站解析（更新 leaderState / cmd / slotScale / t_ref）
        self._inbound.step(self._inbound_u, self._inbound_y)
        self.cxt.rally_t_ref = self._inbound_y.t_ref
        self.cxt.rally_t_ref_valid = self._inbound_y.t_ref_valid
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)

        stage = self.cxt.cmd.stage

        if stage == FormStageE.NONE:
            if previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._rally_join.reset()
            copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)
            zero_velocity(self.cxt.selfCmd.v)
            zero_acceleration(self.cxt.selfAccCmd)
            self._update_outbound()
            self._outbound.step(self._outbound_u, self._outbound_y)
            fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
            return

        if stage == FormStageE.RALLY and self.cxt.cmd.step == RallyPhaseE.JOINING:
            # JOINING 阶段：平等飞行 / 盘旋 / 切出
            self._rally_join_u.t_ref = self.cxt.rally_t_ref
            self._rally_join_u.t_ref_valid = self.cxt.rally_t_ref_valid
            self._rally_join_u.t_now = u.now_s
            self._rally_join.step(self._rally_join_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)
        else:
            # RALLY step>=1（CATCHUP/LOOSE/COMPRESS）或 HOLD：三维槽位跟随
            # CATCHUP 与 LOOSE/COMPRESS 用同一套算法——slotScale 处于松散值时即是 CATCHUP/LOOSE，
            # 二者的区别只在 Rally 任务的阶段门控上，位置解算本身不需要区分。
            self._pos_calc_slot.step(self._slot_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)
        self._update_outbound()
        self._outbound.step(self._outbound_u, self._outbound_y)
        fill_output(self.cxt, self._pos_track_diag, self._outbox, y)

    def reset(self) -> None:
        """复位 RallyFollowerEntity 的动态状态。"""
        reset_context(self.cxt)
        self._inbound.reset()
        self._tra_plan.reset()
        self._rally_join.reset()
        self._pos_calc_slot.reset()
        self._pos_track.reset()
        self._outbound.reset()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._inbox.clear()
        self._outbox.clear()

    def close(self) -> None:
        """释放 RallyFollowerEntity 持有的资源。"""
        return None

    def _update_outbound(self) -> None:
        """将 RallyJoinPos 状态同步到出站端口。"""
        self._outbound_u.rally_state = self._rally_join.state
        self._outbound_u.eta_s = self._rally_join.eta_s
        self._outbound_u.reached_slot_once = self._rally_join.reached_slot_once
        self._outbound_u.selfArrived = 1 if self._rally_join.state == RALLY_STATE_EXITED else 0
