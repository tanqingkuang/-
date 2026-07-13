"""集结场景僚机实体：集结期间平等飞行 → 盘旋等待 → 切出，之后跟随松散/压缩编队。"""

from __future__ import annotations

import math

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    FormStageE,
    MotionProfS,
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
    RALLY_STATE_STANDBY,
    RallyJoinPos,
    RallyJoinPosInitS,
    RallyJoinPosInputS,
)
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
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
    route_heading_rad,
    resolve_formation_slot,
)


class RallyFollowerEntity(EntityBase):
    """集结僚机实体：JOINING 阶段平等飞行/盘旋，LOOSE/COMPRESS 阶段跟随松散槽位，HOLD 阶段维持编队。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyFollowerEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("RallyFollowerEntity: route 至少需要两个航点")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("RallyFollowerEntity: rally_cfg must be RallyTaskInitS")

        # 从统一航线第一航段计算集结航向，再按目标队形槽位算本机松散目标点。
        _A = cfg.route[0].pos
        _heading = route_heading_rad(cfg.route)
        _slot = resolve_formation_slot(cfg.commInit, rally_cfg.targetPattern, cfg.selfInit.id)
        if _slot is None:
            raise ValueError(
                f"RallyFollowerEntity: 节点 {cfg.selfInit.id!r} 在目标队形 {rally_cfg.targetPattern!r} "
                "的槽位表中未找到对应条目（目标队形不在 formPat 中，或 formPos 缺少该队形/该节点）"
            )
        _rally_target = rally_loose_target(_A, _heading, rally_cfg.looseScale, _slot)
        if cfg.rally_layer_altitude_m is not None:
            _rally_target.h = cfg.rally_layer_altitude_m

        self.cxt = FormContextS()
        self._inbox: list = []
        self._outbox: list = []
        self._leader_cmd = MotionProfS()

        loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)

        slow_radius_m = max(rally_cfg.arrival_radius_m * 3.0, 60.0)

        # 单元实例
        self._inbound = RallyLeaderFollower()
        self._tra_plan = Noop()
        self._rally_join = RallyJoinPos()
        self._pos_calc_slot = SlotGeometry()
        self._pos_track = PidCompose()
        self._outbound = FollowerBroadcast()
        self._rally_layer_altitude_m = cfg.rally_layer_altitude_m

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
            standby_altitude_m=cfg.rally_layer_altitude_m,
        ))
        self._pos_calc_slot.init(SlotGeometryInitS(cfg.selfInit.id, cfg.commInit.formPat, cfg.commInit.formPos))
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
            leaderCmd=self._leader_cmd,
            cmd=self.cxt.cmd,
            slotScale=self.cxt.slotScale,
        )
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        self._rally_join_u = RallyJoinPosInputS(selfState=self.cxt.selfState)
        self._slot_u = SlotGeometryInputS(
            selfState=self.cxt.selfState,
            leaderState=self.cxt.leaderState,
            leaderCmd=self._leader_cmd,
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
        remote_stage = u.remote.stage if u.remote is not None else None
        previous_stage = self.cxt.cmd.stage
        self._inbox.clear()
        self._inbox.extend(u.inbox)
        standby_requested = remote_stage == FormStageE.STANDBY

        # 通信槽位先正常解析长机广播，待命只在后续阶段选择覆盖本机位置解算。
        self._inbound.step(self._inbound_u, self._inbound_y)
        self.cxt.rally_t_ref = self._inbound_y.t_ref
        self.cxt.rally_t_ref_valid = self._inbound_y.t_ref_valid
        if standby_requested:
            # 本地远控阶段只决定本机 pos_calc 策略，不阻断长机广播解析。
            self.cxt.cmd.stage = FormStageE.STANDBY
            self.cxt.cmd.step = RallyPhaseE.JOINING
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)

        stage = self.cxt.cmd.stage

        if stage == FormStageE.NONE:
            # NONE 是停控空策略，保留当前位置零速输出，和 STANDBY 本地盘旋分开处理。
            if previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._rally_join.reset()
            copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)
            zero_velocity(self.cxt.selfCmd.v)
            zero_acceleration(self.cxt.selfAccCmd)
            self._update_outbound()
            self._outbound.step(self._outbound_u, self._outbound_y)
            fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
            return

        if stage == FormStageE.STANDBY or (stage == FormStageE.RALLY and self.cxt.cmd.step == RallyPhaseE.JOINING):
            # STANDBY/JOINING 都属于 RallyJoinPos 位置解算策略，只由 standby 输入切换内部状态。
            # 待命没有长机 T_ref，显式压成无效，切到 RALLY 后再恢复广播值。
            self._rally_join_u.standby = stage == FormStageE.STANDBY
            self._rally_join_u.t_ref = 0.0 if stage == FormStageE.STANDBY else self.cxt.rally_t_ref
            self._rally_join_u.t_ref_valid = False if stage == FormStageE.STANDBY else self.cxt.rally_t_ref_valid
            self._rally_join_u.t_now = u.now_s
            self._rally_join.step(self._rally_join_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)
        else:
            # RALLY step>=1（CATCHUP/LOOSE/COMPRESS）或 HOLD：三维槽位跟随
            # CATCHUP 与 LOOSE/COMPRESS 用同一套算法——slotScale 处于松散值时即是 CATCHUP/LOOSE，
            # 二者的区别只在 Rally 任务的阶段门控上，位置解算本身不需要区分。
            self._pos_calc_slot.step(self._slot_u, self._pos_calc_y)
            # CATCHUP 尚未形成松散队形，继续错高；LOOSE 后再回到正常槽位高度。
            if (
                stage == FormStageE.RALLY
                and self.cxt.cmd.step == RallyPhaseE.CATCHUP
                and self._rally_layer_altitude_m is not None
            ):
                self.cxt.selfCmd.pos.h = self._rally_layer_altitude_m
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

    def _update_standby_outbound(self) -> None:
        """将本地待命盘旋状态同步到僚机回报端口。"""
        self._outbound_u.rally_state = RALLY_STATE_STANDBY
        self._outbound_u.eta_s = 0.0
        self._outbound_u.reached_slot_once = False
        self._outbound_u.selfArrived = 0
