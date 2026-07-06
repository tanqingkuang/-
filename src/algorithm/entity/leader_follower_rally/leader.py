"""集结场景长机实体：JOINING 阶段平等飞行/盘旋，完成后切换到任务航线并驱动 LOOSE→COMPRESS→HOLD。"""

from __future__ import annotations

import math

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    FormationAnalysisS,
    FormStageE,
    PosTrackDiagS,
    RallyPhaseE,
    RemoteCmdS,
    copy_motion,
    copy_pos_track_diag,
    zero_acceleration,
    zero_velocity,
    copy_position,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.leader import _default_tracker_init, _follower_tracker_init, waypoint_inputs_to_waylines
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RallyJoinPos,
    RallyJoinPosInitS,
    RallyJoinPosInputS,
)
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS, RouteInterpInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.algorithm.units.process.formation_task.base import FormationTaskOutputS
from src.algorithm.units.process.formation_task.rally import Rally, RallyTaskInitS, RallyTaskInputS, RallyTaskOutputS
from src.algorithm.units.process.inbound.follower_status import FollowerStatus, FollowerStatusInitS, FollowerStatusInputS, FollowerStatusOutputS
from src.algorithm.units.process.outbound.base import OutboundInitS, OutboundOutputS
from src.algorithm.units.process.outbound.rally_leader_broadcast import RallyLeaderBroadcast, RallyLeaderBroadcastInputS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute, LeaderRouteInitS
from src.algorithm.entity.leader_follower_rally import fill_output, loiter_speed_bounds, rally_route_heading_rad

_LEADER_L1_DISTANCE_M = 0.0  # 关闭L1前瞻，直接按航段投影解算目标航迹。大侧偏限角保护已由横侧向变限幅(1.2)接管。
_LEADER_FF_LEAD_TIME_S = 0.5


class RallyLeaderEntity(EntityBase):
    """集结长机实体：JOINING 阶段平等参与汇合，完成后沿任务航线飞行并编排队形压缩。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyLeaderEntity。"""
        if not cfg.rally_route or len(cfg.rally_route) < 2:
            raise ValueError("RallyLeaderEntity: rally_route 至少需要两个航点")
        if not cfg.route:
            raise ValueError("RallyLeaderEntity: route (mission_route) is required")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("RallyLeaderEntity: rally_cfg must be RallyTaskInitS")

        # A = 集结区起点（掌机松散目标）；heading 取第一航段方向，支持多航点集结航线
        rally_start = cfg.rally_route[0].pos
        _heading = rally_route_heading_rad(cfg.rally_route)

        self.cxt = FormContextS()
        self._remote = RemoteCmdS()
        self._outbox: list = []
        self._rally_completed = False
        self._expected_follower_ids: list[str] = list(rally_cfg.expectedFollowerIds)
        self._tight_radius_m: float = rally_cfg.tightRadius_m
        self._stale_timeout_s: float = rally_cfg.staleTimeout_s

        loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)

        slow_radius_m = max(rally_cfg.arrival_radius_m * 3.0, 60.0)

        # 单元实例
        self._inbound = FollowerStatus()
        self._task = Rally()
        self._rally_join = RallyJoinPos()
        self._tra_plan_mission = LeaderRoute()
        self._pos_calc = RouteInterp()
        self._pos_track = PidCompose()
        self._outbound = RallyLeaderBroadcast()

        # 单元初始化
        self._inbound.init(FollowerStatusInitS())
        self._task.init(rally_cfg)
        v_up_min = cfg.velCmdLimit.verticalMin if math.isfinite(cfg.velCmdLimit.verticalMin) else -3.0
        v_up_max = cfg.velCmdLimit.verticalMax if math.isfinite(cfg.velCmdLimit.verticalMax) else 3.0
        self._rally_join.init(RallyJoinPosInitS(
            loose_slot=rally_start,
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
        mission_lines = waypoint_inputs_to_waylines(cfg.route)
        self._tra_plan_mission.init(LeaderRouteInitS(mission_lines))
        self._pos_calc.init(RouteInterpInitS(lookAheadDistance=_LEADER_L1_DISTANCE_M, leadTimeS=_LEADER_FF_LEAD_TIME_S))
        self._pos_track.init(_default_tracker_init(cfg.control_period_s, cfg.velCmdLimit))
        self._outbound.init(OutboundInitS(cfg.selfInit.id, cfg.commInit.netWork))

        # 绑定端口
        self._inbound_u = FollowerStatusInputS(inbox=self._get_inbox_ref(), now_s=0.0)
        self._inbound_y = FollowerStatusOutputS(followerStates=self.cxt.followerStates)
        self._task_u = RallyTaskInputS(remote=self._remote, cmd=self.cxt.cmd, followerStates=self.cxt.followerStates, now_s=0.0)
        self._task_y = RallyTaskOutputS(cmd=self.cxt.cmd, slotScale=self.cxt.slotScale)
        self._rally_join_u = RallyJoinPosInputS(selfState=self.cxt.selfState)
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine, nextWayLine=self.cxt.nextWayLine)
        self._pos_calc_u = RouteInterpInputS(selfState=self.cxt.selfState, wayLine=self.cxt.wayLine, nextWayLine=self.cxt.nextWayLine)
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_diag = PosTrackDiagS()
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd, diag=self._pos_track_diag)
        self._outbound_u = RallyLeaderBroadcastInputS(cmd=self.cxt.cmd, selfState=self.cxt.selfState, slotScale=self.cxt.slotScale)
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

        self._inbox: list = []
        self._inbound_u.inbox = self._inbox

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 RallyLeaderEntity 一个处理周期。"""
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = u.remote.stage
        previous_stage = self.cxt.cmd.stage

        self._inbox.clear()
        self._inbox.extend(u.inbox)
        self._inbound_u.now_s = u.now_s
        self._task_u.now_s = u.now_s

        # 入站解析（从僚机广播更新 followerStates）
        self._inbound.step(self._inbound_u, self._inbound_y)

        # 将长机自身 RallyJoinPos 状态注入 Rally 任务
        self._task_u.leader_eta_s = self._rally_join.eta_s
        self._task_u.leader_join_exited = (self._rally_join.state == RALLY_STATE_EXITED)
        self._task_u.leader_join_flying = (self._rally_join.state == RALLY_STATE_FLYING)
        self._task_u.leader_join_reached_slot_once = self._rally_join.reached_slot_once
        self._task.step(self._task_u, self._task_y)

        # 同步 t_ref 到上下文（供广播）
        self.cxt.rally_t_ref = self._task_y.t_ref
        self.cxt.rally_t_ref_valid = self._task_y.t_ref_valid
        self._outbound_u.t_ref = self.cxt.rally_t_ref
        self._outbound_u.t_ref_valid = self.cxt.rally_t_ref_valid

        stage = self.cxt.cmd.stage
        step = self.cxt.cmd.step

        if stage == FormStageE.NONE:
            if previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._rally_join.reset()
                self._rally_completed = False
                self.cxt.followerStates.clear()
            copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)
            zero_velocity(self.cxt.selfCmd.v)
            zero_acceleration(self.cxt.selfAccCmd)
            self._outbound.step(self._outbound_u, self._outbound_y)
            fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
            return

        if stage == FormStageE.RALLY and step == RallyPhaseE.JOINING:
            # JOINING 阶段：长机平等参与，也飞向自己的松散点（队形中心）
            self._rally_join_u.t_ref = self.cxt.rally_t_ref
            self._rally_join_u.t_ref_valid = self.cxt.rally_t_ref_valid
            self._rally_join_u.t_now = u.now_s
            self._rally_join.step(self._rally_join_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)
        else:
            # RALLY step>=1（LOOSE/COMPRESS）或 HOLD：长机沿任务航线飞行
            self._tra_plan_mission.step(self._tra_plan_u, self._tra_plan_y)
            self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)

        self._outbound.step(self._outbound_u, self._outbound_y)

        # 集结完成判断（仅首帧输出 FormationAnalysisS）
        if self._task_y.rallyCompleted and not self._rally_completed:
            self._rally_completed = True
            y.formationAnalysis = _compute_formation_analysis(
                self.cxt.followerStates,
                self._expected_follower_ids,
                self._tight_radius_m,
                u.now_s,
                self._stale_timeout_s,
            )
        else:
            y.formationAnalysis = None

        fill_output(self.cxt, self._pos_track_diag, self._outbox, y)

    def reset(self) -> None:
        """复位 RallyLeaderEntity 的动态状态。"""
        reset_context(self.cxt)
        self._remote.stage = RemoteCmdS().stage
        self._rally_completed = False
        self._task.reset()
        self._rally_join.reset()
        self._tra_plan_mission.reset()
        self._pos_calc.reset()
        self._pos_track.reset()
        self._outbound.reset()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._outbox.clear()
        self._inbox.clear()

    def close(self) -> None:
        """释放 RallyLeaderEntity 持有的资源。"""
        return None

    def _get_inbox_ref(self) -> list:
        """返回内部 inbox 列表引用，供端口绑定使用。"""
        if not hasattr(self, "_inbox"):
            self._inbox = []
        return self._inbox


def _compute_formation_analysis(
    follower_states: list,
    expected_ids: list[str],
    tight_radius_m: float,
    now_s: float,
    stale_timeout_s: float,
) -> FormationAnalysisS:
    """计算编队质量分析快照。"""
    state_map = {s.id: s for s in follower_states}
    valid_states = [
        state_map[fid]
        for fid in expected_ids
        if fid in state_map
        and state_map[fid].valid
        and (now_s - state_map[fid].lastUpdate_s) <= stale_timeout_s
    ]
    total = len(expected_ids)
    if not valid_states:
        return FormationAnalysisS(
            posErrMax_m=float("nan"),
            posErrRms_m=float("nan"),
            inPositionCount=0,
            totalCount=total,
        )
    errs = [s.posErr_m for s in valid_states]
    pos_err_max = max(errs)
    pos_err_rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    in_position = sum(1 for e in errs if e < tight_radius_m)
    return FormationAnalysisS(
        posErrMax_m=pos_err_max,
        posErrRms_m=pos_err_rms,
        inPositionCount=in_position,
        totalCount=total,
    )


