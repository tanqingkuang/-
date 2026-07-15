"""集结场景长机实体：JOINING 阶段平等飞行/盘旋，完成后切换到任务航线并驱动 LOOSE→COMPRESS→HOLD。"""

from __future__ import annotations

import math
from dataclasses import replace

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    FormationAnalysisS,
    FormStageE,
    PosTrackDiagS,
    MotionProfS,
    RemoteCmdS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc import PosCalcInputS, PosCalcManager, PosCalcOutputS
from src.algorithm.units.algo.pos_track import PosTrackInputS, PosTrackManager, PosTrackOutputS
from src.algorithm.units.process.formation_task.rally import Rally, RallyTaskInitS, RallyTaskInputS, RallyTaskOutputS
from src.algorithm.units.process.inbound.follower_status import FollowerStatus, FollowerStatusInitS, FollowerStatusInputS, FollowerStatusOutputS
from src.algorithm.units.process.outbound.base import OutboundInitS, OutboundOutputS
from src.algorithm.units.process.outbound.rally_leader_broadcast import RallyLeaderBroadcast, RallyLeaderBroadcastInputS
from src.algorithm.units.process.tra_plan import TraPlanInputS, TraPlanManager, TraPlanOutputS
from src.algorithm.entity.leader_follower_rally import fill_output
from src.algorithm.units.algo.pos_calc.rally_join_pos import loiter_speed_bounds


class RallyLeaderEntity(EntityBase):
    """集结长机实体：JOINING 阶段平等参与汇合，完成后沿任务航线飞行并编排队形压缩。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyLeaderEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("RallyLeaderEntity: route 至少需要两个航点")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("RallyLeaderEntity: rally_cfg must be RallyTaskInitS")

        self.cxt = FormContextS()
        self._remote = RemoteCmdS()
        self._outbox: list = []
        self._effective_cmd = MotionProfS()
        self._rally_completed = False
        self._expected_follower_ids: list[str] = list(rally_cfg.expectedFollowerIds)
        self._tight_radius_m: float = rally_cfg.tightRadius_m
        self._stale_timeout_s: float = rally_cfg.staleTimeout_s
        loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)

        # 单元实例
        self._inbound = FollowerStatus()
        self._task = Rally()
        self._tra_plan = TraPlanManager()
        self._pos_track = PosTrackManager()
        self._outbound = RallyLeaderBroadcast()

        # 单元初始化
        self._inbound.init(FollowerStatusInitS())
        self._task.init(replace(
            rally_cfg,
            leaderId=cfg.selfInit.id,
            loiter_speed_min_mps=loiter_min,
            loiter_speed_max_mps=loiter_max,
        ))
        self._tra_plan.init(cfg)
        self._pos_track.init(cfg)
        self._outbound.init(OutboundInitS(cfg.selfInit.id, cfg.commInit.netWork))

        # 绑定端口
        self._inbound_u = FollowerStatusInputS(inbox=self._get_inbox_ref(), now_s=0.0)
        self._inbound_y = FollowerStatusOutputS(followerStates=self.cxt.followerStates)
        self._task_u = RallyTaskInputS(
            remote=self._remote,
            cmd=self.cxt.cmd,
            followerStates=self.cxt.followerStates,
            now_s=0.0,
            posCalcStatus=self.cxt.posCalcStatus,
        )
        self._task_y = RallyTaskOutputS(cmd=self.cxt.cmd)
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine, nextWayLine=self.cxt.nextWayLine)
        self._pos_calc_u = PosCalcInputS(
            selfState=self.cxt.selfState,
            cmd=self.cxt.cmd,
            wayLine=self.cxt.wayLine,
            nextWayLine=self.cxt.nextWayLine,
            clock=self.cxt.clock,
            rallyPlan=self.cxt.rallyPlan,
        )
        self._pos_calc_y = PosCalcOutputS(
            selfCmd=self.cxt.selfCmd,
            status=self.cxt.posCalcStatus,
            posTrackCommand=self.cxt.posTrackCommand,
        )
        self._pos_calc = PosCalcManager()
        self._pos_calc.init(cfg)
        self._pos_track_u = PosTrackInputS(
            command=self.cxt.posTrackCommand,
            selfCmd=self.cxt.selfCmd,
            selfState=self.cxt.selfState,
        )
        self._pos_track_diag = PosTrackDiagS()
        self._pos_track_y = PosTrackOutputS(
            accCmd=self.cxt.selfAccCmd,
            diag=self._pos_track_diag,
            effectiveCmd=self._effective_cmd,
        )
        self._outbound_u = RallyLeaderBroadcastInputS(
            cmd=self.cxt.cmd,
            selfState=self.cxt.selfState,
            leaderCmd=self._effective_cmd,
        )
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

        self._inbox: list = []
        self._inbound_u.inbox = self._inbox

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 RallyLeaderEntity 一个处理周期。"""
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = u.remote.stage
        self.cxt.clock.now_s = u.now_s
        previous_stage = self.cxt.cmd.stage

        self._inbox.clear()
        self._inbox.extend(u.inbox)
        self._inbound_u.now_s = u.now_s
        self._task_u.now_s = u.now_s

        # 通信槽位不区分 STANDBY/RALLY/HOLD，阶段推进只交给任务槽位判断。
        self._inbound.step(self._inbound_u, self._inbound_y)

        self._task.step(self._task_u, self._task_y)

        # 同步 t_ref 到上下文（供广播）
        self.cxt.rally_t_ref = self._task_y.t_ref
        self.cxt.rally_t_ref_valid = self._task_y.t_ref_valid
        self.cxt.rally_loop_counts.clear()
        self.cxt.rally_loop_counts.update(self._task_y.loopCounts)
        self._outbound_u.t_ref = self.cxt.rally_t_ref
        self._outbound_u.t_ref_valid = self.cxt.rally_t_ref_valid
        self._outbound_u.loop_counts = dict(self._task_y.loopCounts)

        stage = self.cxt.cmd.stage
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)

        if stage == FormStageE.NONE:
            # NONE 是停控空策略，仍保持原有早退语义，和 STANDBY 待命盘旋无关。
            if previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._rally_completed = False
                self.cxt.followerStates.clear()
            self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
            self._pos_track.step(self._pos_track_u, self._pos_track_y)
            self._outbound.step(self._outbound_u, self._outbound_y)
            fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
            return

        self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
        self._pos_track.step(self._pos_track_u, self._pos_track_y)
        if stage == FormStageE.STANDBY:
            # effective_cmd 跟随本地盘旋目标，保证输出诊断和跟踪目标一致。
            copy_motion(self.cxt.selfCmd, self._effective_cmd)

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
        copy_motion(MotionProfS(), self._effective_cmd)
        self._remote.stage = RemoteCmdS().stage
        self._rally_completed = False
        self._task.reset()
        self._tra_plan.reset()
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
