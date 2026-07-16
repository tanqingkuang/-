"""集结场景长机实体：JOINING 阶段平等飞行/盘旋，完成后切换到任务航线并驱动 LOOSE→COMPRESS→HOLD。"""

from __future__ import annotations

import math
from dataclasses import replace

from src.algorithm.context.context import reset_context
from src.algorithm.context.leaf_types import (
    FormationAnalysisS,
    FormStageE,
    PosTrackDiagS,
    RemoteCmdS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityManagerInitS, EntityOutputS
from src.algorithm.units.algo.pos_calc import loiter_speed_bounds
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound import FormationInboundInitS
from src.algorithm.units.process.outbound import (
    FormationOutboundInitS,
    OutboundMessageE,
)
from src.algorithm.entity.leader_follower_rally import (
    RALLY_LEADER_PROFILE,
    fill_output,
)


class RallyLeaderEntity(EntityBase):
    """集结长机实体：JOINING 阶段平等参与汇合，完成后沿任务航线飞行并编排队形压缩。"""

    PROFILE = RALLY_LEADER_PROFILE

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyLeaderEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("RallyLeaderEntity: route 至少需要两个航点")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("RallyLeaderEntity: rally_cfg must be RallyTaskInitS")

        self._rally_completed = False
        self._expected_follower_ids: list[str] = list(rally_cfg.expectedFollowerIds)
        self._tight_radius_m: float = rally_cfg.tightRadius_m
        self._stale_timeout_s: float = rally_cfg.staleTimeout_s
        loiter_min, loiter_max = loiter_speed_bounds(cfg.velCmdLimit)

        # 固定流程类和端口由基类定义；策略流程读取本实例绑定的 Profile。
        self._initialize_process_chain(
            {
                "inbound": FormationInboundInitS(cfg.selfInit.id),
                "formation_task": replace(
                    rally_cfg,
                    leaderId=cfg.selfInit.id,
                    loiter_speed_min_mps=loiter_min,
                    loiter_speed_max_mps=loiter_max,
                ),
                "tra_plan": EntityManagerInitS(cfg, self.profile.processes.tra_plan),
                "pos_calc": EntityManagerInitS(cfg, self.profile.processes.pos_calc),
                "pos_track": EntityManagerInitS(cfg, self.profile.processes.pos_track),
                "outbound": FormationOutboundInitS(
                    selfId=cfg.selfInit.id,
                    netWork=cfg.commInit.netWork,
                    messageType=OutboundMessageE.LEADER_BROADCAST,
                ),
            },
        )

    def _prepare_input(self, u: EntityInputS) -> None:
        """写入长机边界输入。注意：业务流程由 EntityBase.step 统一推进。"""
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = u.remote.stage
        self.cxt.clock.now_s = u.now_s
        self._previous_stage = self.cxt.cmd.stage

        self._inbox.clear()
        self._inbox.extend(u.inbox)

    def _finish_output(self, u: EntityInputS, y: EntityOutputS) -> None:
        """完成长机边界输出和完成事件分析。注意：不得在此重复推进流程。"""
        stage = self.cxt.cmd.stage
        if stage == FormStageE.NONE and self._previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
            # NONE 只清除本轮完成锁存和僚机回报，具体停控由各 Manager 的 NOOP 产品完成。
            self._rally_completed = False
            self.cxt.followerStates.clear()

        # 集结完成判断（仅首帧输出 FormationAnalysisS）
        if self._task.rally_completed and not self._rally_completed:
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
        self._previous_stage = FormStageE.NONE
        self._rally_completed = False
        self._reset_processes()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._outbox.clear()
        self._inbox.clear()

    def close(self) -> None:
        """释放 RallyLeaderEntity 持有的资源。"""
        return None


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
