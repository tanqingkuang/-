"""集结场景僚机实体：集结期间平等飞行 → 盘旋等待 → 切出，之后跟随松散/压缩编队。"""

from __future__ import annotations

from dataclasses import replace

from src.algorithm.context.context import reset_context
from src.algorithm.context.leaf_types import (
    FormStageE,
    PosTrackDiagS,
    RemoteCmdS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityManagerInitS, EntityOutputS
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound import FormationInboundInitS
from src.algorithm.units.process.outbound import (
    FormationOutboundInitS,
    OutboundMessageE,
)
from src.algorithm.entity.leader_follower_rally import (
    RALLY_FOLLOWER_PROFILE,
    fill_output,
)


class RallyFollowerEntity(EntityBase):
    """集结僚机实体：JOINING 阶段平等飞行/盘旋，LOOSE/COMPRESS 阶段跟随松散槽位，HOLD 阶段维持编队。"""

    PROFILE = RALLY_FOLLOWER_PROFILE

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 RallyFollowerEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("RallyFollowerEntity: route 至少需要两个航点")
        if not isinstance(cfg.rally_cfg, RallyTaskInitS):
            raise ValueError("RallyFollowerEntity: rally_cfg must be RallyTaskInitS")

        # 固定流程类和端口由基类定义；空策略流程仍使用各自业务参数正常初始化。
        # 同一个实体类同时服务直接 HOLD 和集结后 HOLD，两者只在初始化配置上有差异。
        # Profile 声明完整算法能力，rally_enabled 决定本次任务启用哪套集结专用参数。
        # 运行期间流程对象和产品实例均不重新装配。
        self._initialize_process_chain(
            {
                "inbound": FormationInboundInitS(cfg.selfInit.id),
                "formation_task": replace(
                    cfg.rally_cfg,
                    leaderId=cfg.rally_leader_id,
                ),
                "tra_plan": EntityManagerInitS(cfg, self.profile),
                "pos_calc": EntityManagerInitS(
                    entity=cfg,
                    profile=self.profile,
                ),
                "pos_track": EntityManagerInitS(cfg, self.profile),
                "outbound": FormationOutboundInitS(
                    selfId=cfg.selfInit.id,
                    netWork=cfg.commInit.netWork,
                    leaderId=cfg.rally_leader_id,
                    messageType=(
                        # 普通 HOLD 保持旧协议静默；集结僚机才需要持续回报协调状态。
                        OutboundMessageE.FOLLOWER_STATUS
                        if cfg.rally_enabled
                        else OutboundMessageE.NOOP
                    ),
                ),
            },
        )

    def _prepare_input(self, u: EntityInputS) -> None:
        """写入僚机边界输入。注意：业务流程由 EntityBase.step 统一推进。"""
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        self.cxt.clock.now_s = u.now_s
        self._remote.stage = u.remote.stage if u.remote is not None else FormStageE.NONE
        self._inbox.clear()
        self._inbox.extend(u.inbox)

    def _finish_output(self, u: EntityInputS, y: EntityOutputS) -> None:
        """回填僚机边界输出。注意：不得在此重复推进流程。"""
        del u
        fill_output(self.cxt, self._pos_track_diag, self._outbox, y)

    def reset(self) -> None:
        """复位 RallyFollowerEntity 的动态状态。"""
        reset_context(self.cxt)
        self._remote.stage = RemoteCmdS().stage
        self._reset_processes()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._inbox.clear()
        self._outbox.clear()

    def close(self) -> None:
        """释放 RallyFollowerEntity 持有的资源。"""
        return None
