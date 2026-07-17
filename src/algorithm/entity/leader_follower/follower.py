"""通用僚机实体：支持直接保持，也支持完成集结流程后跟随最终编队。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormStageE
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityManagerInitS
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound import FormationInboundInitS
from src.algorithm.units.process.outbound import (
    FormationOutboundInitS,
    OutboundMessageE,
)
from src.algorithm.entity.leader_follower import (
    FOLLOWER_PROFILE,
)


class FollowerEntity(EntityBase):
    """通用僚机实体：按配置直接保持或参与集结，最终维持编队槽位。"""

    PROFILE = FOLLOWER_PROFILE
    MISSING_REMOTE_STAGE = FormStageE.NONE

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 FollowerEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("FollowerEntity: route 至少需要两个航点")
        if not isinstance(cfg.rally_cfg, RallyTaskInitS):
            raise ValueError("FollowerEntity: rally_cfg must be RallyTaskInitS")

        # 固定流程类和端口由基类定义；空策略流程仍使用各自业务参数正常初始化。
        # 同一个实体类同时服务直接 HOLD 和集结后 HOLD，两者只在初始化配置上有差异。
        # Profile 声明完整算法能力，rally_enabled 决定本次任务启用哪套集结专用参数。
        # 运行期间流程对象和产品实例均不重新装配。
        self._initialize_process_chain(
            {
                "inbound": FormationInboundInitS(cfg.selfInit.id),
                "formation_task": EntityManagerInitS(cfg, self.profile),
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
