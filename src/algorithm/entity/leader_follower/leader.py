"""通用长机实体：支持直接保持，也支持集结完成后沿任务航线保持编队。"""

from __future__ import annotations

from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityManagerInitS
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.algorithm.units.process.inbound import FormationInboundInitS
from src.algorithm.units.process.outbound import (
    FormationOutboundInitS,
    OutboundMessageE,
)
from src.algorithm.entity.leader_follower import (
    LEADER_PROFILE,
)


class LeaderEntity(EntityBase):
    """通用长机实体：按配置直接保持或参与集结，随后沿任务航线飞行。"""

    PROFILE = LEADER_PROFILE

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 LeaderEntity。"""
        if len(cfg.route) < 2:
            raise ValueError("LeaderEntity: route 至少需要两个航点")
        rally_cfg = cfg.rally_cfg
        if not isinstance(rally_cfg, RallyTaskInitS):
            raise ValueError("LeaderEntity: rally_cfg must be RallyTaskInitS")

        # 固定流程类和端口由基类定义；策略流程读取本实例绑定的 Profile。
        # 直接 HOLD 与集结任务共享同一长机流程链，差异只来自运行期任务命令。
        # 长机始终保留航线和广播能力，不因当前阶段更换流程对象。
        # Rally 任务同时覆盖直接保持和集结完成后的保持编排。
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
                    messageType=OutboundMessageE.LEADER_BROADCAST,
                ),
            },
        )
