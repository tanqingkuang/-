"""Pack leader state into a multicast formation message."""

from __future__ import annotations

from src.algorithm.context.leaf_types import CommDirE, MotionProfS
from src.algorithm.units.process.inbound.leader_follower import LEADER_BROADCAST_TOPIC
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.common.envelope import MessageEnvelope


class LeaderBroadcast(OutboundBase):
    def __init__(self) -> None:
        self._self_id = ""
        self._net_work = []

    def init(self, cfg: OutboundInitS) -> None:
        self._self_id = cfg.selfId
        self._net_work = list(cfg.netWork)

    def step(self, u: OutboundInputS, y: OutboundOutputS) -> None:
        if u.cmd is None or u.selfState is None:
            raise ValueError("LeaderBroadcast input ports must be bound")
        targets = self._targets()
        y.outbox.clear()
        if not targets:
            return
        y.outbox.append(
            MessageEnvelope(
                topic=LEADER_BROADCAST_TOPIC,
                source=self._self_id,
                target=targets,
                timestamp=0.0,
                payload={
                    "leader_state": _motion_payload(u.selfState),
                    "cmd": {
                        "stage": int(u.cmd.stage),
                        "pattern": int(u.cmd.pattern),
                        "step": int(u.cmd.step),
                    },
                },
            )
        )

    def reset(self) -> None:
        return None

    def _targets(self) -> list[str]:
        targets: list[str] = []
        for link in self._net_work:
            if link.startId == self._self_id:
                targets.append(link.endId)
            elif link.endId == self._self_id and link.dir == CommDirE.DUPLEX:
                targets.append(link.startId)
        return list(dict.fromkeys(target for target in targets if target and target != self._self_id))


def _motion_payload(motion: MotionProfS) -> dict[str, dict[str, float]]:
    return {
        "pos": {
            "east": motion.pos.east,
            "north": motion.pos.north,
            "h": motion.pos.h,
        },
        "vd": {
            "vEast": motion.vd.vEast,
            "vNorth": motion.vd.vNorth,
            "vUp": motion.vd.vUp,
            "vTheta": motion.vd.vTheta,
            "vPsi": motion.vd.vPsi,
            "vd": motion.vd.vd,
        },
    }
