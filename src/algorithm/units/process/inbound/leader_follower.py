"""Parse leader broadcast messages for follower entities."""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormPatE, FormStageE
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS


LEADER_BROADCAST_TOPIC = "formation.leader"


class LeaderFollower(InboundBase):
    def init(self, cfg: InboundInitS) -> None:
        del cfg

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        if y.leaderState is None or y.cmd is None:
            raise ValueError("LeaderFollower output ports must be bound")
        for msg in u.inbox:
            if msg.topic != LEADER_BROADCAST_TOPIC or not isinstance(msg.payload, dict):
                continue
            payload = msg.payload
            state = payload.get("leader_state")
            cmd = payload.get("cmd")
            if not isinstance(state, dict) or not isinstance(cmd, dict):
                continue
            _write_motion_from_payload(state, y.leaderState)
            y.cmd.stage = FormStageE(int(cmd.get("stage", FormStageE.NONE)))
            y.cmd.pattern = FormPatE(int(cmd.get("pattern", FormPatE.NONE)))
            y.cmd.step = int(cmd.get("step", 0))

    def reset(self) -> None:
        return None


def _write_motion_from_payload(payload: dict[str, object], dst: object) -> None:
    pos = payload.get("pos")
    vd = payload.get("vd")
    if not isinstance(pos, dict) or not isinstance(vd, dict):
        return
    dst.pos.east = float(pos.get("east", 0.0))
    dst.pos.north = float(pos.get("north", 0.0))
    dst.pos.h = float(pos.get("h", 0.0))
    dst.vd.vEast = float(vd.get("vEast", 0.0))
    dst.vd.vNorth = float(vd.get("vNorth", 0.0))
    dst.vd.vUp = float(vd.get("vUp", 0.0))
    dst.vd.vTheta = float(vd.get("vTheta", 0.0))
    dst.vd.vPsi = float(vd.get("vPsi", 0.0))
    dst.vd.vd = float(vd.get("vd", 0.0))
