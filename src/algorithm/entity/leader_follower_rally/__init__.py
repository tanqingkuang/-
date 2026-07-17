"""统一领航跟随实体包。注意：长机与僚机均支持集结后进入队形保持。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    FormStageE,
    PosTrackDiagS,
    RallyPhaseE,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.types import (
    EntityOutputS,
    EntityProfileE,
    EntityProfileS,
    EntityRouteChangeS,
    EntityStrategiesS,
)
from src.algorithm.units.algo.pos_calc import PosCalcStrategyE
from src.algorithm.units.algo.pos_track import PosTrackStrategyE
from src.algorithm.units.process.tra_plan import TraPlanStrategyE

if TYPE_CHECKING:
    from src.algorithm.entity.base import EntityBase


RALLY_STATE_SEQUENCE = (
    (FormStageE.NONE, RallyPhaseE.JOINING),
    (FormStageE.STANDBY, RallyPhaseE.JOINING),
    (FormStageE.RALLY, RallyPhaseE.JOINING),
    (FormStageE.RALLY, RallyPhaseE.CATCHUP),
    (FormStageE.RALLY, RallyPhaseE.LOOSE),
    (FormStageE.RALLY, RallyPhaseE.COMPRESS),
    (FormStageE.HOLD, RallyPhaseE.JOINING),
)
"""集结 Entity 的合法状态及变化点继承顺序。"""
RALLY_STATE_SET = frozenset(RALLY_STATE_SEQUENCE)
"""集结 Entity 合法状态集合。注意：供入站消息提交前校验。"""


RALLY_LEADER_PROFILE = EntityProfileS(
    identity=EntityProfileE.RALLY_LEADER,
    state_sequence=RALLY_STATE_SEQUENCE,
    route_changes=(
        EntityRouteChangeS(
            state=(FormStageE.NONE, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.NOOP,
                pos_track=PosTrackStrategyE.NOOP,
            ),
        ),
        EntityRouteChangeS(
            state=(FormStageE.STANDBY, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.RALLY_JOIN,
                pos_track=PosTrackStrategyE.PID_SPEED,
            ),
        ),
        EntityRouteChangeS(
            state=(FormStageE.RALLY, RallyPhaseE.CATCHUP),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.LEADER_ROUTE,
                pos_calc=PosCalcStrategyE.ROUTE_INTERP,
                pos_track=PosTrackStrategyE.PID_SPEED,
            ),
        ),
    ),
)
"""集结长机身份证。注意：所有长机实例共享此不可变策略配置。"""


RALLY_FOLLOWER_PROFILE = EntityProfileS(
    identity=EntityProfileE.RALLY_FOLLOWER,
    state_sequence=RALLY_STATE_SEQUENCE,
    route_changes=(
        EntityRouteChangeS(
            state=(FormStageE.NONE, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.NOOP,
                pos_track=PosTrackStrategyE.NOOP,
            ),
        ),
        EntityRouteChangeS(
            state=(FormStageE.STANDBY, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.RALLY_JOIN,
                pos_track=PosTrackStrategyE.PID_SPEED,
            ),
        ),
        EntityRouteChangeS(
            state=(FormStageE.RALLY, RallyPhaseE.CATCHUP),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.SLOT_GEOMETRY,
                pos_track=PosTrackStrategyE.PID_POSITION,
            ),
        ),
    ),
)
"""集结僚机身份证。注意：所有僚机实例共享此不可变策略配置。"""


def create_rally_entity(identity: EntityProfileE) -> EntityBase:
    """按实体身份创建独立实例。注意：Profile 可共享，Entity 运行状态不可共享。"""
    if identity == EntityProfileE.RALLY_LEADER:
        from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity

        return RallyLeaderEntity()
    if identity == EntityProfileE.RALLY_FOLLOWER:
        from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity

        return RallyFollowerEntity()
    raise ValueError(f"不支持的集结实体身份: {identity!r}")


def fill_output(cxt: FormContextS, diag: PosTrackDiagS, outbox: list, y: EntityOutputS) -> None:
    """将 Context 中的计算结果回填到实体输出边界。"""
    if y.selfAccCmd is None:
        y.selfAccCmd = cxt.selfAccCmd
    else:
        y.selfAccCmd.accEast = cxt.selfAccCmd.accEast
        y.selfAccCmd.accNorth = cxt.selfAccCmd.accNorth
        y.selfAccCmd.accUp = cxt.selfAccCmd.accUp
    if y.selfCmd is None:
        y.selfCmd = cxt.selfCmd
    else:
        copy_motion(cxt.selfCmd, y.selfCmd)
    if y.controlDiag is None:
        y.controlDiag = diag
    else:
        copy_pos_track_diag(diag, y.controlDiag)
    y.outbox.clear()
    y.outbox.extend(outbox)
