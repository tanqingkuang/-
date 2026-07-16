"""领航跟随集结实体包。注意：长机与僚机实体保持独立导出，避免影响既有保持实体。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    PosTrackDiagS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.types import (
    EntityOutputS,
    EntityProcessSpecS,
    EntityProcessTableS,
    EntityProfileE,
    EntityProfileS,
)
from src.algorithm.units.algo.pos_calc import PosCalcStrategyE
from src.algorithm.units.algo.pos_track import PosTrackStrategyE
from src.algorithm.units.process.tra_plan import TraPlanStrategyE

if TYPE_CHECKING:
    from src.algorithm.entity.base import EntityBase


RALLY_LEADER_PROFILE = EntityProfileS(
    identity=EntityProfileE.RALLY_LEADER,
    processes=EntityProcessTableS(
        tra_plan=EntityProcessSpecS(
            default_strategy=TraPlanStrategyE.LEADER_ROUTE,
            strategies=(TraPlanStrategyE.NOOP, TraPlanStrategyE.LEADER_ROUTE),
        ),
        pos_calc=EntityProcessSpecS(
            default_strategy=PosCalcStrategyE.ROUTE_INTERP,
            strategies=(PosCalcStrategyE.RALLY_JOIN,),
        ),
        pos_track=EntityProcessSpecS(
            strategies=(PosTrackStrategyE.NOOP, PosTrackStrategyE.PID_SPEED),
        ),
    ),
)
"""集结长机身份证。注意：所有长机实例共享此不可变策略配置。"""


RALLY_FOLLOWER_PROFILE = EntityProfileS(
    identity=EntityProfileE.RALLY_FOLLOWER,
    processes=EntityProcessTableS(
        tra_plan=EntityProcessSpecS(
            default_strategy=TraPlanStrategyE.NOOP,
            strategies=(TraPlanStrategyE.NOOP,),
        ),
        pos_calc=EntityProcessSpecS(
            default_strategy=PosCalcStrategyE.SLOT_GEOMETRY,
            strategies=(PosCalcStrategyE.RALLY_JOIN,),
        ),
        pos_track=EntityProcessSpecS(
            strategies=(
                PosTrackStrategyE.NOOP,
                PosTrackStrategyE.PID_SPEED,
                PosTrackStrategyE.PID_POSITION,
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
