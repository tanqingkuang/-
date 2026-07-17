"""通用领航跟随实体包。注意：同一套长机与僚机实体同时支持直接保持和集结后保持。"""

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


# 状态序列是三个 Manager 共用的路由时间轴，顺序必须与 Rally 状态机一致。
# NONE 和 STANDBY 只使用 JOINING 占位，避免为停控阶段引入无业务意义的子阶段。
# RALLY/JOINING 对应切入集结圆，CATCHUP 和 LOOSE 共用任务飞行产品。
# 渐进压缩能力删除后，LOOSE 稳定收敛便直接进入 HOLD，不再登记中间阶段。
# HOLD 重新使用 JOINING 占位，使完成后的策略继承保持单一且可预测。
# 入站协议也复用这份集合校验组合，不能只改任务状态机而遗漏此处。
FORMATION_STATE_SEQUENCE = (
    (FormStageE.NONE, RallyPhaseE.JOINING),
    (FormStageE.STANDBY, RallyPhaseE.JOINING),
    (FormStageE.RALLY, RallyPhaseE.JOINING),
    (FormStageE.RALLY, RallyPhaseE.CATCHUP),
    (FormStageE.RALLY, RallyPhaseE.LOOSE),
    (FormStageE.HOLD, RallyPhaseE.JOINING),
)
"""领航跟随 Entity 的合法状态及变化点继承顺序。"""
FORMATION_STATE_SET = frozenset(FORMATION_STATE_SEQUENCE)
"""领航跟随 Entity 合法状态集合。注意：供入站消息提交前校验。"""


LEADER_PROFILE = EntityProfileS(
    identity=EntityProfileE.LEADER,
    state_sequence=FORMATION_STATE_SEQUENCE,
    route_changes=(
        # 冷启动停控：三个可切换流程全部选择空产品。
        EntityRouteChangeS(
            state=(FormStageE.NONE, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.NOOP,
                pos_track=PosTrackStrategyE.NOOP,
            ),
        ),
        # 本地待命与集结切入：位置解算切到有状态的 RallyJoinPos。
        # TraPlan 仍停控，避免待命期间提前推进任务航段。
        EntityRouteChangeS(
            state=(FormStageE.STANDBY, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.RALLY_JOIN,
                pos_track=PosTrackStrategyE.PID_SPEED,
            ),
        ),
        # 切出完成后沿任务航线飞行；LOOSE/HOLD 沿时间轴继承这组产品。
        # 长机使用速度控制，位置目标由任务航线插值得到。
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
"""通用长机身份证。注意：所有长机实例共享此不可变策略配置。"""


FOLLOWER_PROFILE = EntityProfileS(
    identity=EntityProfileE.FOLLOWER,
    state_sequence=FORMATION_STATE_SEQUENCE,
    route_changes=(
        # 僚机冷启动同样停控，等待本地待命或长机广播建立有效命令。
        EntityRouteChangeS(
            state=(FormStageE.NONE, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.NOOP,
                pos_track=PosTrackStrategyE.NOOP,
            ),
        ),
        # 待命和 JOINING 与长机平等使用 RallyJoinPos，不提前进入槽位跟随。
        # 速度型控制负责执行切线、盘旋和切出轨迹。
        EntityRouteChangeS(
            state=(FormStageE.STANDBY, RallyPhaseE.JOINING),
            strategies=EntityStrategiesS(
                tra_plan=TraPlanStrategyE.NOOP,
                pos_calc=PosCalcStrategyE.RALLY_JOIN,
                pos_track=PosTrackStrategyE.PID_SPEED,
            ),
        ),
        # CATCHUP 起切到最终槽位几何，LOOSE/HOLD 继续继承且不做渐进缩放。
        # 僚机没有任务航线规划，位置跟踪使用槽位位置闭环。
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
"""通用僚机身份证。注意：所有僚机实例共享此不可变策略配置。"""


def create_leader_follower_entity(identity: EntityProfileE) -> EntityBase:
    """按实体身份创建独立实例。注意：Profile 可共享，Entity 运行状态不可共享。"""
    if identity == EntityProfileE.LEADER:
        from src.algorithm.entity.leader_follower.leader import LeaderEntity

        return LeaderEntity()
    if identity == EntityProfileE.FOLLOWER:
        from src.algorithm.entity.leader_follower.follower import FollowerEntity

        return FollowerEntity()
    raise ValueError(f"不支持的领航跟随实体身份: {identity!r}")


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
