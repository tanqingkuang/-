"""目标位置计算基础接口。注意：集结实体由具体策略维护端口，显式端口仅兼容旧调用。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FormSnapshotS,
    MotionProfS,
    PosCalcStatusS,
    PosCalcStrategyE as PosCalcStrategyE,
    PosTrackCommandS,
    RallyPlanS,
    WayLineS,
)

if TYPE_CHECKING:
    from src.algorithm.context.context import FormContextS


@dataclass
class PosCalcInitS:
    """目标位置计算初始化基类。注意：具体算法可继承后追加配置字段。"""

    pass


@dataclass
class PosCalcInputS:
    """旧式统一输入端口。注意：仅供既有显式端口调用兼容，新策略应定义专属输入。"""

    selfState: MotionProfS | None = None  # 本机实测运动状态
    cmd: FormSnapshotS | None = None  # 任务编排产生的阶段和队形指令
    wayLine: WayLineS | None = None  # 轨迹规划产生的当前航段
    nextWayLine: WayLineS | None = None  # 曲率前馈使用的下一航段
    leaderState: MotionProfS | None = None  # 僚机收到的长机实测状态
    leaderCmd: MotionProfS | None = None  # 僚机收到的长机目标状态
    clock: AlgorithmClockS | None = None  # 跨流程共享的仿真时钟引用
    rallyPlan: RallyPlanS | None = None  # 跨流程共享的集结计划引用


@dataclass
class PosCalcOutputS:
    """旧式统一输出端口。注意：仅供既有显式端口调用兼容，新策略应定义专属输出。"""

    selfCmd: MotionProfS | None = None  # 写入黑板的本机目标运动状态
    status: PosCalcStatusS | None = None  # 写入黑板的位置解算运行状态
    posTrackCommand: PosTrackCommandS | None = None  # 写入下一流程的控制语义命令


class PosCalcBase:
    """目标位置计算算法基类。注意：子类只负责生成目标运动剖面，不直接输出加速度。"""

    def bind(self, cxt: FormContextS) -> None:
        """绑定算法黑板。注意：具体策略自行创建专属输入输出快照。"""
        raise NotImplementedError

    def init(self, cfg: PosCalcInitS) -> None:
        """按配置初始化 PosCalcBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(
        self,
        u: PosCalcInputS | None = None,
        y: PosCalcOutputS | None = None,
    ) -> None:
        """推进一个处理周期。注意：无参模式使用策略内部快照，显式端口仅用于兼容。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosCalcBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError


def reset_pos_calc_status(status: PosCalcStatusS, strategy: PosCalcStrategyE) -> None:
    """重置并标记公共位置解算状态。注意：具体策略随后写入自身扩展字段。"""
    status.active_strategy = strategy
    status.rally_state = ""
    status.planned_path_length_m = -1.0
    status.remaining_path_length_m = -1.0
    status.remaining_loops = 0
    status.reached_slot_once = False
    status.join_exited = False


def copy_pos_calc_status(src: PosCalcStatusS, dst: PosCalcStatusS) -> None:
    """原地复制位置解算状态。注意：不得替换黑板持有的状态对象。"""
    dst.active_strategy = src.active_strategy
    dst.rally_state = src.rally_state
    dst.planned_path_length_m = src.planned_path_length_m
    dst.remaining_path_length_m = src.remaining_path_length_m
    dst.remaining_loops = src.remaining_loops
    dst.reached_slot_once = src.reached_slot_once
    dst.join_exited = src.join_exited
