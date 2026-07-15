"""目标位置计算基础接口。注意：输出端口需由调用方绑定。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FormSnapshotS,
    MotionProfS,
    PosCalcStatusS,
    PosCalcStrategyE,
    PosTrackCommandS,
    RallyPlanS,
    WayLineS,
)


@dataclass
class PosCalcInitS:
    """目标位置计算初始化基类。注意：具体算法可继承后追加配置字段。"""

    pass


@dataclass
class PosCalcInputS:
    """目标位置计算统一输入端口。注意：具体策略只校验和读取自身需要的字段。"""

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
    """目标位置计算输出端口。注意：输出对象由调用方预先绑定。"""

    selfCmd: MotionProfS | None = None  # 写入黑板的本机目标运动状态
    status: PosCalcStatusS | None = None  # 写入黑板的位置解算运行状态
    posTrackCommand: PosTrackCommandS | None = None  # 写入下一流程的控制语义命令


class PosCalcBase:
    """目标位置计算算法基类。注意：子类只负责生成目标运动剖面，不直接输出加速度。"""

    def init(self, cfg: PosCalcInitS) -> None:
        """按配置初始化 PosCalcBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: PosCalcInputS, y: PosCalcOutputS) -> None:
        """推进 PosCalcBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosCalcBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
