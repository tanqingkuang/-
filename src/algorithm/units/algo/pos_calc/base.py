"""目标位置计算基础接口。注意：输出端口需由调用方绑定。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS


@dataclass
class PosCalcInitS:
    """目标位置计算初始化基类。注意：具体算法可继承后追加配置字段。"""

    pass


@dataclass
class PosCalcInputS:
    """目标位置计算输入端口。注意：selfState 需要由实体边界在每拍写入。"""

    selfState: MotionProfS | None = None


@dataclass
class PosCalcOutputS:
    """目标位置计算输出端口。注意：selfCmd 由调用方预先绑定可写对象。"""

    selfCmd: MotionProfS | None = None


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
