"""入站消息处理基础接口。注意：实现需过滤不相关 topic。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS
from src.common.envelope import MessageEnvelope


@dataclass
class InboundInitS:
    """入站单元初始化配置基类。注意：当前无字段，预留派生扩展。"""

    pass


@dataclass
class InboundInputS:
    """入站单元输入端口。注意：inbox 含本帧全部消息，需自行按 topic 过滤。"""

    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧收到的消息


@dataclass
class InboundOutputS:
    """入站单元输出端口。注意：解析结果写入长机状态与编队指令。"""

    leaderState: MotionProfS | None = None  # 解析出的长机运动状态
    cmd: FormSnapshotS | None = None  # 解析出的编队指令


class InboundBase:
    """入站消息处理抽象基类。注意：子类须过滤无关 topic，仅消费目标主题的消息。"""

    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 InboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        """推进 InboundBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 InboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
