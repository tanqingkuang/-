"""入站消息处理基础接口。注意：实现需过滤不相关 topic。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS
from src.common.envelope import MessageEnvelope


@dataclass
class InboundInitS:
    pass


@dataclass
class InboundInputS:
    inbox: list[MessageEnvelope] = field(default_factory=list)


@dataclass
class InboundOutputS:
    leaderState: MotionProfS | None = None
    cmd: FormSnapshotS | None = None


class InboundBase:
    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 InboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        """推进 InboundBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 InboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
