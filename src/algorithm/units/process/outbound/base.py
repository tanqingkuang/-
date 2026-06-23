"""出站消息处理基础接口。注意：实现需维护消息目标和载荷格式。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, NetWorkS
from src.common.envelope import MessageEnvelope


@dataclass
class OutboundInitS:
    selfId: str = ""
    netWork: list[NetWorkS] = field(default_factory=list)


@dataclass
class OutboundInputS:
    cmd: FormSnapshotS | None = None
    selfState: MotionProfS | None = None


@dataclass
class OutboundOutputS:
    outbox: list[MessageEnvelope] = field(default_factory=list)


class OutboundBase:
    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 OutboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: OutboundInputS, y: OutboundOutputS) -> None:
        """推进 OutboundBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 OutboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
