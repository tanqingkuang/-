"""出站消息处理基础接口。注意：实现需维护消息目标和载荷格式。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, NetWorkS
from src.common.envelope import MessageEnvelope


@dataclass
class OutboundInitS:
    """出站单元初始化配置。注意：netWork 用于解析广播目标。"""

    selfId: str = ""  # 本机标识，用于在拓扑中识别自身链路
    netWork: list[NetWorkS] = field(default_factory=list)  # 通信拓扑链路集合


@dataclass
class OutboundInputS:
    """出站单元输入端口。注意：cmd 与 selfState 一并打包进消息载荷。"""

    cmd: FormSnapshotS | None = None  # 待广播的编队指令
    selfState: MotionProfS | None = None  # 待广播的本机状态


@dataclass
class OutboundOutputS:
    """出站单元输出端口。注意：outbox 由 step 填充并被外层取走。"""

    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧生成的待发消息


class OutboundBase:
    """出站消息处理抽象基类。注意：子类须自行计算目标列表并按约定格式封装载荷。"""

    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 OutboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: OutboundInputS, y: OutboundOutputS) -> None:
        """推进 OutboundBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 OutboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
