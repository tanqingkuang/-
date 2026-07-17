"""出站消息处理基础接口。注意：实现需维护消息目标和载荷格式。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import NetWorkS

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


class OutboundMessageE(IntEnum):
    """出站消息类型。注意：实体初始化后不得在运行期切换。"""

    NOOP = 0  # 不生成报文；用于直接 HOLD 的普通僚机保持既有通信语义
    LEADER_BROADCAST = 1  # 长机向通信拓扑中的僚机广播状态、指令和集结计划
    FOLLOWER_STATUS = 2  # 僚机向指定长机回报位置误差和集结状态


@dataclass
class OutboundInitS:
    """出站单元初始化配置。注意：netWork 用于解析广播目标。"""

    selfId: str = ""  # 本机标识，用于在拓扑中识别自身链路
    netWork: list[NetWorkS] = field(default_factory=list)  # 通信拓扑链路集合


class OutboundBase:
    """出站消息处理抽象基类。注意：子类须自行计算目标列表并按约定格式封装载荷。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：具体出站实现自行维护协议快照。"""
        raise NotImplementedError

    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 OutboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """处理一个出站周期。注意：具体实现自行读取黑板并写出件箱。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 OutboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
