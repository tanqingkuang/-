"""入站消息处理基础接口。注意：实现需过滤不相关 topic。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


@dataclass
class InboundInitS:
    """入站单元初始化配置基类。注意：当前无字段，预留派生扩展。"""

    pass


class InboundBase:
    """入站消息处理抽象基类。注意：子类须过滤无关 topic，仅消费目标主题的消息。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：具体入站实现自行维护协议快照。"""
        raise NotImplementedError

    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 InboundBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """处理一个入站周期。注意：具体实现自行读取邮箱并提交黑板。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 InboundBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
