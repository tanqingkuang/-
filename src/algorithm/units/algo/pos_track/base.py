"""位置跟踪基础接口。注意：低速奇异情况应显式处理。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    PosTrackStrategyE as PosTrackStrategyE,
)

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


@dataclass
class PosTrackInitS:
    """位置跟踪初始化基类。注意：具体跟踪器可继承后追加控制参数。"""

    pass


class PosTrackBase:
    """位置跟踪算法基类。注意：子类负责把目标运动剖面转换为加速度命令。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：具体产品自行维护控制快照。"""
        raise NotImplementedError

    def init(self, cfg: PosTrackInitS) -> None:
        """按配置初始化 PosTrackBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """推进一个位置跟踪周期。注意：产品自行读取并提交黑板。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosTrackBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
