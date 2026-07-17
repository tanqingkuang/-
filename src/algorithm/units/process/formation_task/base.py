"""编队任务编排基础接口。注意：具体任务自行读取并更新实体黑板。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityManagerInitS, EntityRuntimeS


@dataclass
class FormationTaskInitS:
    """编队任务初始化配置基类。注意：当前无字段，预留派生扩展。"""

    pass


class FormationTaskBase:
    """编队任务编排抽象基类。注意：子类须把决策出的阶段和队形写入黑板。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：具体任务自行维护所需快照。"""
        raise NotImplementedError

    def init(self, cfg: EntityManagerInitS) -> None:
        """按配置初始化 FormationTaskBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """推进一个任务编排周期。注意：具体任务自行读取并提交黑板。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 FormationTaskBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
