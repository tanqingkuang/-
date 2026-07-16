"""轨迹规划基础接口。注意：输出航段需保持完整起终点信息。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


class TraPlanStrategyE(IntEnum):
    """轨迹规划策略枚举。注意：只表达规划能力，不表达实体角色。"""

    NOOP = 0  # 不更新当前任务航段
    LEADER_ROUTE = 1  # 按任务航线推进当前航段


@dataclass
class TraPlanInitS:
    """轨迹规划初始化配置基类。注意：具体规划器可派生扩展字段。"""

    pass


class TraPlanBase:
    """轨迹规划单元抽象基类。注意：具体策略负责完整黑板读写事务。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：具体策略自行维护所需快照。"""
        raise NotImplementedError

    def init(self, cfg: TraPlanInitS) -> None:
        """按配置初始化 TraPlanBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """推进一个轨迹规划周期。注意：策略自行读取并提交黑板。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 TraPlanBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
