"""僚机占位流程使用的空轨迹规划器。注意：不写入航线输出。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


class Noop(TraPlanBase):
    """空轨迹规划器：占位用，三个生命周期方法均不做任何事，不写航线输出。注意：僚机航路由长机决定，故此处无需规划。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定运行环境。注意：空策略不读取任何运行数据。"""
        del runtime

    def init(self, cfg: TraPlanInitS) -> None:
        """按配置初始化 Noop。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self) -> None:
        """推进空轨迹规划周期。注意：不得改写当前航段。"""
        return None

    def reset(self) -> None:
        """复位 Noop 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
