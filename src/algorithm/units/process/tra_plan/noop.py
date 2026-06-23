"""僚机占位流程使用的空轨迹规划器。注意：不写入航线输出。"""

from __future__ import annotations

from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS, TraPlanInputS, TraPlanOutputS


class Noop(TraPlanBase):
    """空轨迹规划器：占位用，三个生命周期方法均不做任何事，不写航线输出。注意：僚机航路由长机决定，故此处无需规划。"""

    def init(self, cfg: TraPlanInitS) -> None:
        """按配置初始化 Noop。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        """推进 Noop 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        del u, y

    def reset(self) -> None:
        """复位 Noop 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
