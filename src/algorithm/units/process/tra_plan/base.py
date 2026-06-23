"""轨迹规划基础接口。注意：输出航段需保持完整起终点信息。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, WayLineS


@dataclass
class TraPlanInitS:
    pass


@dataclass
class TraPlanInputS:
    cmd: FormSnapshotS | None = None
    wayLine: WayLineS | None = None
    selfState: MotionProfS | None = None


@dataclass
class TraPlanOutputS:
    wayLine: WayLineS | None = None


class TraPlanBase:
    def init(self, cfg: TraPlanInitS) -> None:
        """按配置初始化 TraPlanBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        """推进 TraPlanBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 TraPlanBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
