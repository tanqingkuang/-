"""位置跟踪基础接口。注意：低速奇异情况应显式处理。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import AccInEarthS, MotionProfS


@dataclass
class PosTrackInitS:
    pass


@dataclass
class PosTrackInputS:
    selfCmd: MotionProfS | None = None
    selfState: MotionProfS | None = None


@dataclass
class PosTrackOutputS:
    accCmd: AccInEarthS | None = None


class PosTrackBase:
    def init(self, cfg: PosTrackInitS) -> None:
        """按配置初始化 PosTrackBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        """推进 PosTrackBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosTrackBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
