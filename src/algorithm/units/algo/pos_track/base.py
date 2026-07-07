"""位置跟踪基础接口。注意：低速奇异情况应显式处理。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import AccInEarthS, MotionProfS, PosTrackDiagS


@dataclass
class PosTrackInitS:
    """位置跟踪初始化基类。注意：具体跟踪器可继承后追加控制参数。"""

    pass


@dataclass
class PosTrackInputS:
    """位置跟踪输入端口。注意：selfCmd 和 selfState 必须同时绑定。"""

    selfCmd: MotionProfS | None = None
    selfState: MotionProfS | None = None


@dataclass
class PosTrackOutputS:
    """位置跟踪输出端口。注意：accCmd 和 diag 由调用方预先绑定可写对象。"""

    accCmd: AccInEarthS | None = None
    diag: PosTrackDiagS | None = None
    effectiveCmd: MotionProfS | None = None  # 位置跟踪后的有效运动指令，供编队坐标系广播使用。


class PosTrackBase:
    """位置跟踪算法基类。注意：子类负责把目标运动剖面转换为加速度命令。"""

    def init(self, cfg: PosTrackInitS) -> None:
        """按配置初始化 PosTrackBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        """推进 PosTrackBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosTrackBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
