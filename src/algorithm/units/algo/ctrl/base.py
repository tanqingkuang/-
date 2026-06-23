"""原子控制律基础接口。注意：具体控制律需实现统一生命周期。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CtrlInitS:
    """单轴控制律初始化参数。注意：dt、限幅和增益需由上层按控制周期配置。"""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    dt: float = 0.0
    iMax: float = 0.0
    outMax: float = 0.0


class CtrlBase:
    """原子控制律基类。注意：子类必须实现初始化、单步计算和复位接口。"""

    def init(self, cfg: CtrlInitS) -> None:
        """按配置初始化 CtrlBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, posErr: float, velErr: float) -> float:
        """推进 CtrlBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 CtrlBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
