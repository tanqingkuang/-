"""PID 原子控制律。注意：积分和输出限幅由配置决定。"""

from __future__ import annotations

from src.algorithm.units.algo.ctrl.base import CtrlBase, CtrlInitS
from src.algorithm.units.algo.formation_math import clamp


class Pid(CtrlBase):
    """单轴 PID 控制器，微分项使用速度误差。注意：reset 会清除积分和上一拍误差。"""

    def __init__(self) -> None:
        """初始化 Pid 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = CtrlInitS()
        self._integral = 0.0

    def init(self, cfg: CtrlInitS) -> None:
        """按配置初始化 Pid。注意：调用方需先准备好必要依赖和输入数据。"""
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        self._cfg = cfg
        self.reset()

    def step(self, posErr: float, velErr: float) -> float:
        """推进 Pid 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        self._integral += posErr * self._cfg.dt
        if self._cfg.iMax > 0.0:
            self._integral = clamp(self._integral, -self._cfg.iMax, self._cfg.iMax)
        output = self._cfg.kp * posErr + self._cfg.ki * self._integral + self._cfg.kd * velErr
        if self._cfg.outMax > 0.0:
            output = clamp(output, -self._cfg.outMax, self._cfg.outMax)
        return output

    def reset(self) -> None:
        """复位 Pid 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._integral = 0.0
