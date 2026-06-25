"""PID 原子控制律。注意：积分和输出限幅由配置决定。"""

from __future__ import annotations

from src.algorithm.units.algo.ctrl.base import CtrlBase, CtrlInitS
from src.algorithm.units.algo.formation_math import clamp


class Pid(CtrlBase):
    """通用双通道控制器：位置误差走 kp/ki、速度误差走 kd/kiv，两路积分独立。注意：长机置 kp=ki=0 为速度环(kd 比例、kiv 可选积分)，僚机置 kiv=0 为位置 PID。"""

    def __init__(self) -> None:
        """初始化 Pid 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = CtrlInitS()
        self._integral = 0.0  # 位置误差积分
        self._integral_vel = 0.0  # 速度误差积分

    def init(self, cfg: CtrlInitS) -> None:
        """按配置初始化 Pid。注意：调用方需先准备好必要依赖和输入数据。"""
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        # 互斥保护：位置积分与速度积分同时投入会形成双积分器互相绕死，配置阶段直接拦截。
        if cfg.ki != 0.0 and cfg.kiv != 0.0:
            raise ValueError("ki and kiv are mutually exclusive: set one to 0")
        self._cfg = cfg
        self.reset()

    def step(self, posErr: float, velErr: float) -> float:
        """推进 Pid 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        self._integral += posErr * self._cfg.dt
        if self._cfg.iMax > 0.0:
            self._integral = clamp(self._integral, -self._cfg.iMax, self._cfg.iMax)
        self._integral_vel += velErr * self._cfg.dt
        if self._cfg.iMaxVel > 0.0:
            self._integral_vel = clamp(self._integral_vel, -self._cfg.iMaxVel, self._cfg.iMaxVel)
        output = (
            self._cfg.kp * posErr
            + self._cfg.ki * self._integral
            + self._cfg.kd * velErr
            + self._cfg.kiv * self._integral_vel
        )
        if self._cfg.outMax > 0.0:
            output = clamp(output, -self._cfg.outMax, self._cfg.outMax)
        return output

    def reset(self) -> None:
        """复位 Pid 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._integral = 0.0
        self._integral_vel = 0.0
