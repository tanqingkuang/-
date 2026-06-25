"""串级 P+PI 控制律。注意：外环纯 P 把位置误差转成速度反馈，叠加速度前馈后限幅，内环 PI 跟踪该速度指令并输出限幅加速度。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.units.algo.ctrl.base import CtrlBase
from src.algorithm.units.algo.formation_math import clamp


@dataclass
class PPIInitS:
    """串级 P+PI 初始化参数。注意：外环只有比例(kpPos)，内环为 PI(kpVel/kiVel)，速度指令与加速度均支持非对称限幅。"""

    kpPos: float = 0.0  # 外环：位置误差 -> 速度反馈
    kpVel: float = 0.0  # 内环：速度误差比例
    kiVel: float = 0.0  # 内环：速度误差积分
    dt: float = 0.0
    iOutMax: float = 0.0  # 内环积分(以加速度贡献量存储)限幅(<=0 表示不限)
    vCmdMin: float = float("-inf")  # 速度指令下限(非对称)
    vCmdMax: float = float("inf")  # 速度指令上限(非对称)
    accMin: float = float("-inf")  # 加速度输出下限(非对称)
    accMax: float = float("inf")  # 加速度输出上限(非对称)


class PPI(CtrlBase):
    """串级 P+PI 控制器：位置误差经外环 P 转速度反馈，叠加速度前馈并限幅得速度指令，内环 PI 跟踪。注意：积分以加速度贡献量存储，输出饱和时按反算法回退积分抗饱和。"""

    def __init__(self) -> None:
        """初始化 PPI 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = PPIInitS()
        self._integral = 0.0  # 内环速度误差积分，存为加速度贡献量(kiVel 已折入)

    def init(self, cfg: PPIInitS) -> None:
        """按配置初始化 PPI。注意：调用方需先准备好必要依赖和输入数据。"""
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        if cfg.vCmdMin > cfg.vCmdMax:
            raise ValueError("vCmdMin must be <= vCmdMax")
        if cfg.accMin > cfg.accMax:
            raise ValueError("accMin must be <= accMax")
        self._cfg = cfg
        self.reset()

    def step(self, posErr: float, velFf: float, velActual: float) -> float:
        """推进 PPI 一个处理周期。注意：posErr 为位置误差，velFf 为速度前馈(原指令)，velActual 为实测速度。"""
        # 外环纯 P：位置误差 -> 速度反馈，叠加前馈后限幅，得到可下手限速的速度指令。
        vel_cmd = velFf + self._cfg.kpPos * posErr
        vel_cmd = clamp(vel_cmd, self._cfg.vCmdMin, self._cfg.vCmdMax)
        vel_err = vel_cmd - velActual

        # 内环 PI：积分以加速度贡献量累积(kiVel 折入)，便于饱和时按输出量直接回退。
        self._integral += self._cfg.kiVel * vel_err * self._cfg.dt
        if self._cfg.iOutMax > 0.0:
            self._integral = clamp(self._integral, -self._cfg.iOutMax, self._cfg.iOutMax)

        acc = self._cfg.kpVel * vel_err + self._integral
        acc_clamped = clamp(acc, self._cfg.accMin, self._cfg.accMax)

        # 抗饱和(反算法)：输出被夹时把溢出从积分里回退，再夹回 iOutMax，防止积分持续累积绕死。
        if self._cfg.kiVel != 0.0 and acc_clamped != acc:
            self._integral -= acc - acc_clamped
            if self._cfg.iOutMax > 0.0:
                self._integral = clamp(self._integral, -self._cfg.iOutMax, self._cfg.iOutMax)

        return acc_clamped

    def reset(self) -> None:
        """复位 PPI 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._integral = 0.0
