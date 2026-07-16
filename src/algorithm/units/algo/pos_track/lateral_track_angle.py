"""横侧向"串级 P+PI + 航迹角变限幅"控制律。

背景与推导见 docs/横侧向点号切入问题/横侧向点号切入问题.md(2.1 串联、2.2 变限幅)。

核心问题：这架三自由度质点机的右向加速度只能靠转弯兑现(见 environment/model.py：
a_right→nz/phi→psi_dot，且 nz 与左转为正的 psi_dot 异号)，因此"大侧偏→持续的大侧向指令"就是"持续转弯"，会把航迹画成圆。
仅靠限侧向加速度救不了(有界但持续的侧向加速度=定半径圆)，必须约束**累积航迹角(拦截角)**。

做法：串级 P+PI——
  外环 P：横偏 dZ → 指令"侧向速度误差"(即拦截角的代理量)，并按侧偏做**变限幅**；
  内环 PI：对(侧向速度误差 − 指令)做 P(+可选 I)，输出侧向加速度。

坐标系约定：一切在"目标速度系"(苏联式航迹系，侧向轴以右为正)下度量，
本机侧向速度误差 velErr = V_self·sin(χ)，χ=本机航向−目标航向；
故把指令 velErr 饱和到 ±V·sin(ψ_max) 恰好等价于把指令拦截角限到 ψ_max。

参数为串级 P+PI 直接量(与前向/垂向 PPIInitS 同构)：外环 kpPos、内环 kpVel/kiVel。
无饱和 + kiVel=0 时，输出 = kpVel·(kpPos·dZ + velErr) = (kpPos·kpVel)·dZ + kpVel·velErr，
即等效并联 PD：位置增益 Kp=kpPos·kpVel、速度增益 Kd=kpVel(阻尼 ζ=0.5·√(kpVel/kpPos)，与 PPI 一致)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.units.algo.formation_math import clamp

_GRAVITY_MPS2 = 9.80665


@dataclass
class LateralTrackAngleInitS:
    """横侧向串级 P+PI + 变限幅初始化参数。

    与前向/垂向 PPIInitS 同构：外环 P 把横偏转成指令侧向速度误差，内环 PI 跟踪该指令。
    小侧偏无饱和时等效并联 PD：位置增益 Kp=kpPos·kpVel、速度增益 Kd=kpVel。
    """

    kpPos: float = 0.0  # 外环：横偏 dZ → 指令侧向速度误差(拦截角代理量)
    kpVel: float = 0.0  # 内环：侧向速度误差比例，必须非零
    kiVel: float = 0.0  # 内环：侧向速度误差积分(以加速度贡献量累积)
    dt: float = 0.0
    # 执行层限幅用**滚转角**(而非侧向加速度)：协调平飞下 a_lat=g·tan(φ)，故把侧向加速度限到 g·tan(rollMaxRad)。
    # 比"限侧向加速度"物理(侧向本就是 bank-to-turn)，且与模型滚转硬限(±70°)同量纲、便于对齐。
    rollMaxRad: float = math.radians(40.0)  # 侧向滚转角限幅(执行层)，必须在 (0, pi/2)
    gammaMaxRad: float = math.radians(25.0)  # 定变限幅半径 R=V²/(g·sinΓmax)·margin 的最大航迹角(转弯半径尺度)，调参旋钮
    floorRad: float = math.radians(7.0)  # 航迹角限幅地板，避免中心线附近仍大角震荡
    # 指令航迹角上限(变限幅天花板)：限的是**指令**角，实际角由内环动态决定会过冲。
    # 默认 pi/2(垂直切入，旧行为)；抬内环带宽 kpVel 后，大侧偏垂直切入的实际角会过冲越过 90°、
    # 机头转过垂直→东向瞬间后退，故把指令天花板收到 <90°、给内环过冲留裕度(见 leader.py 整定说明)。
    psiCmdMaxRad: float = math.pi / 2  # 必须在 (floorRad, pi/2]
    margin: float = 1.2  # R 余量(向上留裕度)


class LateralTrackAngle:
    """横侧向串级 P+PI + 按侧偏的航迹角变限幅控制器。

    注意：不实现 CtrlBase.step(posErr, velFf, velActual) 接口——它需要本机地速标量 V 来做
    "航迹角↔侧向速度"的映射和 R 计算，而该量不在通用三参签名里，故由 PidCompose 直接以
    (dZ, velErr, V) 调用；向心前馈(lateral_ff)仍由 PidCompose 在外部叠加。
    """

    def __init__(self) -> None:
        """初始化 LateralTrackAngle 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = LateralTrackAngleInitS()
        self._integral = 0.0  # 内环速度误差积分(以加速度贡献量存储，kiVel 已折入)
        self.last_vel_err_cmd = 0.0  # 最近一拍三段航迹角约束对应的侧向速度误差指令，供上层生成有效航迹指令。
        self.last_saturated = False  # 最近一拍是否处于 7° 地板以外的航迹角约束区间。

    def init(self, cfg: LateralTrackAngleInitS) -> None:
        """按配置初始化 LateralTrackAngle。注意：调用方需先准备好必要依赖和输入数据。"""
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        if cfg.kpVel == 0.0:
            # 内环比例=kpVel 必须非零，否则串级内环无输出、退化无意义。
            raise ValueError("kpVel must be non-zero for cascade lateral control")
        if not 0.0 < cfg.rollMaxRad < math.pi / 2:
            raise ValueError("rollMaxRad must be in (0, pi/2)")
        if not 0.0 < cfg.gammaMaxRad < math.pi / 2:
            raise ValueError("gammaMaxRad must be in (0, pi/2)")
        if not 0.0 <= cfg.floorRad < math.pi / 2:
            raise ValueError("floorRad must be in [0, pi/2)")
        if not cfg.floorRad < cfg.psiCmdMaxRad <= math.pi / 2:
            # 指令天花板必须高于地板、且不超过垂直切入。
            raise ValueError("psiCmdMaxRad must be in (floorRad, pi/2]")
        if cfg.margin <= 0.0:
            raise ValueError("margin must be > 0")
        self._cfg = cfg
        # 滚转角限幅折算成侧向加速度上限：a_lat_max = g·tan(rollMax)。
        self._acc_max = _GRAVITY_MPS2 * math.tan(cfg.rollMaxRad)
        self.reset()

    def track_angle_limit_rad(self, dz_abs: float, v_ground: float) -> float:
        """按侧偏幅值给出当前允许的最大航迹角(拦截角)限幅，弧度。

        R = V²/(g·sinΓmax)·margin(借鉴 L1 前视半径)；天花板取 psiCmdMax(默认 90° 垂直切入)：
        |dZ|>=R → psiCmdMax；R·sin(floor)<=|dZ|<R → min(asin(|dZ|/R), psiCmdMax)；|dZ|<R·sin(floor) → floor。
        """
        cfg = self._cfg
        cap = cfg.psiCmdMaxRad
        radius = v_ground * v_ground / (_GRAVITY_MPS2 * math.sin(cfg.gammaMaxRad)) * cfg.margin
        if radius <= 1e-9:
            return cap
        if dz_abs >= radius:
            return cap
        floor_dz = radius * math.sin(cfg.floorRad)
        if dz_abs <= floor_dz:
            return cfg.floorRad
        return min(math.asin(clamp(dz_abs / radius, 0.0, 1.0)), cap)

    def step(self, dz: float, vel_err: float, v_ground: float) -> float:
        """推进一个周期，返回目标速度系下的侧向加速度(不含向心前馈)。

        dz: 目标系横偏位置误差(cmd-self，右为正)；
        vel_err: 目标系侧向速度误差(velFf-velActual，≈ V_self·sin(χ))；
        v_ground: 本机地速标量，用于航迹角↔侧向速度映射与 R 计算。
        """
        cfg = self._cfg
        v = max(v_ground, 0.0)
        # 外环 P：横偏 → 指令侧向速度误差(拦截角代理量)，直接用 kpPos。
        vel_err_cmd = -cfg.kpPos * dz
        # 变限幅：把指令拦截角限到 psi_max，等价把指令侧向速度误差限到 ±V·sin(psi_max)。
        psi_max = self.track_angle_limit_rad(abs(dz), v)
        sat = v * math.sin(psi_max)
        vel_err_cmd = clamp(vel_err_cmd, -sat, sat)
        radius = v * v / (_GRAVITY_MPS2 * math.sin(cfg.gammaMaxRad)) * cfg.margin
        floor_dz = radius * math.sin(cfg.floorRad) if radius > 1e-9 else 0.0
        self.last_saturated = abs(dz) > floor_dz
        # 上层广播的是三段几何约束航迹角，不是外环 P 指令是否刚好被 clamp 后的结果。
        self.last_vel_err_cmd = -math.copysign(sat, dz) if self.last_saturated and dz != 0.0 else vel_err_cmd

        # 内环 PI：对(实际−指令)做 P(+可选 I)。kiVel=0 时退化纯 P，无饱和时等效并联 PD。
        err = vel_err - vel_err_cmd
        acc = cfg.kpVel * err
        if cfg.kiVel != 0.0:
            self._integral += cfg.kiVel * err * cfg.dt
            self._integral = clamp(self._integral, -self._acc_max, self._acc_max)
            acc += self._integral

        # 执行层滚转角限幅：把侧向加速度夹到 ±g·tan(rollMax)。
        acc_clamped = clamp(acc, -self._acc_max, self._acc_max)
        # 抗饱和(反算法)：输出被夹时把溢出从积分回退，防止积分绕死。
        if cfg.kiVel != 0.0 and acc_clamped != acc:
            self._integral -= acc - acc_clamped
            self._integral = clamp(self._integral, -self._acc_max, self._acc_max)
        return acc_clamped

    def reset(self) -> None:
        """复位 LateralTrackAngle 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._integral = 0.0
        self.last_vel_err_cmd = 0.0
        self.last_saturated = False
