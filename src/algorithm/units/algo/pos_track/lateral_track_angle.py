"""横侧向"串级 P+PI + 航迹角变限幅"控制律。

背景与推导见 docs/横侧向点号切入问题/横侧向点号切入问题.md(2.1 串联、2.2 变限幅)。

核心问题：这架三自由度质点机的侧向加速度只能靠转弯兑现(见 environment/model.py:
a_psi→nz/phi→psi_dot)，因此"大侧偏→持续的大侧向指令"就是"持续转弯"，会把航迹画成圆。
仅靠限侧向加速度救不了(有界但持续的侧向加速度=定半径圆)，必须约束**累积航迹角(拦截角)**。

做法：把旧并联式侧偏 PID 等价改写为串级——
  外环：横偏 dZ → 指令"侧向速度误差"(即拦截角的代理量)，并按侧偏做**变限幅**；
  内环：对(侧向速度误差 − 指令)做 P(+可选 I)，输出侧向加速度。

坐标系约定：一切在"目标速度系"(苏联式航迹系，侧向轴以右为正)下度量，
本机侧向速度误差 velErr = V_self·sin(χ)，χ=本机航向−目标航向；
故把指令 velErr 饱和到 ±V·sin(ψ_max) 恰好等价于把指令拦截角限到 ψ_max。

无饱和 + ki=0 时，输出 = kp·dZ + kd·velErr，与旧并联式侧偏 PID 严格相等(平滑迁移)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.units.algo.formation_math import clamp

_GRAVITY_MPS2 = 9.80665


@dataclass
class LateralTrackAngleInitS:
    """横侧向串级 + 变限幅初始化参数。

    kp/kd/ki 与旧并联式侧偏 PID(CtrlInitS)一一对应，便于等价迁移：
    外环 K1=-kp/kd(横偏→指令侧向速度误差)，内环比例=kd，内环积分 K3=kd·ki/kp(ki=0 时为 0)。
    """

    kp: float = 0.0  # 横偏比例(等价旧并联 kp)
    kd: float = 0.0  # 侧向速度误差/航迹角比例(等价旧并联 kd)，必须非零
    ki: float = 0.0  # 横偏积分(等价旧并联 ki)；串联内环积分 K3=kd·ki/kp
    dt: float = 0.0
    # 执行层限幅用**滚转角**(而非侧向加速度)：协调平飞下 a_lat=g·tan(φ)，故把侧向加速度限到 g·tan(rollMaxRad)。
    # 比"限侧向加速度"物理(侧向本就是 bank-to-turn)，且与模型滚转硬限(±70°)同量纲、便于对齐。
    rollMaxRad: float = math.radians(40.0)  # 侧向滚转角限幅(执行层)，必须在 (0, pi/2)
    gammaMaxRad: float = math.radians(25.0)  # 定变限幅半径 R=V²/(g·sinΓmax)·margin 的最大航迹角(转弯半径尺度)，调参旋钮
    floorRad: float = math.radians(7.0)  # 航迹角限幅地板，避免中心线附近仍大角震荡
    margin: float = 1.2  # R 余量(向上留裕度)


class LateralTrackAngle:
    """横侧向串级(P+PI) + 按侧偏的航迹角变限幅控制器。

    注意：不实现 CtrlBase.step(posErr, velFf, velActual) 接口——它需要本机地速标量 V 来做
    "航迹角↔侧向速度"的映射和 R 计算，而该量不在通用三参签名里，故由 PidCompose 直接以
    (dZ, velErr, V) 调用；向心前馈(lateral_ff)仍由 PidCompose 在外部叠加。
    """

    def __init__(self) -> None:
        """初始化 LateralTrackAngle 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._cfg = LateralTrackAngleInitS()
        self._integral = 0.0  # 内环速度误差积分(以加速度贡献量存储，K3 已折入)

    def init(self, cfg: LateralTrackAngleInitS) -> None:
        """按配置初始化 LateralTrackAngle。注意：调用方需先准备好必要依赖和输入数据。"""
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        if cfg.kd == 0.0:
            # 外环 K1=-kp/kd、内环比例=kd 都要求 kd 非零，否则串联退化无意义。
            raise ValueError("kd must be non-zero for cascade lateral control")
        if cfg.ki != 0.0 and cfg.kp == 0.0:
            # 内环积分 K3=kd·ki/kp 需要 kp 非零；ki=0 时无此约束。
            raise ValueError("kp must be non-zero when ki != 0")
        if not 0.0 < cfg.rollMaxRad < math.pi / 2:
            raise ValueError("rollMaxRad must be in (0, pi/2)")
        if not 0.0 < cfg.gammaMaxRad < math.pi / 2:
            raise ValueError("gammaMaxRad must be in (0, pi/2)")
        if not 0.0 <= cfg.floorRad < math.pi / 2:
            raise ValueError("floorRad must be in [0, pi/2)")
        if cfg.margin <= 0.0:
            raise ValueError("margin must be > 0")
        self._cfg = cfg
        # 滚转角限幅折算成侧向加速度上限：a_lat_max = g·tan(rollMax)。
        self._acc_max = _GRAVITY_MPS2 * math.tan(cfg.rollMaxRad)
        self.reset()

    def track_angle_limit_rad(self, dz_abs: float, v_ground: float) -> float:
        """按侧偏幅值给出当前允许的最大航迹角(拦截角)限幅，弧度。

        R = V²/(g·sinΓmax)·margin(借鉴 L1 前视半径)；
        |dZ|>=R → 90°(垂直切入)；R·sin(floor)<=|dZ|<R → asin(|dZ|/R)；|dZ|<R·sin(floor) → floor。
        """
        cfg = self._cfg
        radius = v_ground * v_ground / (_GRAVITY_MPS2 * math.sin(cfg.gammaMaxRad)) * cfg.margin
        if radius <= 1e-9:
            return math.pi / 2
        if dz_abs >= radius:
            return math.pi / 2
        floor_dz = radius * math.sin(cfg.floorRad)
        if dz_abs <= floor_dz:
            return cfg.floorRad
        return math.asin(clamp(dz_abs / radius, 0.0, 1.0))

    def step(self, dz: float, vel_err: float, v_ground: float) -> float:
        """推进一个周期，返回目标速度系下的侧向加速度(不含向心前馈)。

        dz: 目标系横偏位置误差(cmd-self，右为正)；
        vel_err: 目标系侧向速度误差(velFf-velActual，≈ V_self·sin(χ))；
        v_ground: 本机地速标量，用于航迹角↔侧向速度映射与 R 计算。
        """
        cfg = self._cfg
        v = max(v_ground, 0.0)
        # 外环：横偏 → 指令侧向速度误差(拦截角代理量)，K1=-kp/kd。
        vel_err_cmd = -(cfg.kp / cfg.kd) * dz
        # 变限幅：把指令拦截角限到 psi_max，等价把指令侧向速度误差限到 ±V·sin(psi_max)。
        psi_max = self.track_angle_limit_rad(abs(dz), v)
        sat = v * math.sin(psi_max)
        vel_err_cmd = clamp(vel_err_cmd, -sat, sat)

        # 内环：对(实际−指令)做 P(+可选 I)。ki=0 时 K3=0，退化为纯 P，与旧并联式严格等价。
        err = vel_err - vel_err_cmd
        acc = cfg.kd * err
        if cfg.ki != 0.0:
            k3 = cfg.kd * cfg.ki / cfg.kp
            self._integral += k3 * err * cfg.dt
            self._integral = clamp(self._integral, -self._acc_max, self._acc_max)
            acc += self._integral

        # 执行层滚转角限幅：把侧向加速度夹到 ±g·tan(rollMax)。
        acc_clamped = clamp(acc, -self._acc_max, self._acc_max)
        # 抗饱和(反算法)：输出被夹时把溢出从积分回退，防止积分绕死。
        if cfg.ki != 0.0 and acc_clamped != acc:
            self._integral -= acc - acc_clamped
            self._integral = clamp(self._integral, -self._acc_max, self._acc_max)
        return acc_clamped

    def reset(self) -> None:
        """复位 LateralTrackAngle 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._integral = 0.0
