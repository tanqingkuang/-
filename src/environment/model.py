"""ENU 坐标系下的无人机三自由度质点模型。注意：内部角度使用弧度。"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

from src.common.coordinates import enu_to_fur, fur_basis_from_angles, fur_to_enu


# 状态向量维数：位置(x,y,h)、速度、俯仰、航向、三轴滤波加速度、三轴加速度变化率，共 12 维。
STATE_SIZE = 12


def node_id_from_config(node: dict[str, object], index: int) -> str:
    """从节点配置推导节点 ID。注意：配置缺省时使用序号生成稳定 ID。"""
    # 优先用显式 node_id/id；都缺省时回退为 A01、A02… 保证 ID 唯一且可复现。
    return str(node.get("node_id") or node.get("id") or f"A{index + 1:02d}")


# cos(theta) 的下限：psi_dot 公式分母含 cos(theta)，俯仰接近 ±90° 时会奇异，故钳一个最小余弦值。
_COS_THETA_EPS = 1e-3
# 俯仰角硬限幅，留出 1° 余量避免逼近垂直时的数值奇异。
_MAX_ABS_THETA_RAD = math.radians(89.0)


@dataclass(frozen=True)
class AccelerationCommand:
    """ENU 坐标系下的加速度指令（单位 m/s^2），作为质点模型的外部控制输入。"""

    ax_cmd_mps2: float = 0.0
    ay_cmd_mps2: float = 0.0
    az_cmd_mps2: float = 0.0

    def as_vector(self) -> tuple[float, float, float]:
        """把状态对象转换为数值向量。注意：向量顺序必须与模型积分约定一致。"""
        return (self.ax_cmd_mps2, self.ay_cmd_mps2, self.az_cmd_mps2)


@dataclass(frozen=True)
class PointMassModelConfig:
    """三自由度质点模型的数值和物理常量。注意：配置需满足基础可飞约束。"""

    gravity_mps2: float  # 重力加速度 g
    min_speed_mps: float  # 最小可飞空速，兼作角速率分母下限
    natural_frequency_rad_s: float  # 加速度二阶滤波自然频率 omega
    damping_ratio: float  # 加速度二阶滤波阻尼比 zeta，须在 (0,1)
    acceleration_command_limit_mps2: float  # ENU 加速度指令幅值上限
    max_climb_rate_mps: float  # 最大爬升率，正值，单位米每秒
    max_descent_rate_mps: float  # 最大下沉率，正值，单位米每秒
    nx_min: float  # 切向过载下限
    nx_max: float  # 切向过载上限
    n_normal_min: float  # 法向合过载下限（恒非负）
    n_normal_max: float  # 法向合过载上限
    phi_min_rad: float  # 滚转角下限，右倾为正
    phi_max_rad: float  # 滚转角上限，右倾为正


@dataclass
class AircraftState:
    """单架飞行器完整状态：ENU 位置、空速轨迹角及三轴滤波加速度。

    nx/ny/nz 与偏航角速率为由滤波加速度反算出的派生量，不进入 12 维积分状态向量。
    风速同样不进入积分状态，只用于从空速派生地速，供制导、日志和 UI 使用。
    """

    node_id: str
    x_m: float  # ENU 东向位置
    y_m: float  # ENU 北向位置
    altitude_m: float  # 高度（天向）
    speed_mps: float  # 空速（标量）
    theta_rad: float  # 航迹倾角，正为爬升
    psi_rad: float  # 航向角，自东向逆时针
    ax_mps2: float  # 三轴滤波后加速度（ENU）
    ay_mps2: float
    az_mps2: float
    ax_rate_mps3: float  # 三轴加速度变化率（二阶滤波内部状态）
    ay_rate_mps3: float
    az_rate_mps3: float
    nx: float  # 空速航迹系 x 前向过载
    ny: float  # 空速航迹系 y 上向过载
    nz: float  # 空速航迹系 z 右向过载
    psi_dot_deg_s: float  # 空速航向角速率（度/秒），左转为正
    wind_east_mps: float = 0.0  # ENU 东向风速
    wind_north_mps: float = 0.0  # ENU 北向风速
    wind_up_mps: float = 0.0  # ENU 天向风速
    ground_psi_dot_deg_s: float = 0.0  # 地面航迹角速率（度/秒），左转为正

    @property
    def theta_deg(self) -> float:
        """返回俯仰角的角度值。注意：内部状态仍以弧度保存。"""
        return math.degrees(self.theta_rad)

    @property
    def psi_v_deg(self) -> float:
        """返回航迹偏角的角度值。注意：内部状态仍以弧度保存。"""
        return math.degrees(self.psi_rad)

    @property
    def phi_deg(self) -> float:
        """返回右倾为正的滚转角。注意：角度由上向/右向过载分量唯一确定。"""
        return math.degrees(self.phi_rad)

    @property
    def phi_rad(self) -> float:
        """返回右倾为正的滚转角弧度值。注意：平飞无侧向载荷时为零。"""
        # atan2(右, 上) 使平飞配平落在 0，向右倾斜时自然得到正角。
        return math.atan2(self.nz, self.ny)

    @property
    def n_normal(self) -> float:
        """返回法向合过载。注意：该标量不是航迹系 z 轴分量。"""
        return math.hypot(self.ny, self.nz)

    @property
    def air_vx_mps(self) -> float:
        """返回空速的 ENU 东向分量。注意：不包含风速。"""
        # 空速球面状态的水平投影先乘 cos(theta)，再按 ENU 航向分解到东轴。
        return self.speed_mps * math.cos(self.theta_rad) * math.cos(self.psi_rad)

    @property
    def air_vy_mps(self) -> float:
        """返回空速的 ENU 北向分量。注意：不包含风速。"""
        # psi 自东向北逆时针为正，因此北向分量使用 sin(psi)，不沿用屏幕 y 轴符号。
        return self.speed_mps * math.cos(self.theta_rad) * math.sin(self.psi_rad)

    @property
    def air_vz_mps(self) -> float:
        """返回空速的 ENU 天向分量。注意：theta>0 表示爬升。"""
        # ENU 第三轴向天为正，与航迹倾角爬升为正的定义直接同号。
        return self.speed_mps * math.sin(self.theta_rad)

    @property
    def ground_vx_mps(self) -> float:
        """返回地速的 ENU 东向分量。注意：等于空速分量加东向风速。"""
        # 风场只改变对地平移，不反向修改积分中的空速航迹方向。
        return self.air_vx_mps + self.wind_east_mps

    @property
    def ground_vy_mps(self) -> float:
        """返回地速的 ENU 北向分量。注意：等于空速分量加北向风速。"""
        # 地面航迹必须由完整地速重新建系，不能把空速 psi 与横风简单拼接。
        return self.air_vy_mps + self.wind_north_mps

    @property
    def ground_vz_mps(self) -> float:
        """返回地速的 ENU 天向分量。注意：等于空速分量加天向风速。"""
        # 天向风同样属于 ENU 地速；算法垂向误差应看到这一分量。
        return self.air_vz_mps + self.wind_up_mps

    @property
    def ground_speed_mps(self) -> float:
        """返回三维地速大小。注意：算法的 vd 仍取水平地速。"""
        return math.sqrt(
            self.ground_vx_mps * self.ground_vx_mps
            + self.ground_vy_mps * self.ground_vy_mps
            + self.ground_vz_mps * self.ground_vz_mps
        )

    @property
    def ground_horizontal_speed_mps(self) -> float:
        """返回水平地速大小。注意：与 VdInEarthS.vd 契约一致。"""
        return math.hypot(self.ground_vx_mps, self.ground_vy_mps)

    @property
    def ground_theta_rad(self) -> float:
        """返回地速航迹倾角。注意：水平地速退化时仍由 atan2 保持有定义。"""
        return math.atan2(self.ground_vz_mps, self.ground_horizontal_speed_mps)

    @property
    def ground_psi_rad(self) -> float:
        """返回地面航迹角。注意：水平地速为零时回退空速航向。"""
        if self.ground_horizontal_speed_mps <= 1e-12:
            return self.psi_rad
        return math.atan2(self.ground_vy_mps, self.ground_vx_mps)

    @property
    def vx_mps(self) -> float:
        """兼容字段：返回 ENU 东向地速。注意：空速请使用 air_vx_mps。"""
        return self.ground_vx_mps

    @property
    def vy_mps(self) -> float:
        """兼容字段：返回 ENU 北向地速。注意：空速请使用 air_vy_mps。"""
        return self.ground_vy_mps

    @property
    def vz_mps(self) -> float:
        """兼容字段：返回 ENU 天向地速。注意：空速请使用 air_vz_mps。"""
        return self.ground_vz_mps

    def as_vector(self) -> list[float]:
        """把状态对象转换为数值向量。注意：向量顺序必须与模型积分约定一致。"""
        # 顺序必须与 derivative/update_from_vector 严格对齐，否则积分会错位。
        return [
            self.x_m,
            self.y_m,
            self.altitude_m,
            self.speed_mps,
            self.theta_rad,
            self.psi_rad,
            self.ax_mps2,
            self.ay_mps2,
            self.az_mps2,
            self.ax_rate_mps3,
            self.ay_rate_mps3,
            self.az_rate_mps3,
        ]

    def update_from_vector(self, vector: Sequence[float]) -> None:
        """用数值向量更新状态对象。注意：调用前应保证向量长度和单位正确。"""
        # 解包顺序须与 as_vector 完全镜像；过载、风速和角速率由后续单独刷新。
        (
            self.x_m,
            self.y_m,
            self.altitude_m,
            self.speed_mps,
            self.theta_rad,
            self.psi_rad,
            self.ax_mps2,
            self.ay_mps2,
            self.az_mps2,
            self.ax_rate_mps3,
            self.ay_rate_mps3,
            self.az_rate_mps3,
        ) = vector


@dataclass(frozen=True)
class PointMassInputs:
    """三自由度质点方程输入。注意：三个分量使用空速航迹系前、上、右。"""

    nx: float  # 切向过载（沿速度方向），驱动加减速
    ny: float  # 上向法向过载，驱动航迹倾角变化
    nz: float  # 右侧向过载，驱动航向角变化

    @property
    def n_normal(self) -> float:
        """返回法向平面的合过载。注意：用于统一载荷包线限幅。"""
        return math.hypot(self.ny, self.nz)

    @property
    def phi_rad(self) -> float:
        """返回右倾为正的滚转角。注意：由上向和右向过载分量派生。"""
        return math.atan2(self.nz, self.ny)


class PointMass3DoFModel:
    """带加速度输入滤波的非线性三自由度质点模型。注意：积分前会做指令限幅。"""

    def __init__(self, config: PointMassModelConfig) -> None:
        """初始化 PointMass3DoFModel 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.config = config

    def acceleration_to_inputs(
        self,
        filtered_accel_mps2: Sequence[float],
        theta_rad: float,
        psi_rad: float,
    ) -> PointMassInputs:
        """把 ENU 加速度指令转换为模型输入量。注意：输出会受载荷和滚转限制约束。"""

        ax, ay, az = (
            self.clamp_acceleration(filtered_accel_mps2[0]),
            self.clamp_acceleration(filtered_accel_mps2[1]),
            self.clamp_acceleration(filtered_accel_mps2[2]),
        )
        # 气动需求力 = 期望加速度 + 抵消重力；竖直方向叠加 +g 表示升力须先平衡重力。
        # 这里仍是 ENU 矢量，只有完成重力补偿后才能投影成无量纲过载。
        force_east = ax
        force_north = ay
        force_up = az + self.config.gravity_mps2

        # 需求力统一投影到苏联式航迹系：x 前、y 上、z 右。
        # 公共变换矩阵同时服务正反变换，避免两套手写公式各自翻一次符号。
        a_forward, a_up, a_right = enu_to_fur(
            (force_east, force_north, force_up),
            fur_basis_from_angles(theta_rad, psi_rad),
        )

        gravity = self.config.gravity_mps2
        # nx：切向加速度归一化为过载并限幅。
        nx = self._clamp(a_forward / gravity, self.config.nx_min, self.config.nx_max)
        # 法向包线约束作用于 y/z 合量，不能再把合量冒充有符号的 z 轴分量。
        # 合量限幅后保留 atan2 给出的方向，左右机动才会保持严格镜像。
        n_normal = self._clamp(
            math.hypot(a_up, a_right) / gravity,
            self.config.n_normal_min,
            self.config.n_normal_max,
        )
        # 滚转角按右倾为正；限幅后再分解，保证 ny/nz 与最终滚转包线严格一致。
        phi_rad = self._clamp(
            math.atan2(a_right, a_up),
            self.config.phi_min_rad,
            self.config.phi_max_rad,
        )
        return PointMassInputs(
            nx=nx,
            ny=n_normal * math.cos(phi_rad),
            nz=n_normal * math.sin(phi_rad),
        )

    def derivative(
        self,
        state: Sequence[float],
        control: Sequence[float],
        wind_velocity_mps: Sequence[float],
    ) -> list[float]:
        """计算三自由度模型状态导数。注意：输入单位必须与模型内部约定一致。"""

        (
            _x_m,
            _y_m,
            _altitude_m,
            speed_mps,
            theta_rad,
            psi_rad,
            ax_mps2,
            ay_mps2,
            az_mps2,
            ax_rate_mps3,
            ay_rate_mps3,
            az_rate_mps3,
        ) = state

        # 速度出现在 theta_dot/psi_dot 分母，低速时钳到 min_speed 防止角速率爆炸。
        speed_for_denom = max(abs(speed_mps), self.config.min_speed_mps)
        cos_theta = math.cos(theta_rad)
        cos_theta_for_denom = self._safe_cos_theta(cos_theta)
        sin_theta = math.sin(theta_rad)

        # 把当前滤波加速度反算为过载/滚转输入，再代入运动学方程。
        inputs = self.acceleration_to_inputs(
            (ax_mps2, ay_mps2, az_mps2),
            theta_rad,
            psi_rad,
        )

        # 对地速度 = 空速球面分解 + 风速；位置导数即对地速度（含风的平移漂移）。
        ground_vx = speed_mps * cos_theta * math.cos(psi_rad) + wind_velocity_mps[0]
        ground_vy = speed_mps * cos_theta * math.sin(psi_rad) + wind_velocity_mps[1]
        ground_vz = speed_mps * sin_theta + wind_velocity_mps[2]

        gravity = self.config.gravity_mps2
        # 标准质点三自由度方程（弧度制）：
        # 切向：nx 提供加速度，减去重力沿速度分量 g·sin(theta)。
        speed_dot = gravity * (inputs.nx - sin_theta)
        # 俯仰率：y 上向过载减去维持当前航迹倾角所需的重力投影。
        theta_dot = gravity / speed_for_denom * (inputs.ny - cos_theta)
        # z 向右为正，而 ENU 航向角向左/逆时针为正，因此二者符号相反。
        # 这一处显式负号取代旧实现中“左向基 + 正号”两次错误相消的偶然正确。
        psi_dot = self.compute_psi_dot_rad_s(
            gravity, inputs.nz, speed_for_denom, cos_theta_for_denom
        )

        # 加速度滤波建模为二阶环节(omega 自然频率, zeta 阻尼)，把阶跃指令平滑成可飞过载。
        omega = self.config.natural_frequency_rad_s
        omega_squared = omega * omega
        damping = 2.0 * self.config.damping_ratio * omega
        cmd_ax, cmd_ay, cmd_az = (
            self.clamp_acceleration(control[0]),
            self.clamp_acceleration(control[1]),
            self.clamp_acceleration(control[2]),
        )

        # 后三项为二阶滤波器导数 a'' = omega^2(cmd-a) - 2·zeta·omega·a'，状态量是加速度及其变化率。
        return [
            ground_vx,
            ground_vy,
            ground_vz,
            speed_dot,
            theta_dot,
            psi_dot,
            ax_rate_mps3,
            ay_rate_mps3,
            az_rate_mps3,
            omega_squared * (cmd_ax - ax_mps2) - damping * ax_rate_mps3,
            omega_squared * (cmd_ay - ay_mps2) - damping * ay_rate_mps3,
            omega_squared * (cmd_az - az_mps2) - damping * az_rate_mps3,
        ]

    def step(
        self,
        state: Sequence[float],
        control: Sequence[float],
        wind_velocity_mps: Sequence[float],
        dt_s: float,
    ) -> list[float]:
        """推进 PointMass3DoFModel 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""

        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        # 经典四阶 Runge-Kutta：取四组斜率(头、两次中点、尾)加权平均推进一步，精度优于欧拉法。
        # 同一控制指令在整步内保持不变（零阶保持）。
        k1 = self.derivative(state, control, wind_velocity_mps)
        k2 = self.derivative(self._add_scaled(state, k1, dt_s * 0.5), control, wind_velocity_mps)
        k3 = self.derivative(self._add_scaled(state, k2, dt_s * 0.5), control, wind_velocity_mps)
        k4 = self.derivative(self._add_scaled(state, k3, dt_s), control, wind_velocity_mps)
        # 加权 (k1 + 2k2 + 2k3 + k4)/6 即 RK4 增量公式。
        stepped = [
            value + dt_s * (k1_value + 2.0 * k2_value + 2.0 * k3_value + k4_value) / 6.0
            for value, k1_value, k2_value, k3_value, k4_value in zip(state, k1, k2, k3, k4)
        ]
        return self._clamp_state_vector(stepped)

    def _clamp_state_vector(self, vector: list[float]) -> list[float]:
        """限制状态向量中的关键物理量。注意：用于防止积分过程产生非法姿态或速度。"""
        vector[3] = max(self.config.min_speed_mps, vector[3])  # 速度不低于最小可飞速度
        vector[4] = self._clamp(vector[4], -_MAX_ABS_THETA_RAD, _MAX_ABS_THETA_RAD)  # 俯仰限幅
        vector[4] = self._clamp_theta_for_vertical_rate(vector[3], vector[4])  # 爬升/下沉率包线
        vector[5] = math.atan2(math.sin(vector[5]), math.cos(vector[5]))  # 航向归一化到 (-pi, pi]
        # 三轴滤波加速度饱和；一旦触限就把对应变化率清零，避免抗饱和失效后状态持续冲出。
        for acc_idx, rate_idx in ((6, 9), (7, 10), (8, 11)):
            original = vector[acc_idx]
            vector[acc_idx] = self.clamp_acceleration(original)
            if vector[acc_idx] != original:
                vector[rate_idx] = 0.0
        return vector

    def clamp_acceleration(self, value: float) -> float:
        """限制 ENU 加速度指令幅值。注意：饱和后可能改变跟踪误差收敛速度。"""
        limit = self.config.acceleration_command_limit_mps2
        return self._clamp(float(value), -limit, limit)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        """按上下限裁剪单个数值。注意：作为模型内部工具函数使用。"""
        return max(lower, min(upper, value))

    @staticmethod
    def _safe_cos_theta(cos_theta: float) -> float:
        """计算安全的俯仰角余弦值。注意：避免接近零导致除法放大。"""
        # 保留符号地把 |cos| 抬到 eps 以上，防止 psi_dot 分母趋零放大数值误差。
        if abs(cos_theta) >= _COS_THETA_EPS:
            return cos_theta
        return _COS_THETA_EPS if cos_theta >= 0.0 else -_COS_THETA_EPS

    @staticmethod
    def compute_psi_dot_rad_s(
        gravity: float,
        nz: float,
        speed: float,
        cos_theta: float,
    ) -> float:
        """由右向过载计算航向角速率。注意：航向角左转为正，故与 nz 异号。"""
        return -gravity * nz / (speed * cos_theta)

    def _clamp_theta_for_vertical_rate(self, speed_mps: float, theta_rad: float) -> float:
        """按最大爬升/下沉率限制航迹倾角。注意：保持速度标量不变，仅裁剪垂向分量。"""
        speed = max(abs(speed_mps), self.config.min_speed_mps)
        lower_vz = -min(self.config.max_descent_rate_mps, speed)
        upper_vz = min(self.config.max_climb_rate_mps, speed)
        vz_mps = speed * math.sin(theta_rad)
        clamped_vz = self._clamp(vz_mps, lower_vz, upper_vz)
        return math.asin(self._clamp(clamped_vz / speed, -1.0, 1.0))

    @staticmethod
    def _add_scaled(left: Sequence[float], right: Sequence[float], scale: float) -> list[float]:
        """按比例叠加两个向量。注意：用于 RK4 积分中间状态计算。"""
        # 计算 left + scale*right，用于由斜率推出 RK4 中间评估点。
        return [left_value + right_value * scale for left_value, right_value in zip(left, right)]


class ModelIterator:
    """持有并推进全部无人机质点模型实例。注意：reset 会恢复配置初始状态。"""

    DEFAULT_GRAVITY_MPS2 = 9.80665  # 标准重力加速度
    DEFAULT_MIN_SPEED_MPS = 1.0
    DEFAULT_NATURAL_FREQUENCY_RAD_S = 4.0  # 加速度滤波带宽，越大跟踪越快但越接近裸指令
    DEFAULT_DAMPING_RATIO = 0.65  # 欠阻尼，兼顾响应速度与超调
    DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2 = 6.0  # 约 0.6g 的指令幅值上限
    DEFAULT_MAX_CLIMB_RATE_MPS = 8.0  # 最大爬升率，限制垂向速度继续增大
    DEFAULT_MAX_DESCENT_RATE_MPS = 8.0  # 最大下沉率，限制垂向速度继续增大
    DEFAULT_NX_MIN = -1.0
    DEFAULT_NX_MAX = 1.0
    DEFAULT_N_NORMAL_MIN = 0.0
    DEFAULT_N_NORMAL_MAX = 4.0  # 最大法向合过载 4g
    DEFAULT_PHI_MIN_DEG = -70.0  # 最大滚转 ±70°，限制转弯/机动能力
    DEFAULT_PHI_MAX_DEG = 70.0

    def __init__(self) -> None:
        """初始化 ModelIterator 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._config = self._default_config()
        self._system = PointMass3DoFModel(self._config)
        self._states: dict[str, AircraftState] = {}
        self._initial_states: dict[str, AircraftState] = {}
        self._controls: dict[str, AccelerationCommand] = {}
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        self._time_s = 0.0

    def init(self, config: dict[str, object], seed: int) -> None:
        """按配置初始化 ModelIterator。注意：调用方需先准备好必要依赖和输入数据。"""

        del seed  # 本模型确定性，无需随机种子
        self._config = self._parse_model_config(config.get("model", {}))
        self._system = PointMass3DoFModel(self._config)
        self._time_s = 0.0
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        self._states = {}

        nodes = config.get("nodes", [])
        if nodes is None:
            nodes = []
        if not isinstance(nodes, list):
            raise ValueError("nodes must be a list")
        for index, node in enumerate(nodes):
            if not isinstance(node, dict):
                raise ValueError("each node must be an object")
            state = self._make_initial_state(node, index)
            if state.node_id in self._states:
                raise ValueError(f"duplicate node_id: {state.node_id}")
            self._states[state.node_id] = state

        # 深拷贝一份初始状态用于 reset；replace 生成独立副本，避免运行期改动污染基线。
        self._initial_states = {
            node_id: replace(state)
            for node_id, state in self._states.items()
        }
        self._controls = {
            node_id: AccelerationCommand()
            for node_id in self._states
        }

    def read_states(self) -> dict[str, AircraftState]:
        """读取所有飞机的状态副本。注意：返回值供外部读取，不能作为内部状态引用使用。"""

        # 返回副本，防止外部读取者意外修改内部状态。
        return {
            node_id: replace(state)
            for node_id, state in self._states.items()
        }

    def apply_controls(self, controls: Mapping[str, AccelerationCommand]) -> None:
        """应用各节点控制指令。注意：缺省节点会保持上一拍或零指令。"""

        for node_id, control in controls.items():
            if node_id not in self._states:
                continue  # 忽略未知节点指令
            # 入口处先对指令限幅，保证存入的指令已落在可飞包线内。
            self._controls[node_id] = AccelerationCommand(
                self._system.clamp_acceleration(control.ax_cmd_mps2),
                self._system.clamp_acceleration(control.ay_cmd_mps2),
                self._system.clamp_acceleration(control.az_cmd_mps2),
            )

    def step(self, dt_s: float) -> None:
        """推进 ModelIterator 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""

        # 逐机独立积分一步（机间无耦合）；缺省指令视为零加速度。
        for node_id, state in self._states.items():
            control = self._controls.get(node_id, AccelerationCommand())
            vector = self._system.step(
                state.as_vector(),
                control.as_vector(),
                self._wind_velocity_mps,
                dt_s,
            )
            state.update_from_vector(vector)
            # 积分后刷新展示用过载/滚转/偏航角速率，保持与新加速度同步。
            self._update_inputs_from_filtered_acceleration(state)
        self._time_s += dt_s

    def tick(self, dt_s: float) -> None:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""

        self.step(dt_s)  # tick 与 step 等价，提供统一生命周期接口名

    def inject_wind(self, command: object) -> None:
        """注入恒定风场扰动。注意：风速单位为米每秒。"""

        params = self._command_params(command)
        speed_mps = float(params.get("speed_mps", 0.0))
        direction_deg = float(params.get("direction_deg", 0.0))
        vertical_mps = float(params.get("vertical_mps", 0.0))
        if (
            not math.isfinite(speed_mps)
            or speed_mps < 0.0
            or not math.isfinite(direction_deg)
            or not math.isfinite(vertical_mps)
        ):
            raise ValueError(
                "wind params require finite speed_mps >= 0, direction_deg and vertical_mps"
            )
        direction_rad = math.radians(direction_deg)
        # 风方向角按 ENU：0° 指东、90° 指北；水平风速分解到东/北，竖直分量单独给出。
        self._wind_velocity_mps = (
            speed_mps * math.cos(direction_rad),
            speed_mps * math.sin(direction_rad),
            vertical_mps,
        )
        self._sync_wind_to_states()

    def clear_wind(self) -> None:
        """清除当前风场扰动。注意：不会重置飞机运动状态。"""

        self._wind_velocity_mps = (0.0, 0.0, 0.0)  # 风速归零，飞行状态不动
        self._sync_wind_to_states()

    def reset(self) -> None:
        """复位 ModelIterator 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""

        self._time_s = 0.0
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        # 从初始基线深拷贝恢复状态，控制指令清零。
        self._states = {
            node_id: replace(state)
            for node_id, state in self._initial_states.items()
        }
        self._controls = {
            node_id: AccelerationCommand()
            for node_id in self._states
        }

    def close(self) -> None:
        """释放 ModelIterator 持有的资源。注意：关闭后不应继续调用运行接口。"""
        # 清空全部容器；本模型无外部句柄/文件，置空即可。
        self._states.clear()
        self._initial_states.clear()
        self._controls.clear()

    def _make_initial_state(self, node: dict[str, object], index: int) -> AircraftState:
        """根据节点配置构造初始飞机状态。注意：配置缺省值需与 base.json 约定一致。"""
        node_id = node_id_from_config(node, index)

        spherical_keys = {"speed_mps", "theta_deg", "psi_v_deg"}
        component_keys = {"vx_mps", "vy_mps", "vz_mps", "climb_rate_mps"}
        has_spherical = any(key in node for key in spherical_keys)
        has_components = any(key in node for key in component_keys)
        if has_spherical and has_components:
            raise ValueError(
                f"node {node_id!r} velocity representations cannot mix "
                "speed/theta/psi with ENU velocity components"
            )
        if has_components:
            # 分量表示至少要给东、北；天向可省略为 0。禁止 vz 与历史别名同时出现，避免双重权威。
            if "vx_mps" not in node or "vy_mps" not in node:
                raise ValueError(
                    f"node {node_id!r} ENU velocity representation requires vx_mps and vy_mps"
                )
            if "vz_mps" in node and "climb_rate_mps" in node:
                raise ValueError(
                    f"node {node_id!r} velocity representations cannot provide both "
                    "vz_mps and climb_rate_mps"
                )
            vx_mps = float(node["vx_mps"])
            vy_mps = float(node["vy_mps"])
            vz_mps = float(node.get("vz_mps", node.get("climb_rate_mps", 0.0)))
            if not all(math.isfinite(value) for value in (vx_mps, vy_mps, vz_mps)):
                raise ValueError(f"node {node_id!r} ENU velocity components must be finite")
            horizontal_speed_mps = math.hypot(vx_mps, vy_mps)
            if horizontal_speed_mps <= 1e-9:
                raise ValueError(
                    f"node {node_id!r} ENU velocity requires non-zero horizontal speed"
                )
            speed_mps = math.sqrt(
                vx_mps * vx_mps + vy_mps * vy_mps + vz_mps * vz_mps
            )
            if speed_mps < self._config.min_speed_mps:
                raise ValueError(
                    f"node {node_id!r} ENU velocity magnitude must be >= model.min_speed_mps"
                )
            theta_rad = math.atan2(vz_mps, horizontal_speed_mps)
            psi_rad = math.atan2(vy_mps, vx_mps)
        else:
            speed_mps = float(node.get("speed_mps", 5.0))
            theta_rad = math.radians(float(node.get("theta_deg", 0.0)))
            psi_rad = math.radians(float(node.get("psi_v_deg", 0.0)))
            if not all(math.isfinite(value) for value in (speed_mps, theta_rad, psi_rad)):
                raise ValueError(f"node {node_id!r} speed/theta/psi must be finite")
            speed_mps = max(self._config.min_speed_mps, speed_mps)
        theta_rad = self._system._clamp_theta_for_vertical_rate(speed_mps, theta_rad)

        # 初始加速度优先取显式 ENU 值，否则由过载/滚转配平反算。
        if any(key in node for key in ("ax_mps2", "ay_mps2", "az_mps2")):
            ax_mps2 = float(node.get("ax_mps2", 0.0))
            ay_mps2 = float(node.get("ay_mps2", 0.0))
            az_mps2 = float(node.get("az_mps2", 0.0))
            if not all(math.isfinite(value) for value in (ax_mps2, ay_mps2, az_mps2)):
                raise ValueError(f"node {node_id!r} initial ENU acceleration must be finite")
        else:
            # 未给 ENU 加速度时，支持 FUR 三分量或“法向合量+滚转角”二选一。
            nx = float(node.get("nx", math.sin(theta_rad)))
            has_axis_load = any(key in node for key in ("ny", "nz"))
            has_polar_load = any(key in node for key in ("n_normal", "phi_deg"))
            if has_axis_load and has_polar_load:
                raise ValueError(
                    f"node {node_id!r} load representations cannot mix ny/nz with n_normal/phi_deg"
                )
            if has_polar_load:
                n_normal = float(node.get("n_normal", math.cos(theta_rad)))
                phi_rad = math.radians(float(node.get("phi_deg", 0.0)))
                if not math.isfinite(n_normal) or n_normal < 0.0 or not math.isfinite(phi_rad):
                    raise ValueError(
                        f"node {node_id!r} n_normal must be finite and non-negative, "
                        "and phi_deg must be finite"
                    )
                ny = n_normal * math.cos(phi_rad)
                nz = n_normal * math.sin(phi_rad)
            else:
                ny = float(node.get("ny", math.cos(theta_rad)))
                nz = float(node.get("nz", 0.0))
            if not all(math.isfinite(value) for value in (nx, ny, nz)):
                raise ValueError(f"node {node_id!r} initial FUR load components must be finite")
            ax_mps2, ay_mps2, az_mps2 = self._inputs_to_enu_acceleration(
                nx,
                ny,
                nz,
                theta_rad,
                psi_rad,
            )

        state = AircraftState(
            node_id=node_id,
            # 缺省初始队形：沿东向每架后退 45m、南北错开、阶梯抬高 15m，避免初始重叠。
            x_m=float(node.get("x_m", index * -45.0)),
            y_m=float(node.get("y_m", 0.0 if index == 0 else (index * 2 - 3) * 50.0)),
            altitude_m=float(node.get("altitude_m", 1200.0 + index * 15.0)),
            speed_mps=speed_mps,
            theta_rad=theta_rad,
            psi_rad=psi_rad,
            ax_mps2=ax_mps2,
            ay_mps2=ay_mps2,
            az_mps2=az_mps2,
            ax_rate_mps3=float(node.get("ax_rate_mps3", 0.0)),
            ay_rate_mps3=float(node.get("ay_rate_mps3", 0.0)),
            az_rate_mps3=float(node.get("az_rate_mps3", 0.0)),
            nx=0.0,
            ny=0.0,
            nz=0.0,
            psi_dot_deg_s=0.0,
        )
        self._update_inputs_from_filtered_acceleration(state)
        return state

    def _update_inputs_from_filtered_acceleration(self, state: AircraftState) -> None:
        """根据滤波后的加速度更新模型输入。注意：保持输入与状态量同步。"""
        # 由当前滤波加速度+姿态反算过载/滚转，写回展示字段（不影响积分状态）。
        inputs = self._system.acceleration_to_inputs(
            (state.ax_mps2, state.ay_mps2, state.az_mps2),
            state.theta_rad,
            state.psi_rad,
        )
        state.nx = inputs.nx
        state.ny = inputs.ny
        state.nz = inputs.nz
        cos_theta_safe = PointMass3DoFModel._safe_cos_theta(math.cos(state.theta_rad))
        speed_safe = max(self._config.min_speed_mps, state.speed_mps)
        state.psi_dot_deg_s = math.degrees(
            PointMass3DoFModel.compute_psi_dot_rad_s(
                self._config.gravity_mps2,
                inputs.nz,
                speed_safe,
                cos_theta_safe,
            )
        )
        state.ground_psi_dot_deg_s = math.degrees(
            self._ground_psi_dot_rad_s(state, inputs)
        )

    def _sync_wind_to_states(self) -> None:
        """把当前 ENU 风矢量同步到飞机派生状态。注意：不修改空速积分状态。"""
        wind_east, wind_north, wind_up = self._wind_velocity_mps
        for state in self._states.values():
            state.wind_east_mps = wind_east
            state.wind_north_mps = wind_north
            state.wind_up_mps = wind_up
            # 横风会立即改变地面航迹角及其角速率，事件注入当拍就要刷新。
            self._update_inputs_from_filtered_acceleration(state)

    def _ground_psi_dot_rad_s(
        self,
        state: AircraftState,
        inputs: PointMassInputs,
    ) -> float:
        """计算恒定风场下的地面航迹角速率。注意：地速退化时返回零。"""
        gravity = self._config.gravity_mps2
        speed = state.speed_mps
        speed_safe = max(self._config.min_speed_mps, speed)
        cos_theta = math.cos(state.theta_rad)
        sin_theta = math.sin(state.theta_rad)
        cos_theta_safe = PointMass3DoFModel._safe_cos_theta(cos_theta)
        speed_dot = gravity * (inputs.nx - sin_theta)
        theta_dot = gravity / speed_safe * (inputs.ny - cos_theta)
        air_psi_dot = PointMass3DoFModel.compute_psi_dot_rad_s(
            gravity,
            inputs.nz,
            speed_safe,
            cos_theta_safe,
        )

        air_horizontal_speed = speed * cos_theta
        air_horizontal_accel = (
            speed_dot * cos_theta - speed * sin_theta * theta_dot
        )
        cos_psi = math.cos(state.psi_rad)
        sin_psi = math.sin(state.psi_rad)
        ground_east = air_horizontal_speed * cos_psi + state.wind_east_mps
        ground_north = air_horizontal_speed * sin_psi + state.wind_north_mps
        accel_east = (
            air_horizontal_accel * cos_psi
            - air_horizontal_speed * sin_psi * air_psi_dot
        )
        accel_north = (
            air_horizontal_accel * sin_psi
            + air_horizontal_speed * cos_psi * air_psi_dot
        )
        ground_horizontal_speed_sq = (
            ground_east * ground_east + ground_north * ground_north
        )
        if ground_horizontal_speed_sq <= 1e-12:
            return 0.0
        return (
            ground_east * accel_north - ground_north * accel_east
        ) / ground_horizontal_speed_sq

    def _inputs_to_enu_acceleration(
        self,
        nx: float,
        ny: float,
        nz: float,
        theta_rad: float,
        psi_rad: float,
    ) -> tuple[float, float, float]:
        """把 FUR 三轴过载反算为 ENU 加速度。注意：用于初始配平与回环测试。"""
        gravity = self._config.gravity_mps2
        force_east, force_north, force_up = fur_to_enu(
            (gravity * nx, gravity * ny, gravity * nz),
            fur_basis_from_angles(theta_rad, psi_rad),
        )
        # acceleration_to_inputs 投影的是“净加速度 + 抵消重力”，故逆变换最后减去重力。
        return (force_east, force_north, force_up - gravity)

    @classmethod
    def _default_config(cls) -> PointMassModelConfig:
        """生成模型默认配置。注意：只在外部配置缺省时兜底使用。"""
        return PointMassModelConfig(
            gravity_mps2=cls.DEFAULT_GRAVITY_MPS2,
            min_speed_mps=cls.DEFAULT_MIN_SPEED_MPS,
            natural_frequency_rad_s=cls.DEFAULT_NATURAL_FREQUENCY_RAD_S,
            damping_ratio=cls.DEFAULT_DAMPING_RATIO,
            acceleration_command_limit_mps2=cls.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2,
            max_climb_rate_mps=cls.DEFAULT_MAX_CLIMB_RATE_MPS,
            max_descent_rate_mps=cls.DEFAULT_MAX_DESCENT_RATE_MPS,
            nx_min=cls.DEFAULT_NX_MIN,
            nx_max=cls.DEFAULT_NX_MAX,
            n_normal_min=cls.DEFAULT_N_NORMAL_MIN,
            n_normal_max=cls.DEFAULT_N_NORMAL_MAX,
            phi_min_rad=math.radians(cls.DEFAULT_PHI_MIN_DEG),
            phi_max_rad=math.radians(cls.DEFAULT_PHI_MAX_DEG),
        )

    @classmethod
    def _parse_model_config(cls, raw_config: object) -> PointMassModelConfig:
        """解析模型配置并合并默认值。注意：字段单位需与配置文档一致。"""
        if raw_config is None:
            raw_config = {}
        if not isinstance(raw_config, Mapping):
            raise ValueError("model must be an object")

        filter_config = raw_config.get("acceleration_filter", {})
        if filter_config is None:
            filter_config = {}
        if not isinstance(filter_config, Mapping):
            raise ValueError("model.acceleration_filter must be an object")

        limits_config = raw_config.get("limits", {})
        if limits_config is None:
            limits_config = {}
        if not isinstance(limits_config, Mapping):
            raise ValueError("model.limits must be an object")
        if "nz_min" in limits_config or "nz_max" in limits_config:
            raise ValueError(
                "model.limits.nz_min/nz_max used the old normal-load magnitude meaning; "
                "rename them to n_normal_min/n_normal_max"
            )

        # 取值优先级：嵌套子配置 > 顶层旧字段 > 类默认值，兼容新旧配置写法。
        natural_frequency = float(
            filter_config.get(
                "natural_frequency_rad_s",
                raw_config.get("natural_frequency_rad_s", cls.DEFAULT_NATURAL_FREQUENCY_RAD_S),
            )
        )
        # 阻尼比同样遵循子配置 > 顶层 > 默认的优先级。
        damping_ratio = float(
            filter_config.get(
                "damping_ratio",
                raw_config.get("damping_ratio", cls.DEFAULT_DAMPING_RATIO),
            )
        )
        # 加速度限幅：新键 limits.acceleration_command_mps2，回退旧键 max_acceleration_mps2。
        acceleration_limit = float(
            limits_config.get(
                "acceleration_command_mps2",
                raw_config.get("max_acceleration_mps2", cls.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2),
            )
        )
        gravity = float(raw_config.get("gravity_mps2", cls.DEFAULT_GRAVITY_MPS2))
        min_speed = float(raw_config.get("min_speed_mps", cls.DEFAULT_MIN_SPEED_MPS))

        config = PointMassModelConfig(
            gravity_mps2=gravity,
            min_speed_mps=min_speed,
            natural_frequency_rad_s=natural_frequency,
            damping_ratio=damping_ratio,
            acceleration_command_limit_mps2=acceleration_limit,
            max_climb_rate_mps=float(limits_config.get("max_climb_rate_mps", cls.DEFAULT_MAX_CLIMB_RATE_MPS)),
            max_descent_rate_mps=float(limits_config.get("max_descent_rate_mps", cls.DEFAULT_MAX_DESCENT_RATE_MPS)),
            nx_min=float(limits_config.get("nx_min", cls.DEFAULT_NX_MIN)),
            nx_max=float(limits_config.get("nx_max", cls.DEFAULT_NX_MAX)),
            n_normal_min=float(
                limits_config.get("n_normal_min", cls.DEFAULT_N_NORMAL_MIN)
            ),
            n_normal_max=float(
                limits_config.get("n_normal_max", cls.DEFAULT_N_NORMAL_MAX)
            ),
            phi_min_rad=math.radians(float(limits_config.get("phi_min_deg", cls.DEFAULT_PHI_MIN_DEG))),
            phi_max_rad=math.radians(float(limits_config.get("phi_max_deg", cls.DEFAULT_PHI_MAX_DEG))),
        )
        cls._validate_config(config)  # 构造后立即校验，非法配置直接报错
        return config

    @staticmethod
    def _validate_config(config: PointMassModelConfig) -> None:
        """校验模型配置合法性。注意：失败应阻止仿真继续运行。"""
        if not all(
            math.isfinite(value)
            for value in (
                config.gravity_mps2,
                config.min_speed_mps,
                config.natural_frequency_rad_s,
                config.damping_ratio,
                config.acceleration_command_limit_mps2,
                config.max_climb_rate_mps,
                config.max_descent_rate_mps,
                config.nx_min,
                config.nx_max,
                config.n_normal_min,
                config.n_normal_max,
                config.phi_min_rad,
                config.phi_max_rad,
            )
        ):
            raise ValueError("model numeric configuration must be finite")
        if config.gravity_mps2 <= 0.0:
            raise ValueError("model.gravity_mps2 must be positive")
        if config.min_speed_mps <= 0.0:
            raise ValueError("model.min_speed_mps must be positive")
        if config.natural_frequency_rad_s <= 0.0:
            raise ValueError("model.acceleration_filter.natural_frequency_rad_s must be positive")
        # 阻尼比须在 (0,1) 之间，保证加速度滤波是欠阻尼二阶环节（有上升不发散）。
        if not 0.0 < config.damping_ratio < 1.0:
            raise ValueError(
                "model.acceleration_filter.damping_ratio must be in (0, 1) "
                "for underdamped response"
            )
        if config.acceleration_command_limit_mps2 <= 0.0:
            raise ValueError("model.limits.acceleration_command_mps2 must be positive")
        if config.max_climb_rate_mps <= 0.0:
            raise ValueError("model.limits.max_climb_rate_mps must be positive")
        if config.max_descent_rate_mps <= 0.0:
            raise ValueError("model.limits.max_descent_rate_mps must be positive")
        if config.nx_min >= config.nx_max:
            raise ValueError("model.limits.nx_min must be less than nx_max")
        # n_normal 由 hypot 得出恒非负，故下限必须 >=0 且严格小于上限。
        if config.n_normal_min < 0.0 or config.n_normal_min >= config.n_normal_max:
            raise ValueError(
                "model.limits.n_normal_min must be non-negative and less than n_normal_max"
            )
        if config.phi_min_rad >= config.phi_max_rad:
            raise ValueError("model.limits.phi_min_deg must be less than phi_max_deg")
        # 下面三条保证过载/滚转区间覆盖平飞配平点 (nx=0, ny=1, nz=0, phi=0)。
        if config.n_normal_max < 1.0:
            raise ValueError(
                "model.limits.n_normal_max must be >= 1.0 to allow level-flight trim"
            )
        if config.n_normal_min > 1.0:
            raise ValueError(
                "model.limits.n_normal range must include 1.0 to allow level-flight trim"
            )
        if config.nx_min > 0.0 or config.nx_max < 0.0:
            raise ValueError("model.limits.nx range must include 0.0 to allow level-flight trim (nx=0)")
        if config.phi_min_rad > 0.0 or config.phi_max_rad < 0.0:
            raise ValueError("model.limits.phi range must include 0.0 to allow straight flight (phi=0)")

    @staticmethod
    def _command_params(command: object) -> Mapping[str, object]:
        """读取指令限制参数。注意：缺省值来自模型配置。"""
        # 兼容字典与对象两种指令载体；非映射类型回退为空参数。
        params = ModelIterator._command_value(command, "params")
        return params if isinstance(params, Mapping) else {}

    @staticmethod
    def _command_value(command: object, name: str) -> object:
        """读取单个指令参数值。注意：非法数值会回退或抛错取决于调用场景。"""
        # 字典用 get、对象用 getattr，统一读取指令字段。
        if isinstance(command, Mapping):
            return command.get(name)
        return getattr(command, name, None)
