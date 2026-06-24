"""ENU 坐标系下的无人机三自由度质点模型。注意：内部角度使用弧度。"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace


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
    nz_min: float  # 法向过载下限（恒非负）
    nz_max: float  # 法向过载上限
    phi_min_rad: float  # 滚转角下限
    phi_max_rad: float  # 滚转角上限


@dataclass
class AircraftState:
    """单架飞行器完整状态：ENU 位置/高度、速度、航迹角(弧度)及三轴滤波加速度与变化率。

    nx/nz/phi_rad/psi_dot_deg_s 为由滤波加速度反算出的展示派生量，不进入 12 维积分状态向量。
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
    nx: float  # 反算展示量：切向过载
    nz: float  # 反算展示量：法向过载
    phi_rad: float  # 反算展示量：滚转角
    psi_dot_deg_s: float  # 反算展示量：航迹偏航角速率（度/秒），左转为正

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
        """返回滚转角的角度值。注意：内部状态仍以弧度保存。"""
        return math.degrees(self.phi_rad)

    @property
    def vx_mps(self) -> float:
        """返回东向速度分量。注意：单位为米每秒。"""
        # 速度按航迹倾角 theta、航向 psi 球面分解；水平分量先乘 cos(theta) 再投到东向。
        return self.speed_mps * math.cos(self.theta_rad) * math.cos(self.psi_rad)

    @property
    def vy_mps(self) -> float:
        """返回北向速度分量。注意：单位为米每秒。"""
        return self.speed_mps * math.cos(self.theta_rad) * math.sin(self.psi_rad)

    @property
    def vz_mps(self) -> float:
        """返回天向速度分量。注意：单位为米每秒。"""
        # 垂向分量只取航迹倾角的正弦，theta>0 表示爬升。
        return self.speed_mps * math.sin(self.theta_rad)

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
        # 解包顺序须与 as_vector 完全镜像；nx/nz/phi_rad/psi_dot_deg_s 不在向量中，由后续单独刷新。
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
    """三自由度质点方程输入：切向过载 nx、法向过载 nz、滚转角 phi(弧度)。"""

    nx: float  # 切向过载（沿速度方向），驱动加减速
    nz: float  # 法向过载（垂直速度方向），与 phi 一起决定转弯/爬升
    phi_rad: float  # 速度滚转角，决定法向过载在俯仰/偏航平面的分配


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
        force_east = ax
        force_north = ay
        force_up = az + self.config.gravity_mps2

        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_psi = math.cos(psi_rad)
        sin_psi = math.sin(psi_rad)

        # 把 ENU 需求力旋到航迹坐标系：a_v 沿速度、a_theta 在竖直平面内法向、a_psi 在水平面内侧向。
        a_v = (
            cos_theta * cos_psi * force_east
            + cos_theta * sin_psi * force_north
            + sin_theta * force_up
        )
        a_theta = (
            -sin_theta * cos_psi * force_east
            - sin_theta * sin_psi * force_north
            + cos_theta * force_up
        )
        a_psi = -sin_psi * force_east + cos_psi * force_north

        gravity = self.config.gravity_mps2
        # nx：切向加速度归一化为过载并限幅。
        nx = self._clamp(a_v / gravity, self.config.nx_min, self.config.nx_max)
        # nz：法向需求力的合幅值(总在垂直速度的平面内)归一化为过载，恒非负。
        nz = self._clamp(
            math.hypot(a_theta, a_psi) / gravity,
            self.config.nz_min,
            self.config.nz_max,
        )
        # phi：法向力的方位角，决定 nz 在爬升与转弯之间如何分配；限幅模拟滚转能力上限。
        phi_rad = self._clamp(
            math.atan2(a_psi, a_theta),
            self.config.phi_min_rad,
            self.config.phi_max_rad,
        )
        return PointMassInputs(nx, nz, phi_rad)

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
        # 俯仰率：法向力竖直分量 nz·cos(phi) 减去维持平飞所需的 cos(theta)，再除以速度。
        theta_dot = (
            gravity
            / speed_for_denom
            * (inputs.nz * math.cos(inputs.phi_rad) - cos_theta)
        )
        # 航向率：法向力水平分量 nz·sin(phi) 转弯，除以水平速度 (speed·cos_theta)。
        psi_dot = self.compute_psi_dot_rad_s(
            gravity, inputs.nz, inputs.phi_rad, speed_for_denom, cos_theta_for_denom
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
        phi_rad: float,
        speed: float,
        cos_theta: float,
    ) -> float:
        """计算航向角速率（弧度/秒）。注意：speed 和 cos_theta 须已完成奇异点保护。"""
        return gravity * nz * math.sin(phi_rad) / (speed * cos_theta)

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
    DEFAULT_NZ_MIN = 0.0
    DEFAULT_NZ_MAX = 4.0  # 最大法向过载 4g
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
        direction_rad = math.radians(float(params.get("direction_deg", 0.0)))
        vertical_mps = float(params.get("vertical_mps", 0.0))
        # 风方向角按 ENU：0° 指东、90° 指北；水平风速分解到东/北，竖直分量单独给出。
        self._wind_velocity_mps = (
            speed_mps * math.cos(direction_rad),
            speed_mps * math.sin(direction_rad),
            vertical_mps,
        )

    def clear_wind(self) -> None:
        """清除当前风场扰动。注意：不会重置飞机运动状态。"""

        self._wind_velocity_mps = (0.0, 0.0, 0.0)  # 风速归零，飞行状态不动

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

        speed_mps = max(self._config.min_speed_mps, float(node.get("speed_mps", 5.0)))

        # 配置角度以度给出，内部统一转弧度。
        theta_rad = math.radians(float(node.get("theta_deg", 0.0)))
        psi_rad = math.radians(float(node.get("psi_v_deg", 0.0)))
        has_velocity_components = any(
            key in node
            for key in ("vx_mps", "vy_mps", "vz_mps", "climb_rate_mps")
        )
        if has_velocity_components:
            # 若配置直接给速度分量，则由分量反推空速与航迹角，覆盖角度缺省值。
            default_vx = speed_mps * math.cos(theta_rad) * math.cos(psi_rad)
            default_vy = speed_mps * math.cos(theta_rad) * math.sin(psi_rad)
            default_vz = speed_mps * math.sin(theta_rad)
            vx_mps = float(node.get("vx_mps", default_vx))
            vy_mps = float(node.get("vy_mps", default_vy))
            vz_mps = float(node.get("vz_mps", node.get("climb_rate_mps", default_vz)))
            speed_mps = max(self._config.min_speed_mps, math.sqrt(vx_mps * vx_mps + vy_mps * vy_mps + vz_mps * vz_mps))
            # theta = asin(vz/V)，clamp 防止浮点越界导致 asin 定义域错误。
            if "theta_deg" not in node:
                theta_rad = max(-_MAX_ABS_THETA_RAD, min(_MAX_ABS_THETA_RAD,
                    math.asin(max(-1.0, min(1.0, vz_mps / speed_mps)))))
            # 仅当水平速度足够大时才由分量定航向，避免接近垂直/悬停时 atan2 抖动。
            if "psi_v_deg" not in node and math.hypot(vx_mps, vy_mps) > 1e-9:
                psi_rad = math.atan2(vy_mps, vx_mps)
        theta_rad = self._system._clamp_theta_for_vertical_rate(speed_mps, theta_rad)

        # 初始加速度优先取显式 ENU 值，否则由过载/滚转配平反算。
        if any(key in node for key in ("ax_mps2", "ay_mps2", "az_mps2")):
            ax_mps2 = float(node.get("ax_mps2", 0.0))
            ay_mps2 = float(node.get("ay_mps2", 0.0))
            az_mps2 = float(node.get("az_mps2", 0.0))
        else:
            # 未给 ENU 加速度时，用过载/滚转描述初始配平：平飞配平 nx=sin(theta)、nz=cos(theta)。
            nx = float(node.get("nx", math.sin(theta_rad)))
            nz = float(node.get("nz", math.cos(theta_rad)))
            phi_rad = math.radians(float(node.get("phi_deg", 0.0)))
            ax_mps2, ay_mps2, az_mps2 = self._inputs_to_enu_acceleration(
                nx,
                nz,
                phi_rad,
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
            nz=0.0,
            phi_rad=0.0,
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
        state.nz = inputs.nz
        state.phi_rad = inputs.phi_rad
        cos_theta_safe = PointMass3DoFModel._safe_cos_theta(math.cos(state.theta_rad))
        speed_safe = max(self._config.min_speed_mps, state.speed_mps)
        state.psi_dot_deg_s = math.degrees(
            PointMass3DoFModel.compute_psi_dot_rad_s(
                self._config.gravity_mps2, inputs.nz, inputs.phi_rad,
                speed_safe, cos_theta_safe,
            )
        )

    def _inputs_to_enu_acceleration(
        self,
        nx: float,
        nz: float,
        phi_rad: float,
        theta_rad: float,
        psi_rad: float,
    ) -> tuple[float, float, float]:
        """把模型输入反算为 ENU 加速度。注意：用于快照和调试显示。"""
        # acceleration_to_inputs 的逆变换：先把过载还原为航迹系三轴力。
        gravity = self._config.gravity_mps2
        a_v = gravity * nx
        a_theta = gravity * nz * math.cos(phi_rad)
        a_psi = gravity * nz * math.sin(phi_rad)

        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_psi = math.cos(psi_rad)
        sin_psi = math.sin(psi_rad)

        force_east = (
            cos_theta * cos_psi * a_v
            - sin_theta * cos_psi * a_theta
            - sin_psi * a_psi
        )
        force_north = (
            cos_theta * sin_psi * a_v
            - sin_theta * sin_psi * a_theta
            + cos_psi * a_psi
        )
        force_up = sin_theta * a_v + cos_theta * a_theta
        # 再旋回 ENU，并减去重力得到净加速度（正变换里曾 +g，这里对称地 -g）。
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
            nz_min=cls.DEFAULT_NZ_MIN,
            nz_max=cls.DEFAULT_NZ_MAX,
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
            nz_min=float(limits_config.get("nz_min", cls.DEFAULT_NZ_MIN)),
            nz_max=float(limits_config.get("nz_max", cls.DEFAULT_NZ_MAX)),
            phi_min_rad=math.radians(float(limits_config.get("phi_min_deg", cls.DEFAULT_PHI_MIN_DEG))),
            phi_max_rad=math.radians(float(limits_config.get("phi_max_deg", cls.DEFAULT_PHI_MAX_DEG))),
        )
        cls._validate_config(config)  # 构造后立即校验，非法配置直接报错
        return config

    @staticmethod
    def _validate_config(config: PointMassModelConfig) -> None:
        """校验模型配置合法性。注意：失败应阻止仿真继续运行。"""
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
        # nz 由 hypot 得出恒非负，故下限必须 >=0 且严格小于上限。
        if config.nz_min < 0.0 or config.nz_min >= config.nz_max:
            raise ValueError("model.limits.nz_min must be non-negative and less than nz_max")
        if config.phi_min_rad >= config.phi_max_rad:
            raise ValueError("model.limits.phi_min_deg must be less than phi_max_deg")
        # 下面三条保证过载/滚转区间覆盖平飞配平点 (nx=0, nz=1, phi=0)，否则无法稳定平飞。
        if config.nz_max < 1.0:
            raise ValueError("model.limits.nz_max must be >= 1.0 to allow level-flight trim (nz=1)")
        if config.nz_min > 1.0:
            raise ValueError("model.limits.nz range must include 1.0 to allow level-flight trim (nz=1)")
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
