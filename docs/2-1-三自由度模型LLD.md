# 三自由度无人机质点模型 LLD

## 1. 范围

本模型用于仿真固定翼 / 无人机在三维空间中的三自由度质点运动。模型保留经典三自由度状态：

```text
x = [E, N, U, V, theta, psi]^T
```

含义：

- `E/N/U`：东北天坐标系位置，单位均为 m。
- `V`：空速或无风条件下的飞行速度。
- `theta`：航迹倾角，向上爬升为正。
- `psi`：航向角，`psi=0` 指东，`psi=90 deg` 指北。

本模型不再把无人机简化为三轴二重积分器。算法侧输出东北天坐标系三轴加速度指令，模型侧再把加速度层指令转换为三自由度质点模型的输入量。

边界：

- 不建模刚体六自由度姿态动力学。
- 不建模角速度、惯量、舵面、发动机、升阻力气动系数。
- `phi` 仅作为三自由度质点模型中的滚转输入 / 协调转弯分配量，不表示完整姿态动力学。

## 2. 坐标系

### 2.1 东北天坐标系 `I`

```text
I = [E, N, U]
```

- `E`：East，向东为正。
- `N`：North，向北为正。
- `U`：Up，向上为正。
- 重力加速度为 `[0, 0, -g]^T`。

工程字段映射：

| 坐标 | 字段 | 说明 |
| --- | --- | --- |
| `E` | `x_m` | 东向位置，单位 m |
| `N` | `y_m` | 北向位置，单位 m |
| `U` | `altitude_m` | 高度，单位 m |

### 2.2 航迹坐标系 `k`

航迹坐标系随速度方向定义：

- `e_v`：沿速度方向。
- `e_theta`：航迹倾角增大方向。
- `e_psi`：航向角增大方向。

在东北天坐标系下：

```text
e_v     = [ cos(theta) cos(psi),  cos(theta) sin(psi), sin(theta)]^T
e_theta = [-sin(theta) cos(psi), -sin(theta) sin(psi), cos(theta)]^T
e_psi   = [-sin(psi),             cos(psi),            0         ]^T
```

令：

```text
C_k^I = [e_v, e_theta, e_psi]
```

则东北天向量转换到航迹坐标系：

```text
a_k = (C_k^I)^T a_I
```

## 3. 控制输入链路

节点算法输出东北天坐标系三轴加速度层指令：

```text
a_I,c = [a_E,c, a_N,c, a_U,c]^T
```

该指令表示期望的东北天坐标系运动加速度，不包含重力。模型接收指令后，先通过二阶内回路滤波得到实际响应加速度：

```text
a_I = [a_E, a_N, a_U]^T
```

滤波后的 `a_I` 才进入控制分配。模型输入需要的是飞机产生的比力 / 过载效果，因此对滤波后的加速度做重力补偿：

```text
f_I = [a_E, a_N, a_U + g]^T
```

再转换到航迹坐标系：

```text
[a_v, a_theta, a_psi]^T = (C_k^I)^T f_I
```

由航迹坐标系加速度分量得到三自由度质点模型输入：

```text
N_x = a_v / g
N_z = sqrt(a_theta^2 + a_psi^2) / g
phi = atan2(a_psi, a_theta)
```

其中：

- `N_x`：切向过载输入，控制速度变化。
- `N_z`：法向过载输入，控制航迹弯曲能力。
- `phi`：滚转输入，用于把法向过载分配到爬升平面和转弯平面。

## 4. 二阶内回路

二阶内回路放在模型接收东北天加速度指令之后、重力补偿和坐标转换之前。每个东北天加速度通道保留二阶系统，模拟内回路延迟和超调：

```text
r_ddot + 2 zeta_r omega_r r_dot + omega_r^2 r = omega_r^2 r_c
```

其中：

```text
r in {a_E, a_N, a_U}
r_c in {a_E,c, a_N,c, a_U,c}
```

等价一阶状态：

```text
r_dot     = q
q_dot     = omega_r^2 (r_c - r) - 2 zeta_r omega_r q
```

模型推进时使用滤波后的东北天加速度：

```text
a_E, a_N, a_U
```

而不是直接使用节点算法给出的命令：

```text
a_E,c, a_N,c, a_U,c
```

默认参数：

```text
omega_r = 4 rad/s
zeta_r  = 0.65
```

`zeta_r < 1`，因此阶跃响应会出现可见超调。按 `zeta=0.65` 估算，超调约为 `6.8%`。

## 5. 状态定义

物理三自由度状态：

```text
x_3dof = [E, N, U, V, theta, psi]^T
```

为保留二阶内回路，模型增广状态为：

```text
x = [
  E, N, U,
  V, theta, psi,
  a_E, a_N, a_U,
  a_E_dot, a_N_dot, a_U_dot
]^T
```

说明：

- 前 6 个状态是三自由度质点模型状态。
- 后 6 个状态是加速度指令二阶内回路状态。
- 数值状态维数仍为 12，但后 6 个状态表示滤波后的东北天加速度及其变化率，不再直接作为质点模型状态积分出位置。

## 6. 三自由度质点方程

滤波后的东北天加速度经过重力补偿和坐标转换后，形成 `N_x / N_z / phi`，再进入原三自由度质点模型：

```text
E_dot     = V cos(theta) cos(psi)
N_dot     = V cos(theta) sin(psi)
U_dot     = V sin(theta)

V_dot     = g (N_x - sin(theta))
theta_dot = g / V * (N_z cos(phi) - cos(theta))
psi_dot   = g N_z sin(phi) / (V cos(theta))
```

其中：

- `N_x`：由滤波后东北天加速度转换得到的切向过载。
- `N_z`：由滤波后东北天加速度转换得到的法向过载。
- `phi`：由滤波后东北天加速度转换得到的滚转输入。

工程实现需要对以下奇异点做保护：

- `V <= V_min` 时，使用 `V_min` 参与分母计算。
- `abs(cos(theta)) <= eps` 时，限制 `theta` 或使用 `eps` 防止航向角速度发散。

## 7. 风扰

风扰作为地速修正进入位置方程：

```text
p_dot = V e_v + wind_I
```

即：

```text
E_dot = V cos(theta) cos(psi) + wind_E
N_dot = V cos(theta) sin(psi) + wind_N
U_dot = V sin(theta)          + wind_U
```

风扰不直接进入东北天加速度二阶内回路。

`inject_wind` 接口的 `direction_deg` 采用**数学角约定**（0° 指东、90° 指北、逆时针为正），与气象角约定（从哪个方向吹来）相反。调用方注意区分。

## 8. 输入限幅

输入链路按以下顺序限幅：

1. 东北天加速度指令限幅，避免算法输出异常。
2. 二阶滤波后的东北天加速度再做安全夹紧，避免滤波超调导致异常输入。
3. 转换后的 `N_x / N_z / phi` 限幅，保证三自由度模型输入物理可接受。

建议默认值：

```text
g = 9.80665 m/s^2
V_min = 1.0 m/s

N_x in [-1.0,  1.0]
N_z in [ 0.0,  4.0]
phi in [-70 deg, 70 deg]
```

模型**不约束终端速度上限**。无气动模型时持续正切向过载会导致速度无界增长，属设计固有特性（无阻力）。算法侧应自行管理速度目标；如需速度包线，可通过 `max_speed_mps` 扩展配置项引入。

## 9. 数值推进

- 仿真控制默认以 `dt=0.005s` 调用模型。
- 节点算法未触发的基础 tick 中，沿用上一帧加速度指令，等价于零阶保持。
- 连续方程使用四阶 Runge-Kutta 推进。
- RK4 推进的状态包含三自由度质点状态和二阶内回路状态。

## 10. 配置

建议配置结构：

```json
{
  "model": {
    "gravity_mps2": 9.80665,
    "min_speed_mps": 1.0,
    "acceleration_filter": {
      "natural_frequency_rad_s": 4.0,
      "damping_ratio": 0.65
    },
    "limits": {
      "acceleration_command_mps2": 6.0,
      "max_climb_rate_mps": 8.0,
      "max_descent_rate_mps": 8.0,
      "nx_min": -1.0,
      "nx_max": 1.0,
      "nz_min": 0.0,
      "nz_max": 4.0,
      "phi_min_deg": -70.0,
      "phi_max_deg": 70.0
    }
  }
}
```

为了兼容已有配置，实现可以继续接受：

```json
{
  "model": {
    "natural_frequency_rad_s": 4.0,
    "damping_ratio": 0.65,
    "max_acceleration_mps2": 6.0
  }
}
```

兼容字段映射：

- `natural_frequency_rad_s` -> `acceleration_filter.natural_frequency_rad_s`
- `damping_ratio` -> `acceleration_filter.damping_ratio`
- `max_acceleration_mps2` -> `limits.acceleration_command_mps2`

垂向速度包线：

- `limits.max_climb_rate_mps`：最大爬升率，正值，默认 `8.0 m/s`。
- `limits.max_descent_rate_mps`：最大下沉率，正值，默认 `8.0 m/s`。
- 模型在积分后裁剪航迹倾角 `theta`，保证 `V * sin(theta)` 落在 `[-max_descent_rate_mps, max_climb_rate_mps]` 内；速度标量 `V` 不因该裁剪被直接改写。

## 11. 初始条件

每个节点支持以下初始量：

| 配置字段 | 含义 | 默认值 |
| --- | --- | --- |
| `x_m/y_m/altitude_m` | 东北天位置 `E/N/U` | 按节点序号生成 |
| `speed_mps` | 初始速度 `V` | `5.0` |
| `theta_deg` | 初始航迹倾角 | `0.0` |
| `psi_v_deg` | 初始航向角 `psi` | `0.0` |
| `nx` | 初始切向过载 | `sin(theta)` |
| `nz` | 初始法向过载 | `cos(theta)` |
| `phi_deg` | 初始滚转输入 | `0.0` |

平飞配平时：

```text
theta = 0
N_x = 0
N_z = 1
phi = 0
```

## 12. 软件接口

```python
class ModelIterator:
    def init(config: dict[str, object], seed: int) -> None: ...
    def read_states() -> dict[str, AircraftState]: ...
    def apply_controls(controls: Mapping[str, AccelerationCommand]) -> None: ...
    def step(dt_s: float) -> None: ...
    def inject_wind(command: object) -> None: ...
    def clear_wind() -> None: ...  # 仅清除风扰，不涉及健康状态
    def reset() -> None: ...
    def close() -> None: ...
```

`AircraftState` 只包含物理状态，不含制导或任务层概念：

```python
AircraftState(
    node_id,          # 节点标识
    x_m, y_m, altitude_m,          # 东北天位置
    speed_mps, theta_rad, psi_rad,  # 速度、航迹倾角、航向角
    ax_mps2, ay_mps2, az_mps2,     # 滤波后的东北天加速度
    ax_rate_mps3, ay_rate_mps3, az_rate_mps3,  # 加速度变化率
    nx, nz, phi_rad,  # 转换后的质点模型输入
    psi_dot_deg_s,    # 航迹偏航角速率（度/秒），左转为正
)
```

节点角色（`role`）、健康状态（`health`）、侧偏（`cross_track_error_m`）等属于仿真控制 / 制导层，不在模型层管理。

`AccelerationCommand` 仍表示算法输出的东北天加速度层指令：

```python
AccelerationCommand(
    ax_cmd_mps2,  # a_E,c
    ay_cmd_mps2,  # a_N,c
    az_cmd_mps2,  # a_U,c
)
```

`ModelIterator` 内部负责：

1. 对东北天加速度指令做二阶滤波。
2. 对滤波后的东北天加速度做重力补偿。
3. 东北天坐标系到航迹坐标系转换。
4. 转换为 `N_x / N_z / phi`。
5. 推进三自由度质点方程。

## 13. 与仿真控制的关系

每个基础 tick 内：

1. 节点算法生成东北天三轴加速度指令。
2. 仿真控制保留最近一次有效指令。
3. 仿真控制调用 `apply_controls()`。
4. 加扰模块更新风扰；健康状态由加扰模块自行维护，不下发给模型。
5. 模型内部完成加速度二阶滤波、控制分配和三自由度积分。
6. 仿真控制读取新状态并生成快照。

仿真控制不理解 `N_x / N_z / phi` 的物理含义，也不直接调用坐标转换；这些都属于模型迭代模块。

## 14. 扩展边界

以下能力不应继续塞入当前三自由度质点模型，应使用新模型类型或扩展状态：

- 六自由度刚体动力学。
- 姿态角速度和姿态控制律。
- 舵面、推力、发动机动态。
- 升力、阻力、侧力气动系数表。
- 真实自动驾驶仪控制律。
