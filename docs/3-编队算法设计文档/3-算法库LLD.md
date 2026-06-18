# 算法库 LLD

> 算法库是一组**不碰 `Mode` 的纯计算类**（实例态在 `self`、无全局态），跨方法复用；由流程在外部按 Mode 选 / 切（分界见 `0-HLD.md` 原则 9）。
> 单元小协议见 `1-LLD综述.md` §2.1：`init(params, bind) / step(self) / reset / read_state`。除 `ControlLaw` 被 `Tracking` 内部组合调用外，顶层单元均通过已绑定黑板视图读写。

## 1. 范围与包结构

目标包结构：

```text
src/algorithm/algo_lib/
├── types.py             # Segment/Target/Deviation/FormationCommand 等算法叶类型
├── math_utils.py        # 坐标变换、限幅、角度归一化、向量工具
├── position.py          # PositionSolve 族：航线插值、槽位几何
├── deviation.py         # DeviationCalc 族
├── tracking.py          # Tracking 族
└── control_law.py       # ControlLaw 族：PID 等
```

本轮算法库包含 4 个策略族 + 通用数学：

| 族 | 顶层 step 单元 | 本轮实现 | 输出契约 |
| --- | --- | --- | --- |
| `PositionSolve` | 是 | 航线插值、槽位几何 | 必写 `target`；航线插值另写 `distance_to_go_m` |
| `DeviationCalc` | 是 | 标准 ENU 误差 | 必写 `deviation` |
| `Tracking` | 是 | 三轴 PID 加速度跟踪 | 必写 `control` |
| `ControlLaw` | 否，被 `Tracking` 组合 | PID | 返回标量控制量 |
| 通用数学 | 否 | 坐标 / 限幅 / 滤波工具 | 无状态函数 |

## 2. 数据类型

算法库使用的叶类型应可直接映射为 C struct。

```python
@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

@dataclass
class Segment:
    segment_id: int
    start: Vec3
    end: Vec3
    speed_mps: float

@dataclass
class Target:
    position: Vec3
    velocity: Vec3
    speed_mps: float
    theta_rad: float
    psi_rad: float

@dataclass
class Deviation:
    position_error: Vec3
    velocity_error: Vec3
    along_track_error_m: float
    cross_track_error_m: float
    altitude_error_m: float
    speed_error_mps: float
```

`NavState` 来自对外契约，含 `x_m / y_m / altitude_m / speed_mps / theta_rad / psi_rad`。通用速度向量：

```text
v = speed * [cos(theta)cos(psi), cos(theta)sin(psi), sin(theta)]
```

坐标约定：

- ENU：`x=East`、`y=North`、`z=Up`。
- `psi=0` 指东，`psi=pi/2` 指北。
- 算法库输出 `AccelerationCommand` 表示不含重力的 ENU 运动加速度指令。

## 3. 通用数学

通用数学是无状态工具函数，不作为策略族挂黑板。

| 函数 | 说明 |
| --- | --- |
| `nav_velocity(state) -> Vec3` | 从 `speed/theta/psi` 计算 ENU 速度 |
| `unit_from_yaw_pitch(psi, theta) -> Vec3` | 航迹单位向量 |
| `normalize_angle_rad(angle) -> float` | 归一到 `[-pi, pi]` |
| `clamp(value, lo, hi)` / `clamp_vec3` | 标量 / 向量限幅 |
| `dot/cross/norm/normalize` | 基础向量运算 |
| `track_frame(psi, theta) -> (e_forward, e_right, e_up)` | 槽位几何用局部坐标系 |

槽位几何使用长机航迹局部系：

```text
e_forward = [cos(theta)cos(psi), cos(theta)sin(psi), sin(theta)]
e_right   = [sin(psi), -cos(psi), 0]       # 航向右侧；psi=0 时右侧为南
e_up      = normalize(cross(e_right, e_forward))
```

`e_up` 与 ENU 上方向接近，但跟随航迹倾角做轻微倾斜。若首版希望更简单，也可固定 `e_up=[0,0,1]`；一旦采用固定上方向，`e_right` 与 `e_forward` 应重新正交化，避免爬升段槽位畸变。本文默认采用上面的航迹局部系。

## 4. `PositionSolve` 族

### 4.1 族契约

父类输出：

| 输出槽 | 类型 | 说明 |
| --- | --- | --- |
| `target` | `Target` | 下游必读目标指令 |

实现私有输出：

| 实现 | 额外输出 |
| --- | --- |
| 航线插值 | `distance_to_go_m` |
| 槽位几何 | 无 |

### 4.2 航线插值（长机）

输入：`segment`、`own_state`。输出：`target`、`distance_to_go_m`。

定义：

```text
p0 = segment.start
p1 = segment.end
p  = own position
d  = p1 - p0
L  = ||d||
e  = d / max(L, eps)
s  = dot(p - p0, e)
s_clamped = clamp(s, 0, L)
target_pos = p0 + s_clamped * e
distance_to_go_m = L - s_clamped
target_vel = segment.speed_mps * e
```

目标角：

```text
psi_ref   = atan2(e_y, e_x)
theta_ref = atan2(e_z, sqrt(e_x^2 + e_y^2))
```

重要口径：

- 航线插值的 `target_pos` 是本机当前位置在航段上的投影点，不前瞻；因此前向位置误差在 `DeviationCalc` 中归零。
- 若 `L < eps`，认为航段退化：`target_pos = own position`，`target_vel = own velocity`，`distance_to_go_m = 0`。
- 航点前瞻、切换和待飞距阈值属于流程库 `TrajectoryPlan`，不放在 `PositionSolve`。

可选前瞻留作未来参数：

```text
target_pos = p0 + clamp(s + lookahead_m, 0, L) * e
```

本轮默认 `lookahead_m = 0`，避免长机在航线切换逻辑未完善时出现额外耦合。

### 4.3 槽位几何（僚机）

输入：`formation_command`、`host_state`；静态参数：`aircraft_id`、`formation_table`。输出：`target`。

队形表按飞机 ID 给相对偏置：

```json
{
  "wedge": {
    "A02": {"forward_m": -40.0, "right_m": -35.0, "up_m": 0.0},
    "A03": {"forward_m": -40.0, "right_m": 35.0, "up_m": 0.0}
  }
}
```

槽位计算：

```text
host_p = [host.x_m, host.y_m, host.altitude_m]
host_v = nav_velocity(host)
(ef, er, eu) = track_frame(host.psi_rad, host.theta_rad)
offset = forward_m * ef + right_m * er + up_m * eu
target_pos = host_p + offset
target_vel = host_v
target_speed = host.speed_mps
target_theta = host.theta_rad
target_psi = host.psi_rad
```

异常处理：

- 未知队形：使用配置默认队形；仍未知则抛配置错误，避免运行期静默飞错槽位。
- 队形中没有当前飞机 ID：`init` 失败，不允许运行时回退到零偏置。
- `host_state` 可能因丢包保持上一帧；新鲜度 / 降级不在本算法单元内处理，本轮由流程库保持槽语义兜底。

## 5. `DeviationCalc` 族

输入：`target`、`own_state`。输出：`deviation`。

标准定义：

```text
pos_error = target.position - own.position
vel_error = target.velocity - own.velocity
speed_error = target.speed_mps - own.speed_mps
```

为便于跟踪和日志，额外计算长机 / 僚机共用的航迹分解：

```text
e_forward = unit_from_yaw_pitch(target.psi_rad, target.theta_rad)
e_right   = [sin(target.psi_rad), -cos(target.psi_rad), 0]
e_up      = normalize(cross(e_right, e_forward))

along_track_error = dot(pos_error, e_forward)
cross_track_error = dot(pos_error, e_right)
altitude_error    = pos_error.z
```

航线插值口径：

- 当 `target` 来自航线投影时，`along_track_error` 应强制置 0 或仅作为日志值，不进入控制。
- 这样长机不会因为投影点在自身前后微小抖动而沿航段来回拉扯；速度误差负责沿航向加减速。

输出要求：

- `Deviation.position_error` 保留完整 ENU 误差，供简单三轴 PID 直接使用。
- `along/cross/altitude/speed` 供日志和未来 L1/TECS 使用。

## 6. `Tracking` 族

本轮实现：三轴 ENU PD/PID 加速度跟踪，内部组合 3 个 `PID` 控制律。

输入：`deviation`、`dt_s`。输出：`control: AccelerationCommand`。

基础律：

```text
a_cmd = Kp_pos * position_error + Kd_vel * velocity_error + Ki_pos * integral(position_error)
```

按轴展开：

```text
ax = pid_x.step(ex_pos, ex_vel, dt)
ay = pid_y.step(ey_pos, ey_vel, dt)
az = pid_z.step(ez_pos, ez_vel, dt)
```

首版建议实现成“位置 P + 速度 D + 位置积分 I”：

```text
u = kp * pos_error + kd * vel_error + ki * integral_pos
```

限幅顺序：

1. 积分限幅：`integral_pos ∈ [-integral_limit, integral_limit]`。
2. 单轴输出限幅：`u_axis ∈ [-axis_limit_mps2, axis_limit_mps2]`。
3. 向量模长限幅：`||a_cmd|| <= acceleration_command_limit_mps2`，默认对齐模型配置 `6.0 m/s^2`。

抗积分饱和：

- 若输出已饱和且误差继续推动同方向饱和，本拍不累积积分。
- `reset()` 清零积分和上一拍误差。

默认参数（首版保守值，后续用测试和仿真调参）：

| 通道 | `kp` | `kd` | `ki` | 单轴限幅 |
| --- | ---: | ---: | ---: | ---: |
| East/X | 0.08 | 0.80 | 0.00 | 4.0 |
| North/Y | 0.08 | 0.80 | 0.00 | 4.0 |
| Up/Z | 0.06 | 0.70 | 0.00 | 3.0 |

说明：

- 默认 `ki=0`，先按 PD 跑通，避免在通信延迟 / 丢包下积分累积导致明显超调。
- 增益在 `init` 从配置覆盖；文档默认值只用于没有配置时的可运行兜底。
- `Tracking` 不读 `Mode`；不同模态若需不同跟踪组合，由实体挂接 / 未来流程切管线决定。

## 7. `ControlLaw` 族

`ControlLaw` 是原子控制律，被 `Tracking` 组合调用，不直接绑定黑板。

### 7.1 PID

接口建议：

```python
class PID:
    def step(self, error: float, error_rate: float, dt_s: float) -> float: ...
    def reset(self) -> None: ...
    def read_state(self) -> dict[str, float]: ...
```

其中 `error_rate` 本轮直接传速度误差，而不是对位置误差做数值差分。这样可以避免因采样噪声和通信保持槽造成差分尖峰。

状态：

| 状态 | 说明 |
| --- | --- |
| `integral` | 位置误差积分 |
| `last_output` | 便于日志 / 饱和判断 |
| `saturated` | 上一拍是否饱和 |

未来实现：

- `L1`：用于水平面路径跟踪，吃横向误差 / 航向误差。
- `TECS`：用于速度 / 高度能量分配。
- `ADRC`：用于扰动估计和鲁棒控制。

## 8. 单元状态与可测试性

每个单元必须支持纯单元测试：

| 单元 | 最小测试 |
| --- | --- |
| 航线插值 | 水平航段、爬升航段、退化航段、越过终点时待飞距为 0 |
| 槽位几何 | `psi=0/90deg` 下左右槽位方向正确，未知飞机 ID 初始化失败 |
| 误差解算 | 目标速度向量、ENU 位置误差、航线前向误差口径 |
| PID | 积分限幅、输出限幅、reset 清状态 |
| Tracking | 三轴输出限幅、速度误差产生阻尼项、`dt_s` 参与积分 |

单元测试应直接构造黑板和视图，不依赖仿真控制或 PySide6。

## 9. TODO

- 用仿真结果调整默认增益，并固化一组可回归的“领航跟随保持”数值测试。
- 补 L1 / TECS / ADRC 的实现文档与选择条件。
- 补通信新鲜度参与控制降级的口径：限速、保航向、停控的阈值与优先级。
- 形成配置 schema：增益、限幅、队形几何、航线前瞻等参数的默认值与合法范围。
