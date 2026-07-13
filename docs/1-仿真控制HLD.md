# 仿真控制 HLD

## 1. 定位

仿真控制是全系统唯一的编排 Control，负责生命周期、任务调度、配置分发、扰动触发、数据落盘和实时数据推送，不承载具体算法、动力学或链路领域逻辑。

UI、CLI、批量脚本都只面向仿真控制，不直接调用编队算法、模型迭代、通信功能或加扰模块。仿真控制对外提供稳定的应用层接口，对内适配各 Control 的领域接口。

## 2. 职责

- 基于配置初始化编队算法、模型迭代、通信功能、加扰和日志。
- 按配置节拍驱动各 Control 的 `tick` / `step`。
- 将拓扑/QoS 配置下发给通信功能。
- 将扰动配置和不确定性索引下发给加扰。
- 从模型迭代和通信功能读取状态 / Inbox，作为参数注入算法。
- 读取算法输出的控制量 / Outbox，并写入模型迭代和通信功能。
- 按日志节拍写关键数据。
- 对 UI 暴露配置加载、生命周期、单步、扰动注入、快照订阅和日志查询入口。
- 对 CLI 暴露同一套运行入口，支持 headless 单次仿真。

## 3. 边界

- 不直接实现编队算法、制导律或控制律。
- 不直接实现动力学积分。
- 不直接实现通信 QoS 行为。
- 不直接触达模型 / 通信的动态扰动注入接口，动态扰动统一转交加扰。
- 不持有 UI 控件对象，不依赖 PySide6；UI 通过回调、信号适配器或轮询方式消费快照。
- 不负责离线分析和报告生成，只负责按日志接口落盘关键数据。

## 4. 应用层数据契约

以下类型为 HLD 级接口契约，代码实现可使用 `dataclass`、`TypedDict` 或 Pydantic 等方式承载。

### 4.1 枚举

```python
RunState = Literal[
    "UNLOADED",   # 未加载配置
    "READY",      # 已加载并初始化，等待运行
    "RUNNING",    # 正在按节拍推进
    "PAUSED",     # 暂停，可单步
    "FINISHED",   # 达到仿真结束时间
]

ControlReport = Literal[
    "待命",
    "集结",
    "保持",
    "重构",
]

EventLevel = Literal["DEBUG", "INFO", "WARN", "ERROR"]
DisturbanceType = Literal["wind", "node_fault", "link_loss", "link_fault", "clear"]

ResultCode = Literal[
    "OK",
    "ERR_NO_CONFIG",
    "ERR_CONFIG_NOT_FOUND",
    "ERR_CONFIG_INVALID",
    "ERR_INVALID_STATE",
    "ERR_INVALID_ARGUMENT",
    "ERR_BUSY",
    "ERR_MODULE_INIT_FAILED",
    "ERR_TICK_FAILED",
    "ERR_LOG_FAILED",
    "ERR_INTERNAL",
]
```

### 4.2 节点状态

结构体名：`NodeState`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `node_id` | `str` | 节点 ID，如 `A01` |
| `role` | `str` | `leader` / `wingman` / `relay` 等 |
| `health` | `str` | `normal` / `degraded` / `fault` / `lost` |
| `x_m` | `float` | 东向位置 `E`，单位 m |
| `y_m` | `float` | 北向位置 `N`，单位 m |
| `altitude_m` | `float` | 天向位置 / 高度 `U`，单位 m |
| `psi_v_deg` | `float` | 航向角 / 航迹方位角，单位 deg；`0 deg` 指东，`90 deg` 指北 |
| `theta_deg` | `float` | 航迹倾角，单位 deg |
| `speed_mps` | `float` | 速度 |
| `vx_mps/vy_mps/vz_mps` | `float` | 由 `V/theta/psi` 派生的东北天速度，供 UI 和日志使用 |
| `nx/nz/phi_deg` | `float` | 由二阶滤波后的东北天加速度转换得到的质点模型输入 |
| `psi_dot_deg_s` | `float` | 航迹偏航角速率，单位 deg/s；左转（逆时针）为正 |
| `cmd_pos_east_m/cmd_pos_north_m/cmd_pos_h_m` | `float` | 位置解算得到的目标位置，东北天坐标系 |
| `cmd_vel_east_mps/cmd_vel_north_mps/cmd_vel_up_mps` | `float` | 位置解算得到的目标速度，东北天坐标系 |
| `pos_err_east_m/pos_err_north_m/pos_err_h_m` | `float` | 控制过程中的位置误差，东北天坐标系 |
| `vel_err_east_mps/vel_err_north_mps/vel_err_up_mps` | `float` | 控制过程中的速度误差，东北天坐标系 |
| `track_pos_err_x_m/track_pos_err_y_m/track_pos_err_z_m` | `float` | 控制过程中的位置误差诊断，航迹坐标系；`x/y/z` 分别为前向、垂向、右侧向；对应轴位置误差未被控制参数使用时输出 0 |
| `track_vel_err_x_mps/track_vel_err_y_mps/track_vel_err_z_mps` | `float` | 控制过程中的速度误差诊断，航迹坐标系；`x/y/z` 分别为前向、垂向、右侧向；对应轴速度误差未被控制参数使用时输出 0 |

坐标命名约定：

- 东北天坐标系：位置字段使用 `east/north/h`，速度和加速度字段使用 `east/north/up`。
- 航迹坐标系：位置、速度和加速度字段都使用 `x/y/z`，其中 `x` 为前向，`y` 为垂向，`z` 为右侧向。
- 仿真控制不从对象组内部 `Context` 抓取诊断量；对象组必须通过 `EntityOutputS` 显式交出位置/速度指令和误差，仿真控制再写入 `NodeState`。

### 4.3 链路状态

结构体名：`LinkState`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `link_id` | `str` | 链路 ID，格式 `源节点-目标节点`，源节点在前、目标节点在后，如 `A01-A02` |
| `direction` | `str` | `duplex` / `simplex`，默认 `duplex` |
| `latency_ms` | `float` | 当前延迟 |
| `loss_rate` | `float` | 当前丢包率，范围 `[0, 1]` |
| `status` | `str` | `normal` / `degraded` / `lost` |

约束：

- `duplex` 表示 `link_id` 两端共享同一条双向链路状态。
- `simplex` 表示链路方向按 `link_id` 的源节点到目标节点解释；反向链路需要单独一条 `link_id`。
- 带宽字段不进入仿真控制基础配置契约。

### 4.4 仿真快照

结构体名：`SimulationSnapshot`

`SimulationSnapshot` 是 UI 和 CLI 观察仿真的唯一实时数据结构。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `time_s` | `float` | 当前仿真时间 |
| `duration_s` | `float` | 仿真总时长 |
| `step_s` | `float` | 仿真步长 |
| `run_state` | `RunState` | 运行状态 |
| `control_report` | `ControlReport` | 顶部回报文本 |
| `nodes` | `list[NodeState]` | 节点状态 |
| `links` | `list[LinkState]` | 链路状态 |

### 4.5 事件

结构体名：`SimulationEvent`

`SimulationEvent` 不随实时快照定时推送，也不嵌入命令返回；日志窗口和需要查看事件的调用方统一通过 `get_recent_events()` 查询最近事件。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `time_s` | `float` | 仿真时间 |
| `level` | `EventLevel` | 等级 |
| `source` | `str` | 事件来源，如 `SimControl`、`Disturbance` |
| `message` | `str` | 给日志窗口展示的文本 |

### 4.6 命令结果

结构体名：`CommandResult`

所有会改变仿真状态的应用层接口返回 `CommandResult`。调用方通过 `code == "OK"` 判断命令是否成功；若需要最新状态，调用 `get_snapshot()` 或等待订阅推送。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `code` | `ResultCode` | `OK` 或错误码 |
| `message` | `str` | 面向 UI / CLI 的短消息 |

## 5. 应用层接口契约

### 5.1 接口总览

UI HLD 中的用户操作统一映射到仿真控制应用层接口：

| UI 操作 | 仿真控制接口 | 说明 |
| --- | --- | --- |
| 选择配置文件 | `load_config(path)` | 解析、校验并保存配置，返回命令执行结果 |
| 开始 / 继续 | `start()` | 从 `READY` 或 `PAUSED` 进入运行；`PAUSED` 下语义为继续 |
| 暂停 | `pause()` | 从 `RUNNING` 进入 `PAUSED` |
| 单步 | `step(count=1)` | 通常在 `READY` / `PAUSED` 下推进指定步数 |
| 重置 | `reset()` | 回到当前配置的初始状态 |
| 注入扰动 | `inject_disturbance(command)` | 转发动态扰动命令给加扰模块 |
| 订阅实时数据 | `subscribe_snapshot(callback)` | 仿真控制按节拍推送 `SimulationSnapshot` |
| 打开日志 | `get_recent_events(limit)` | UI 日志面板按需读取最近事件 |
| 修改播放倍率 | `set_playback_rate(rate)` | 只影响 wall-clock 推进频率，不改变仿真步长 |

接口原则：

- UI 不直接写仿真状态；所有状态变化都通过命令接口完成。
- UI 显示字段完全来自 `SimulationSnapshot`，不拼装跨模块内部数据。
- 配置属于输入；实时快照只表达运行输出，不携带配置摘要。
- 播放倍率是运行调度参数，通过 `set_playback_rate(rate)` 设置，不进入实时快照。
- 渐隐轨迹属于 UI 显示缓存，UI 根据连续 `SimulationSnapshot` 自行维护；保留窗口按仿真时间计算，不随播放倍率变化。
- 窗口关闭或 CLI 退出时调用 `close()` 释放资源，不单独设计 `stop()`。
- “继续”不单独设计 `resume()`，统一使用 `start()`。
- UI 可以缓存事件用于显示，但事件源和最近事件环形缓冲由仿真控制维护，保证后台模块事件、错误事件和日志面板晚打开时的历史事件不会丢失。

### 5.2 创建控制器

```python
controller = SimulationController()
```

创建后状态为 `UNLOADED`。构造函数不读取配置、不启动线程、不 import GUI。

### 5.3 加载配置

```python
def load_config(path: str) -> CommandResult
```

语义：

- 读取 `.yaml`、`.yml` 或 `.json` 配置。
- 校验必填字段、节点列表、链路拓扑、航线 `route`、队形 `formation`、仿真时长、步长、算法分频、扰动配置；`step_s` 必须满足 `0 < step_s <= 0.1`，避免单个基础 tick 跨过多个固定快照边界。
- **航线只支持经纬度**：`route`（含 `route_file` 引用）的航点必须给 `latitude_deg` + `longitude_deg`，加载期由 `route_to_internal` 统一转成内部 ENU（以首航点为 ENU 原点）；非经纬(如 ENU `x_m`/`y_m`)航线在加载期被拒绝，返回 `ERR_CONFIG_INVALID`。节点初始位置 `nodes[*]` 不受此约束；`formation.formation_files` 引用的外部队形文件使用局部航迹坐标 `x_forward_y_up_z_right`，同样不参与经纬转换。
- **有效航线语义**：初始有效航线是 `route_file` 展开的 `route`；调用 `apply_avoidance_route()` 采用避障规划结果后，覆盖航线成为新的有效航线并触发模块重新初始化。集结中心、集结方向、高度分层、GUI 集结几何和集结完成后的任务飞行全部读取同一条有效航线，确保集结阶段也遵循避障结果。
- 初始化模型、通信、算法、加扰和日志对象，但不开始推进仿真时间。
- 成功后状态进入 `READY`，并通过 `get_snapshot()` 或订阅推送提供初始快照。

约束：

- 当前处于 `RUNNING` 时返回 `ERR_BUSY`；用户需要先 `pause()` 或 `reset()`。
- 配置加载失败时状态保持原值；若没有可用旧配置，则保持 `UNLOADED`。

### 5.4 获取当前快照

```python
def get_snapshot() -> SimulationSnapshot
```

语义：

- 返回最近一次完整快照。
- 不推进仿真，不改变状态。
- 若未加载配置，返回 `UNLOADED` 快照或抛出实现约定的 `SimulationError`；建议返回快照，便于 UI 初始化。

### 5.5 开始 / 继续

```python
def start() -> CommandResult
```

语义：

- `READY`：从当前配置的初始状态开始运行。
- `PAUSED`：继续运行。
- `RUNNING`：幂等返回 `OK`，不重复启动循环。

约束：

- 未加载配置返回 `ERR_NO_CONFIG`。
- `FINISHED` 下返回 `ERR_INVALID_STATE`；用户需要先 `reset()`，再 `start()`。

### 5.6 暂停

```python
def pause() -> CommandResult
```

语义：

- `RUNNING` 进入 `PAUSED`。
- `PAUSED` 幂等返回 `OK`。

约束：

- `READY`、`FINISHED` 下返回 `ERR_INVALID_STATE`。

### 5.7 单步

```python
def step(count: int = 1) -> CommandResult
```

语义：

- 在 `READY` 或 `PAUSED` 下推进 `count` 个仿真步。
- 推进完成后保持 `PAUSED`，便于用户继续检查。
- 若推进后达到 `duration_s`，状态进入 `FINISHED`。

约束：

- `count >= 1`。
- `RUNNING` 下返回 `ERR_INVALID_STATE`，避免并发推进。

### 5.8 重置

```python
def reset() -> CommandResult
```

语义：

- 使用当前已加载配置重建所有内部模块。
- 清空运行期动态扰动、通信队列和算法内部状态。
- 仿真时间回到 `0`，状态进入 `READY`。
- 若当前处于 `RUNNING`，先停止推进循环再重建内部模块。

约束：

- 未加载配置返回 `ERR_NO_CONFIG`。
- 不删除已落盘日志文件；是否新建 run 目录由日志配置决定。

### 5.9 关闭

```python
def close() -> None
```

语义：

- 若当前处于 `RUNNING`，先停止推进循环。
- flush 日志。
- 释放日志文件、后台定时器、线程或进程资源。
- 清理订阅者。
- 供 UI 窗口关闭、CLI 退出、批量任务取消或测试清理时调用。
- 调用后 controller 实例不再复用；需要重新创建实例才能再次运行。

### 5.10 设置播放倍率

```python
def set_playback_rate(rate: float) -> CommandResult
```

语义：

- 设置 wall-clock 到 sim-time 的推进倍率。
- 合法范围 `0.1 <= rate <= 50.0`。
- 不改变 `step_s`，不改变算法输入语义。

### 5.11 注入扰动

```python
def inject_disturbance(command: DisturbanceCommand) -> CommandResult
```

`DisturbanceCommand`：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `type` | `DisturbanceType` | 扰动类型 |
| `target` | `str | None` | 节点 ID 或链路 ID；链路 ID 遵循 `源节点-目标节点` 规则 |
| `duration_s` | `float | None` | 持续时间 |
| `params` | `dict[str, object]` | 强度、方向、丢包率等参数 |

典型命令：

```python
{"type": "wind", "duration_s": 8.0, "params": {"speed_mps": 8.0, "direction_deg": 90}}
{"type": "node_fault", "target": "A02", "duration_s": 10.0, "params": {"mode": "degraded"}}
{"type": "link_loss", "target": "A01-A02", "duration_s": 12.0, "params": {"loss_rate": 0.3}}
{"type": "clear"}
```

语义：

- 仿真控制只校验命令合法性和当前状态。
- 具体扰动解释、持续时间管理和对模型 / 通信的 push 由加扰模块负责。

约束：

- 未加载配置返回 `ERR_NO_CONFIG`。
- `FINISHED` 下不接受注入扰动，返回 `ERR_INVALID_STATE`。

### 5.12 订阅快照

```python
def subscribe_snapshot(callback: Callable[[SimulationSnapshot], None]) -> Subscription
```

`Subscription`：

```python
class Subscription:
    def unsubscribe(self) -> None: ...
```

语义：

- 仿真控制按显示回显刷新节拍调用订阅者，默认 `10 Hz`。
- 回调必须是短耗时函数；UI 线程适配由 UI 层负责。
- 同一个订阅者多次订阅时，建议返回同一订阅或替换旧订阅，避免重复刷新。

约束：

- 回调异常不能打断仿真循环；仿真控制记录 `WARN` 事件并继续。
- 订阅回调中不允许重入调用 `step()`、`reset()` 等改变状态的接口；实现需要加锁或排队。

### 5.13 增量读取固定时钟快照

```python
@dataclass(frozen=True)
class TimedSnapshotCursor:
    run_generation: int
    next_index: int

def read_timed_snapshots(
    cursor: TimedSnapshotCursor | None,
) -> tuple[TimedSnapshotCursor, tuple[SimulationSnapshot, ...]]
```

语义：

- 返回关键数据记录器按仿真时间固定 `10 Hz` 产生、且位于传入游标之后的全部内存快照；GUI 可用它维护倍率无关的尾迹，但不得读取或修改记录器内部列表。
- 读取与基础 tick、日志写入使用同一控制器锁；返回批次为不可变元组，一次墙钟轮询可以消费多个连续样本。
- 每次成功加载配置、`reset()` 或其他完整模块重建都会递增 `run_generation`；旧代游标读取新运行时从索引 `0` 开始，不能沿用旧索引跳过新样本。
- 播放倍率、显示回显节拍和调用频率都不改变该接口的采样时刻。

### 5.14 读取最近事件

```python
def get_recent_events(limit: int = 200, min_level: EventLevel | None = None) -> list[SimulationEvent]
```

语义：

- 给 UI “日志”窗口使用。
- 返回内存环形缓冲中的最近事件，不直接扫描 JSONL 关键数据日志或其他大日志文件。
- UI 可以自己缓存已经收到的事件，但不能作为唯一事件源；仿真控制仍需维护最近事件，覆盖 UI 面板晚打开、UI 刷新丢帧、headless 运行和内部错误追踪等场景。

### 5.15 headless 运行

```python
def run_until_complete(config: object | str, *, seed: int | None = None) -> CommandResult
```

语义：

- 给 CLI / 批量仿真使用。
- 若传入路径，内部执行 `load_config(path)`。
- 从 `READY` 运行到 `FINISHED`；异常场景另行设计。
- 过程中按日志配置落盘；可以不启用快照订阅。

约束：

- 不依赖 UI 事件循环。
- 可被多进程批量脚本并发调用；单个 controller 实例不支持并发运行多个 run。

## 6. 状态机

![仿真控制状态机](assets/仿真控制状态机.svg)

> 图源：[`仿真控制状态机.drawio`](assets/仿真控制状态机.drawio)

`load_config()` 失败不触发状态迁移，只通过 `CommandResult.code` 返回 `ERR_CONFIG_NOT_FOUND`、`ERR_CONFIG_INVALID` 等错误码。

`reset()` 可从 `READY`、`RUNNING`、`PAUSED`、`FINISHED` 回到 `READY`，语义见 5.8。

非法状态转换必须返回 `CommandResult(code="ERR_INVALID_STATE")`，不得静默执行不确定行为。

运行期异常场景不进入本状态机，由异常处理策略单独约束。

## 7. 多速率调度

```python
def _tick() -> SimulationSnapshot
```

仿真控制内部以固定基础步长推进仿真时间。基础 tick 为 `200 Hz`，不同模块不强制每个基础 tick 都执行，而是由调度器按各自频率触发。

本节只描述仿真控制内部推进和显示回显刷新，不描述 UI 到仿真控制的命令调用。`load_config()`、`start()`、`pause()`、`step()`、`reset()`、`set_playback_rate()`、`inject_disturbance()`、`close()` 均为事件触发接口，不进入定时调度表。

默认调度如下：

| 顺序 | 调度块 | 默认频率 | 时间基准 | 触发条件 | 动作 |
| --- | --- | --- | --- | --- | --- |
| 1 | 编队算法 | `20 Hz` | sim-time | 每 10 个基础 tick | 读取当前模型状态；<br />从通信功能读取各节点 Inbox；<br />运行各节点编队算法；<br />生成控制量和 Outbox；<br />未触发时沿用上一帧控制量，不产生新的 Outbox |
| 2 | 通信功能 | `100 Hz` | sim-time | 每 2 个基础 tick | 接收编队算法新产生的 Outbox；<br />调用通信功能 `tick(dt_s)`；<br />推进在途消息、延迟和丢包；<br />更新各节点 Inbox |
| 3 | 模型功能 | `200 Hz` | sim-time | 每个基础 tick | 将当前有效控制量写入模型；<br />调用加扰 `tick(time_s, dt_s)` 并把动态扰动写入模型 / 通信；<br />调用模型迭代 `step(step_s)` 推进动力学；<br />更新时间、运行状态和控制回报 |
| 4 | 关键数据记录 | `10 Hz` | sim-time | 每 `0.1 s` 采样点，默认每 20 个 `0.005 s` 基础 tick | 写定时关键数据快照；不同播放倍率下采样点保持一致 |
| 5 | 显示快照生成 | `10 Hz` | wall-clock | wall-clock 节流命中时，或强制产帧 / 日志采样点 / 仿真结束 | 生成最新 `SimulationSnapshot`；显示用快照不随播放倍率升高 |
| 6 | 显示回显刷新 | `10 Hz` | wall-clock | wall-clock 节流命中时 | 更新可供 UI 读取的最近快照；若启用订阅，则通知订阅者 |

频率约束：

- 各 sim-time 频率优先选取能被基础仿真 tick 整除的值，避免分数 tick 调度；本版关键数据记录固定以 `0.1 s` 采样点为准，要求 `step_s <= 0.1 s`。若步长不能整除 `0.1 s`，则在跨过采样点后的第一个基础 tick 记录，但一个 tick 不会跨过两个采样边界。
- 显示回显刷新按 wall-clock 节流，不随 `playback_rate` 增大而提高刷新频率。
- `playback_rate` 只影响 wall-clock 调度间隔，不改变任何 sim-time 频率。
- 编队算法控制周期由仿真控制统一计算并注入算法实体：`control_period_s = step_s * algorithm_decimation`。默认 `step_s=0.005`、`algorithm_decimation=10`，因此默认控制周期为 `0.05 s`（20 Hz）。
- `algorithm_decimation` 可配置，必须为正整数；修改它会改变算法控制周期和 PID 积分步长，属于控制动态变化，不是播放倍率或 UI 刷新变化。
- 若某调度块本次未触发，应复用上一次有效输出，例如控制量保持；消息类输出不重复生成。
- 频率可配置，但默认值先按上表实现。

时序约束：

- 同一基础 tick 内算法读取的是本步开始时的模型 / 通信状态。
- 模型迭代在算法输出和扰动处理之后推进。
- 通信功能只理解消息 envelope，不理解 payload 内算法语义。
- 加扰是动态扰动进入模型 / 通信的唯一通道。

## 8. 对内模块接口

本节定义仿真控制期望各内部模块提供的最小接口，用于模块边界、调用顺序和异常处理对齐。

### 8.1 配置加载

配置加载组件负责读取配置文件、解析 JSON/YAML、校验运行所需字段，并把结构化配置交给仿真控制初始化各模块。

```python
class ConfigLoader:
    def load(path: str) -> dict[str, object]: ...
    def validate(config: dict[str, object]) -> None: ...
```

### 8.2 模型迭代

```python
class ModelIterator:
    def init(config: dict[str, object], seed: int) -> None: ...
    def read_states() -> dict[str, AircraftState]: ...
    def apply_controls(controls: dict[str, AccelerationCommand]) -> None: ...
    def step(dt_s: float) -> None: ...
    def reset() -> None: ...
    def close() -> None: ...
```

动态扰动入口只给加扰模块使用，仿真控制不直接调用：

```python
def inject_wind(command: object) -> None: ...
```

节点健康状态由加扰模块自行管理，不再下发给模型层。

### 8.3 通信功能

通信功能负责维护节点收件箱、在途消息队列和方向级链路状态。仿真控制向通信功能下发节点列表与链路配置，并在算法节拍与通信节拍之间搬运 outbox / inbox。

```python
class CommunicationEngine:
    def init(topology_config: dict[str, object], qos_config: dict[str, object], seed: int) -> None: ...
    def read_inbox(node_id: str) -> list[MessageEnvelope]: ...
    def send(messages: list[MessageEnvelope]) -> None: ...
    def tick(dt_s: float) -> None: ...
    def read_link_states() -> list[LinkRuntimeState]: ...
    def reset() -> None: ...
    def close() -> None: ...
```

动态扰动入口只给加扰模块使用：

```python
def inject_link_fault(command: object) -> None: ...
def inject_link_qos(command: object) -> None: ...
```

### 8.4 编队算法

编队算法实例总数 = **飞行实体 ×N（N≥1）** + **非飞行协调实体 ×0/1**：飞行实体承载编队、制导、控制律并产出 control；集中式协调能力按需以单元寄宿在某飞行实体内（领航-跟随寄宿长机），或独立成一个非飞行协调实体（地面站 / 虚拟节点 / 参考节点，见 `0-架构HLD` 3.3）。仿真控制对所有实体使用同一套统一契约，不再区分协调 / 节点两种类型，**飞行 / 非飞行由 `config.entity_type` 区分**（`flight` 绑定 `aircraft_id`、产 control；`coordination` 不绑、不产 control）。

```python
class FormationAlgorithm:
    def init(entity_id: str, config: dict[str, object]) -> None: ...  # config.entity_type ∈ {flight, coordination}
    def step(context: FormationAlgorithmContext) -> FormationAlgorithmOutput: ...
    def reset() -> None: ...
    def close() -> None: ...
```

`FormationAlgorithmOutput` 字段：

- `control`: 本实体控制量；**`entity_type == coordination`（非飞行）时此字段为空**，仿真控制按 `entity_type` 判定，只对飞行实体写模型。
- `selfCmd`: 本实体目标运动状态，来自对象组位置解算结果，仿真控制转写为 `NodeState` 的目标位置/目标速度字段。
- `controlDiag`: 本实体本拍控制诊断量，来自对象组汇总的小模块 `diag` 输出，仿真控制转写为 `NodeState` 的误差字段。
- `outbox`: 本实体要发送的消息（含协调单元广播的任务 / 队形指令）。
- `status`: 算法状态摘要，供日志和控制回报使用。

> 协调能力寄宿在飞行实体内时，飞行与协调在同一实体对象内共享状态、直接接线，不经仿真控制来回搬运；实体之间的数据才走通信功能。详见 `3-编队算法设计文档/`。

### 8.5 加扰

加扰组件负责解释动态扰动命令、维护扰动持续时间、更新节点健康状态，并把风场、链路中断和链路丢包率等影响写入模型或通信模块。

```python
class DisturbanceEngine:
    def init(config: dict[str, object], seed: int, model: ModelIterator, comm: CommunicationEngine) -> None: ...
    def inject(command: DisturbanceCommand) -> None: ...
    def tick(time_s: float, dt_s: float) -> list[SimulationEvent]: ...
    def clear() -> None: ...
    def reset() -> None: ...
    def close() -> None: ...
```

### 8.6 关键数据日志

关键数据日志组件负责按采样节拍记录运行快照和诊断事件，并保存本次运行配置。

```python
class DataLogger:
    def open(run_id: str, config: dict[str, object]) -> None: ...
    def write_snapshot(snapshot: SimulationSnapshot) -> None: ...
    def write_event(event: SimulationEvent) -> None: ...
    def flush() -> None: ...
    def close() -> None: ...
```

关键数据日志与 UI 事件日志分开：

- 关键数据日志是定时记录的仿真数据，固定 `10 Hz`，由仿真控制按 sim-time 调度调用 `write_snapshot()`；播放倍率只改变单位墙钟内推进的仿真时间，不改变日志采样点。
- 记录对象为 `SimulationSnapshot` 的关键数据子集，至少包含 `time_s`、`run_state`、节点状态和链路状态；节点状态包含位置/速度指令与控制误差，供后处理和未来 UI 使用；`step_s`、`route`、`route_segments` 不写入关键数据日志。事件对象只通过 `write_event()` 作为诊断信息记录，不参与 10Hz 定时采样。
- GUI 顶部“日志”窗口只展示 `SimulationEvent` 最近事件，不读取关键数据日志文件，也不决定关键数据日志频率。
- 日志组件在首次实际推进仿真时在工作目录下创建 `logs/<run-id>/`，其中 `snapshots.jsonl` 记录 10Hz 关键数据快照，`events.jsonl` 记录诊断事件，`config.json` 保存本次运行配置；内存列表用于测试和运行期查询。
- 日志落盘时按字段语义做十进制四舍五入：时间类字段保留 `3` 位小数，位置/距离和速度类字段保留 `2` 位小数，加速度类字段保留 `3` 位小数，过载类字段保留 `4` 位小数，角度类字段保留 `2` 位小数；仿真内部状态不因日志格式截断。

日志写入失败策略：

- 非关键实时快照写失败：记录 `WARN`，仿真可继续。
- run 元数据、配置、最终指标写失败：按异常场景处理。

## 9. 错误码

`ResultCode` 的语义如下：

| 错误码 | 场景 |
| --- | --- |
| `OK` | 成功 |
| `ERR_NO_CONFIG` | 未加载配置 |
| `ERR_CONFIG_NOT_FOUND` | 配置文件不存在 |
| `ERR_CONFIG_INVALID` | 配置内容非法 |
| `ERR_INVALID_STATE` | 当前状态不允许该命令 |
| `ERR_INVALID_ARGUMENT` | 参数非法 |
| `ERR_BUSY` | 当前运行中，不能执行该命令 |
| `ERR_MODULE_INIT_FAILED` | 内部模块初始化失败 |
| `ERR_TICK_FAILED` | tick 推进失败 |
| `ERR_LOG_FAILED` | 关键日志写入失败 |
| `ERR_INTERNAL` | 未分类内部错误 |

## 10. 线程与时间策略

- 仿真控制核心逻辑按单线程状态机设计，先保证确定性。
- UI 只调用事件触发的命令接口并消费回显快照；运行态推进由仿真控制的调度器触发内部 `_tick()`。
- 公共 `step()` 只用于暂停态 / 准备态下的人工单步。
- headless 模式使用同步循环，不依赖 GUI event loop。
- 后台线程与命令接口必须串行化进入仿真控制事件队列，避免 UI 线程和仿真线程同时修改状态。
- `step_s` 是基础 tick 的仿真时间步长；`playback_rate` 决定 wall-clock 调度间隔：`wall_interval_s = step_s / playback_rate`。

## 11. 快照生成策略

- UI 刷新快照低于仿真 tick 频率，默认每 `0.1s` wall-clock 生成 / 推送一次；该频率不随播放倍率增加。
- 日志快照按仿真时间 `0.1s` 采样点写入，不等同 UI 刷新节拍；二者默认同为 10Hz，但时间基准不同。
- 渐隐轨迹缓存由 UI 通过 `read_timed_snapshots()` 增量维护；普通显示快照中的当前节点位置仅作为队列外实时端点，不能按墙钟轮询频率写入历史队列。
- 快照对象生成后应视为不可变，避免 UI 渲染过程中被后台修改。

## 12. 模块归属

仿真控制负责稳定的应用层接口和调度闭环。配置加载、动态扰动和关键数据日志可以作为仿真控制内部组件实现，也可以按数据组 / 环境组边界拆分为独立模块；拆分后的接口语义保持本章定义不变。CLI 入口只包装配置加载与 `run_until_complete(config)`，不改变仿真控制的应用层契约。

## 13. 关联代码

- `src/runner/sim_control.py`
- `src/ui/gui/main_window.py`
- `src/environment/model.py`
- `src/environment/comm.py`
- `src/data/config_loader.py`
- `src/data/logger.py`
- `src/environment/disturb.py`
