# 流程库 LLD

> 流程库是一组**碰 `Mode` 的流程类**（与算法库平级，区别在于它产 / 用 / 切任务模态；分界见 `0-HLD.md` 原则 9）。
> 与算法库同构：每个流程单元都是一个策略族，按“共性父类 / 差异子类、输入下沉 / 输出族级”组织，选哪个实现由实体挂接时决定。

## 1. 范围与包结构

目标包结构：

```text
src/algorithm/flow_lib/
├── types.py          # Mode/MissionCommand/FormationCommand/Route 等流程叶类型
├── inbound.py        # Inbound 族
├── outbound.py       # Outbound 族
├── orchestrate.py    # Orchestrate 族
└── trajectory.py     # TrajectoryPlan 族
```

本轮流程库包含 4 个策略族：

| 族 | 本轮实现 | 角色 | 输出契约 |
| --- | --- | --- | --- |
| `Inbound` | 领航广播解析 | 僚机 | 写 `host_state / mode / formation_command` |
| `Outbound` | 领航广播发送 | 长机 | 写 `outbox` |
| `Orchestrate` | 常量保持编排 | 长机 | 写 `mode / formation_command` |
| `TrajectoryPlan` | 航线推进 / 空规划 | 长机 / 僚机 | 写 `segment` |

流程单元允许读写 `Mode`，但不直接实现控制律、误差计算或位置几何。

## 2. 流程叶类型

```python
class ModeEnum(str, Enum):
    GATHER = "集结"
    HOLD = "保持"
    RECONFIGURE = "重构"

@dataclass
class Mode:
    phase: ModeEnum

@dataclass
class FormationCommand:
    shape: str

@dataclass
class MissionCommand:
    command: str
    params: dict[str, object]

@dataclass
class Route:
    waypoints: list[Waypoint]
    default_speed_mps: float
```

本轮实际只使用 `Mode(HOLD)` 和 `FormationCommand(shape=<default_shape>)`。

## 3. `Inbound` 族

### 3.1 族契约

`Inbound` 是唯一例外签名：

```python
def step(self, inbox: list[MessageEnvelope]) -> None: ...
```

`inbox` 是用完即弃输入，不进入黑板。输出槽：

| 输出槽 | 说明 |
| --- | --- |
| `host_state` | 从长机广播得到的 `NavState` |
| `mode` | 从广播得到的编队模式 |
| `formation_command` | 从广播得到的队形指令 |

### 3.2 领航广播解析

只消费满足以下条件的消息：

| 字段 | 条件 |
| --- | --- |
| `topic` | `"lead_broadcast"` |
| `source` | 等于静态配置 `leader_id` |
| `target` | 当前实体 ID 或 `"broadcast"` |
| `payload` | 包含“编队模式 / 长机运动状态 / 队形指令”三个逻辑字段 |

若同一 inbox 中有多条有效广播，选 `timestamp` 最大的一条；若 timestamp 相同，保留列表中最后一条。

逻辑字段与序列化 key 的映射：

| 逻辑字段 | payload key | 说明 |
| --- | --- | --- |
| `编队模式` | `mode` | `Mode` |
| `长机运动状态` | `leader_state` | `NavState` |
| `队形指令` | `formation_command` | `FormationCommand` |

payload 序列化建议使用 ASCII 字段名，避免 Python/C 边界和 JSON schema 混杂：

```json
{
  "mode": {"phase": "保持"},
  "leader_state": {
    "node_id": "A01",
    "x_m": 140.0,
    "y_m": 260.0,
    "altitude_m": 1200.0,
    "speed_mps": 5.2,
    "theta_rad": 0.0,
    "psi_rad": 0.0
  },
  "formation_command": {"shape": "wedge"}
}
```

解析规则：

- payload 字段缺失或类型非法：忽略该消息，记录 `read_state()` 可见的 `last_error`，不抛运行期异常。
- 无有效消息：不写 `host_state / mode / formation_command`，沿用上次有效值。
- 首拍无广播：使用实体 `init` 播种初值。

状态：

| 状态 | 说明 |
| --- | --- |
| `last_valid_timestamp_s` | 最近一次采用的广播时间戳 |
| `last_receive_time_s` | 本地仿真时间，可从黑板 `time_s` 输入视图读取 |
| `dropped_count` | 因 topic/source/payload 不匹配被忽略的消息数 |
| `last_error` | 最近一次 payload 解析错误 |

新鲜度降级本轮不做控制动作，只保留状态口径；触发阈值见 `1-LLD综述.md` TODO。

## 4. `Outbound` 族

### 4.1 族契约

输入槽：

| 输入槽 | 说明 |
| --- | --- |
| `formation_command` | 当前队形指令 |
| `host_state` | 长机自身运动状态 |
| `mode` | 当前编队模式 |
| `time_s` | 当前仿真时间，用作消息时间戳 |

输出槽：

| 输出槽 | 说明 |
| --- | --- |
| `outbox` | `list[MessageEnvelope]` |

### 4.2 领航广播发送

长机每个协调节拍生成一条广播消息：

```python
MessageEnvelope(
    topic="lead_broadcast",
    source=leader_id,
    target="broadcast",
    timestamp=time_s,
    payload={
        "mode": {"phase": mode.phase},
        "leader_state": nav_state_to_payload(host_state),
        "formation_command": {"shape": formation_command.shape},
    },
)
```

`time_s` 由实体 `_ingest(ctx)` 写入黑板，`Outbound` 通过只读输入视图读取；不要用临时属性绕过统一 `step(self)` 约束。

广播展开边界：

- 算法只产 `target="broadcast"` 的 envelope。
- 是否展开成多个目标、是否受链路拓扑 / QoS 影响，是通信功能职责。
- 当前 `src/runner/sim_control.py` stub 会过滤 `broadcast` 后直接丢弃，实施接入时需要先修通信广播语义或在仿真控制适配层展开。

协调抽帧：

- 本轮默认每个算法拍都广播一次。
- 若配置 `broadcast_period_ticks > 1`，`Outbound` 持有 `tick_counter`，仅在计数命中时写 outbox，否则写空列表。
- 抽帧不改变飞行流执行频率。

## 5. `Orchestrate` 族

### 5.1 族契约

输入槽：

| 输入槽 | 说明 |
| --- | --- |
| `mission_command` | 仿真控制注入的任务命令，本轮可为 `None` |
| `mode` | 上一拍模式 |

输出槽：

| 输出槽 | 说明 |
| --- | --- |
| `mode` | 本拍模式 |
| `formation_command` | 本拍队形指令 |

同槽读写 `mode` 依赖执行顺序：本轮 `Orchestrate` 在拓扑序前段运行，先读到上一拍值，再写本拍值。

### 5.2 常量保持编排

本轮实现：

```text
mode = HOLD
formation_command.shape = default_shape
```

`mission_command` 暂不改变模式，但应保留字段和状态，便于后续真实编排替换：

- 若 `mission_command` 非空但本实现不支持，记录 `last_ignored_command`。
- 不抛异常，避免 UI 未来先接入命令按钮时破坏保持场景。

### 5.3 未来真实编排预留

真实编排上线时再实现：

- `mission_command` 生命周期：仿真控制保持 `hold_ticks` 拍后自动清 `None`。
- 编排按 `None -> 非None` 上升沿幂等消费。
- 编排内部状态防止模态回环，例如 `集结 -> 保持 -> 分散` 不自发倒回集结。
- 集结 / 重构需要全队视图时，长机也挂 `Inbound` 消费僚机遥测。

## 6. `TrajectoryPlan` 族

### 6.1 族契约

输入槽：

| 输入槽 | 说明 |
| --- | --- |
| `mode` | 当前编队模式 |
| `segment` | 上一拍航段 |
| `own_state` | 本体运动状态 |
| `distance_to_go_m` | 上一拍待飞距 |

输出槽：

| 输出槽 | 说明 |
| --- | --- |
| `segment` | 本拍航段 |

`TrajectoryPlan` 属于流程库，因为它读 `mode` 并决定当前应使用哪段轨迹；具体目标点插值仍在算法库 `PositionSolve`。

### 6.2 航线推进（长机）

静态参数：`route.waypoints`、`default_speed_mps`、`switch_distance_m`。

航点转航段：

```text
segment[i] = waypoint[i] -> waypoint[i+1]
speed = waypoint[i+1].speed_mps or route.default_speed_mps
```

推进规则：

```text
if distance_to_go_m <= switch_distance_m and current_index < last_segment:
    current_index += 1
segment = segments[current_index]
```

默认：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `switch_distance_m` | 20.0 | 距离终点小于该值切下一段 |
| `default_speed_mps` | 5.0 | 配置缺省速度 |

边界：

- 少于 2 个航点：`init` 失败；本轮不在运行期构造虚拟航段。
- 到达最后一段终点后：保持最后一段，`PositionSolve` 会把待飞距压到 0；是否盘旋 / 悬停不是本轮内容。
- `mode != HOLD`：本轮仍输出当前航段，并在 `read_state()` 标记 `unsupported_mode`；真实集结 / 重构实现上线前不切换算法。

状态：

| 状态 | 说明 |
| --- | --- |
| `current_index` | 当前航段序号 |
| `switch_count` | 航段切换次数 |
| `last_distance_to_go_m` | 最近一次读到的待飞距 |

### 6.3 空规划（僚机）

僚机保持场景不需要独立航线规划，但保留 `TrajectoryPlan` 族位。空规划实现：

- 读取 `mode / segment / own_state / distance_to_go_m`，允许未来替换。
- 输出一个稳定空航段或沿用上一拍 `segment`。
- 不影响槽位几何；僚机 `PositionSolve` 不读 `segment`。

推荐输出：

```text
segment.start = own_state.position
segment.end = own_state.position
segment.speed_mps = own_state.speed_mps
```

这样调试快照里不会出现未初始化航段。

## 7. 配置建议

流程库相关配置示例：

```json
{
  "algorithm": {
    "leader_id": "A01",
    "default_mode": "保持",
    "default_shape": "wedge",
    "broadcast_period_ticks": 1,
    "route": {
      "default_speed_mps": 5.0,
      "switch_distance_m": 20.0,
      "waypoints": [
        {"x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0, "speed_mps": 5.2},
        {"x_m": 800.0, "y_m": 260.0, "altitude_m": 1200.0}
      ]
    }
  }
}
```

配置校验应在 `init` 完成，运行期不接受结构错误。

## 8. 单元测试建议

| 单元 | 最小测试 |
| --- | --- |
| `LeadBroadcastInbound` | 忽略错误 topic/source；多条消息选最新；无消息保持旧值 |
| `LeadBroadcastOutbound` | payload 字段完整；抽帧时非命中拍 outbox 为空 |
| `ConstantHoldOrchestrate` | 始终输出 `保持` 和默认队形；未知 mission command 被记录但不抛异常 |
| `RouteTrajectoryPlan` | 待飞距低于阈值后切下一段；最后一段不越界；少于 2 航点 init 失败 |
| `NullTrajectoryPlan` | 输出稳定空航段，不影响已有黑板其他槽 |

这些测试只依赖黑板和 `MessageEnvelope`，不依赖仿真控制调度。

## 9. TODO

- 通信功能补 `target="broadcast"` 的拓扑展开和 QoS 语义，或在仿真控制接入层临时展开。
- 真实编排：`mission_command` 上升沿消费、集结 / 保持 / 重构状态机、全队就位判据。
- 僚机上行遥测：新增 topic、长机挂 `Inbound`、黑板承载全队视图。
- 新鲜度 / 降级：`Inbound` 输出 age，`Tracking` 或流程切换按阈值限速 / 保航向 / 停控。
- 动态队形 / 槽位分配：集结 / 重构需要在线分配时新增编排侧单元。
