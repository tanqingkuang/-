# 实体组 LLD

> 实体组是编队算法中**唯一持有状态**的模块：每个实体 = 一个对象，实例化并组合它需要的算法库 / 流程库单元，持有这些子对象即持有全部维护数据。
> 接口契约见 `1-LLD综述.md`；本册讲实现方法。

## 1. 范围与实体类型

本轮只实现飞机本体，且把协调能力寄宿在长机本体内；非飞行协调本体只保留接口位置。

| 实体 | `entity_type` | 组成 | 产 control | 本轮 |
| --- | --- | --- | --- | --- |
| 长机本体 | `flight` | 飞行单元 + 协调单元 | 是 | ✅ |
| 僚机本体 | `flight` | 飞行单元 | 是 | ✅ |
| 协调本体 | `coordination` | 仅协调单元，不飞 | 否 | 预留 |

实体组对仿真控制暴露统一 `FormationAlgorithm` 契约；仿真控制不关心实体内部挂了哪些单元，只按实体 ID 注入 `FormationAlgorithmContext` 并读取 `FormationAlgorithmOutput`。

## 2. 包与类结构

目标包结构如下（当前 `src/algorithm/coord` / `node` 是旧空壳，实施时迁移）：

```text
src/algorithm/
├── entities/
│   ├── base.py          # FormationAlgorithm 协议、Context/Output/Status、NavState 投影
│   ├── blackboard.py    # Context 黑板叶类型与默认值
│   ├── aircraft.py      # AircraftEntity 基类
│   ├── leader.py        # LeaderAircraft
│   └── wingman.py       # WingmanAircraft
├── algo_lib/
└── flow_lib/
```

建议类关系：

```python
class AircraftEntity(FormationAlgorithm):
    entity_id: str
    config: EntityConfig
    blackboard: Blackboard
    units: list[Unit]

class LeaderAircraft(AircraftEntity):
    orch: Orchestrate
    planner: TrajectoryPlan
    possolve: PositionSolve
    devcalc: DeviationCalc
    tracker: Tracking
    tx: Outbound

class WingmanAircraft(AircraftEntity):
    rx: Inbound
    planner: TrajectoryPlan
    possolve: PositionSolve
    devcalc: DeviationCalc
    tracker: Tracking
```

`AircraftEntity` 只承载公共生命周期、黑板、绑定校验和 `_ingest/_emit`；长机 / 僚机子类负责选择策略实现并定义固定拓扑序。

## 3. 黑板结构

每个实体持有一份 per-entity 黑板。黑板是实体内唯一过程数据存储，单元只通过已绑定的 In/Out 视图读写黑板槽。

```python
@dataclass
class Blackboard:
    time_s: float
    dt_s: float
    own_state: NavState
    host_state: NavState
    mode: Mode
    formation_command: FormationCommand
    segment: Segment
    target: Target
    deviation: Deviation
    control: AccelerationCommand
    distance_to_go_m: float
    mission_command: MissionCommand | None
    outbox: list[MessageEnvelope]
    status: AlgorithmStatus
```

字段口径：

| 黑板槽 | 语义 | 初值 |
| --- | --- | --- |
| `time_s` | 当前仿真时间，由 `_ingest(ctx)` 每拍覆盖 | 0 |
| `dt_s` | 当前算法步长，由 `_ingest(ctx)` 每拍覆盖 | 配置步长 |
| `own_state` | 本体运动状态，由 `_ingest(ctx)` 每拍覆盖 | 从首拍 `ctx.self_state` 或配置初始状态投影 |
| `host_state` | 主机运动状态；长机为自身，僚机来自广播 | 长机初值 = 自身；僚机初值 = 配置里的长机初始状态或自身近似值 |
| `mode` | 编队模式，本轮恒 `保持` | `保持` |
| `formation_command` | 队形指令，如 `wedge` / `line` | 配置默认队形 |
| `segment` | 当前航段 | 配置首航段；僚机可为空航段 |
| `target` | 待跟踪目标指令 | 由初始状态构造的零误差目标 |
| `deviation` | 三轴误差 | 全 0 |
| `control` | 三轴加速度命令 | 全 0 |
| `distance_to_go_m` | 待飞距反馈槽 | 首航段长度；僚机可为 0 |
| `mission_command` | 仿真控制注入的遥控指令 | `None` |
| `outbox` | 本实体本拍待发消息 | 每拍 `_ingest` 前清空 |
| `status` | 日志 / UI 摘要 | 当前 mode + 零误差 |

保持槽规则：

- `host_state / mode / formation_command` 由收消息或编排写；若本拍没有新消息，不覆写，沿用上次有效值。
- `segment / distance_to_go_m / mode` 是反馈槽，本轮靠固定顺序读到上拍值，不做双缓冲。
- `outbox` 是边界槽，每拍开始清空，`Outbound.step()` 写入，`_emit()` 读取。

## 4. 挂接与绑定

实体 `init(entity_id, config)` 做三件事：

1. **解析静态配置**：实体角色、飞机 ID、长机 ID、队形几何表、航线、网络拓扑、控制增益、限幅等。
2. **挂接单元**：为每个策略族选择一个实现类并实例化。情景差异只在此处体现，不散落到单元内部 `if/else`。
3. **绑定视图**：把各单元的 In/Out 视图指向黑板槽，并执行单写者校验和初值播种。

绑定建议使用显式 `BindSpec`，便于校验：

```python
@dataclass(frozen=True)
class BindSpec:
    unit_name: str
    reads: tuple[str, ...]
    writes: tuple[str, ...]
```

校验规则：

| 规则 | 说明 |
| --- | --- |
| 槽存在 | 每个 In/Out 指向的槽必须是 `Blackboard` 字段 |
| 单写者 | 除 `_ingest` 注入槽外，每个槽至多一个单元写 |
| 边界槽 | `outbox` 只允许 `Outbound` 写，`control` 只允许 `Tracking` 写 |
| 角色匹配 | 长机必须挂 `Orchestrate/Outbound`；僚机必须挂 `Inbound` 且不挂 `Outbound` |
| 初值完整 | 所有读槽在首拍执行前有非垃圾初值 |

## 5. 长机装配

长机本体在同一对象、同一黑板内同时跑飞行流和协调流。固定执行顺序：

| 顺序 | 单元 | 实现 | 读 → 写 |
| --- | --- | --- | --- |
| 0 | `_ingest` | 实体方法 | `ctx.time_s, ctx.dt_s, ctx.self_state, ctx.mission_command` → `time_s, dt_s, own_state, host_state, mission_command`；清 `outbox` |
| 1 | `Orchestrate` | 常量保持编排 | `mission_command, mode(t-1)` → `mode, formation_command` |
| 2 | `TrajectoryPlan` | 航线推进 | `mode, segment(t-1), own_state, distance_to_go_m(t-1)` → `segment` |
| 3 | `PositionSolve` | 航线插值 | `segment, own_state` → `target, distance_to_go_m` |
| 4 | `DeviationCalc` | 标准误差 | `target, own_state` → `deviation` |
| 5 | `Tracking` | PID 组合 | `deviation` → `control` |
| 6 | `Outbound` | 领航广播 | `formation_command, host_state, mode, time_s` → `outbox` |
| 7 | `_emit` | 实体方法 | `control, outbox, status` → `FormationAlgorithmOutput` |

长机 `_ingest` 必须同步写 `host_state = own_state`，避免 `Outbound` 广播到上一拍主机状态。

## 6. 僚机装配

僚机本体不消费 `mission_command`，模态来自长机广播。固定执行顺序：

| 顺序 | 单元 | 实现 | 读 → 写 |
| --- | --- | --- | --- |
| 0 | `_ingest` | 实体方法 | `ctx.time_s, ctx.dt_s, ctx.self_state` → `time_s, dt_s, own_state`；清 `outbox` |
| 1 | `Inbound` | 领航广播解析 | `ctx.inbox` → `host_state, mode, formation_command` |
| 2 | `TrajectoryPlan` | 空规划 | `mode, segment(t-1), own_state, distance_to_go_m(t-1)` → `segment` |
| 3 | `PositionSolve` | 槽位几何 | `formation_command, host_state` + 静态 `aircraft_id/formation_table` → `target` |
| 4 | `DeviationCalc` | 标准误差 | `target, own_state` → `deviation` |
| 5 | `Tracking` | PID 组合 | `deviation` → `control` |
| 6 | `_emit` | 实体方法 | `control, [], status` → `FormationAlgorithmOutput` |

`Inbound.step(inbox)` 是唯一带入参的单元。若 `inbox` 里没有有效 `lead_broadcast`，不得清空 `host_state/mode/formation_command`。

## 7. `_ingest` 与 `_emit`

`_ingest(ctx)`：

- 将仿真控制提供的 `AircraftState` 投影成 `NavState`，写 `own_state`。
- 写入 `time_s / dt_s`，供发消息时间戳、PID 积分等单元读取。
- 长机同时写 `host_state = own_state`。
- 写入 `mission_command`，僚机可忽略但字段仍可存在。
- 清空 `outbox`，防止上一拍消息重复发送。

`_emit()`：

- `entity_type == flight` 时输出 `control`；`coordination` 时输出 `None`。
- `outbox` 返回黑板当前列表的浅拷贝，避免仿真控制后续改动实体内部列表。
- `status` 至少包含：`mode`、`formation_command`、`position_error_norm_m`、`speed_error_mps`、`source_age_s?`（新鲜度本轮可缺省）。

## 8. 生命周期

| 接口 | 语义 |
| --- | --- |
| `init` | 创建黑板，实例化并挂接单元，绑定 In/Out 视图，播种反馈 / 保持槽 |
| `step` | 消费一拍 `FormationAlgorithmContext`，按固定拓扑序推进，返回输出 |
| `read_state` | 汇出实体与子单元内部状态；不得返回黑板可变对象本体 |
| `reset` | 回到 init 后状态；首版可整实体重建，保证 PID 积分等内部状态清零 |
| `close` | 释放资源；本轮通常为空实现 |

`read_state` 推荐结构：

```python
{
    "entity_id": "A02",
    "role": "wingman",
    "blackboard": {...snapshot...},
    "units": {
        "tracker": {"pid_x": {...}, "pid_y": {...}, "pid_z": {...}},
        "rx": {"last_valid_time_s": ...},
    },
}
```

## 9. 配置输入

实体配置建议由仿真控制按节点拆分后传入：

```json
{
  "entity_type": "flight",
  "role": "wingman",
  "leader_id": "A01",
  "formation": {
    "default_shape": "wedge",
    "slots": {
      "wedge": {
        "A02": {"forward_m": -40.0, "right_m": -35.0, "up_m": 0.0},
        "A03": {"forward_m": -40.0, "right_m": 35.0, "up_m": 0.0}
      }
    }
  },
  "route": [...],
  "tracking": {...},
  "network": {...}
}
```

静态配置走 `init`，不得每拍塞进 `FormationAlgorithmContext`。

## 10. 不变式

- 实体是状态所有者；库单元状态只存在于被实体持有的实例中。
- 无全局 / 类级可变态；同一类挂到多个实体时互不干扰。
- 实体内数据只走黑板和已绑定视图，不在 `step` 中临时拼装单元输入。
- 实体间数据只走 `MessageEnvelope`，不得直接读其他实体黑板或模型状态。
- 固定拓扑序是本轮正确性条件；任何重排都要重新检查反馈槽读写语义。

## 11. TODO

- **编排丰富 / 动态管线**：出现真实模态决策 / 异构僚机时，固定串联让位给按模态选管线。
- **反馈槽双缓冲**：动态管线重连后，`segment / distance_to_go_m / mode` 拆 `prev/cur`。
- **协调本体**：地面站 / 虚拟节点接入时补 `CoordinationEntity` 装配。
- **配置 schema**：与仿真控制配置文件合并时，补完整 JSON/YAML 字段与校验规则。
