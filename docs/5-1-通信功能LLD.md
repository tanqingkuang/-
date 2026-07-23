# 通信功能 LLD

## 1. 定位

通信功能属于仿真环境组，模拟编队内无人机之间的无线链路。负责消息的路由、延迟仿真和丢包仿真，不关心消息内容语义。

## 2. 职责

- 接收算法层（经仿真控制转发）写入的待发消息。
- 严格按消息的 `target` 字段送达，不做路由决策（路由由算法层决定）。
- 按链路配置的丢包率随机决定是否丢弃该消息；未配置的链路视为不通，消息静默丢弃。
- 未丢弃的消息进入该链路的在途队列，并携带剩余延迟计数。
- 每个仿真 tick 推进所有在途队列的延迟倒计时，到期消息移入目标节点的收件箱。
- tick 时检查链路故障是否到期，到期自动恢复为 `normal`。
- 向仿真控制提供各节点的消息读取接口。
- 向仿真控制提供链路状态查询接口，用于生成仿真快照。
- 接受加扰模块注入的链路故障（断链/恢复）。

## 3. 边界

- 不读取全局配置文件，拓扑和 QoS 配置由仿真控制在 `init` 时下发。
- 不解析 `MessageEnvelope.payload` 内容。
- 不持有算法或加扰模块的引用，只暴露被动接受接口。
- 不负责模型动力学或传感器噪声。
- 节点数量在 `init` 时固定，运行期不增删节点。
- 当前不仿真带宽限制。

## 4. 数据结构

### 4.1 MessageEnvelope（引用自 `src/common/envelope.py`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `source` | `str` | 发送方节点 ID |
| `target` | `str \| list[str]` | 接收方节点 ID；单播传字符串，多播传列表，`"broadcast"` 表示发给所有节点 |
| `topic` | `str` | 消息类型标签，通信模块不解析 |
| `timestamp` | `float` | 发送时刻（仿真时间，单位 s） |
| `payload` | `Any` | 消息内容，通信模块不解析 |

### 4.2 LinkConfig（内部，每条链路一个）

| 字段 | 类型 | 说明 |
|------|------|------|
| `canonical_link_id` | `str` | 来源配置的 link_id（如 `"A01-A02"`）；duplex 展开的两条记录共享同一值，供 `inject_*` 接口归一化查找 |
| `direction` | `str` | `duplex` / `simplex`；duplex 表示双向，simplex 表示仅 `link_id` 正向 |
| `latency_ms` | `float` | 链路延迟，单位 ms |
| `loss_rate` | `float` | 丢包率，范围 `[0.0, 1.0]` |
| `status` | `str` | `normal` / `lost` |
| `fault_until_s` | `float \| None` | 故障到期仿真时间；`None` 表示永久故障，有值时 `tick` 到期后自动恢复为 `normal` |

### 4.3 InFlightMessage（内部，在途队列元素）

| 字段 | 类型 | 说明 |
|------|------|------|
| `envelope` | `MessageEnvelope` | 原始消息 |
| `remaining_s` | `float` | 剩余延迟，单位 s；每次 tick 减去 dt_s，≤0 时移入 inbox |

### 4.4 LinkState（对外输出，供快照使用）

| 字段 | 类型 | 说明 |
|------|------|------|
| `link_id` | `str` | 格式 `源节点-目标节点`，如 `A01-A02` |
| `latency_ms` | `float` | 当前延迟 |
| `loss_rate` | `float` | 当前丢包率 |
| `status` | `str` | `normal` / `lost` |
| `frame_rate_hz` | `float \| None` | 运行级发送帧频限制；`None` 表示不限制 |

## 5. 接口定义

### 5.1 生命周期

```python
def init(config: dict, seed: int) -> None
```
- 从 `config["nodes"]` 提取节点 ID 列表，建立各节点收件箱。
- 从 `config["links"]` 建立链路表，含 `direction`、QoS 参数（latency_ms、loss_rate）；`duplex` 链路展开为两个方向，`simplex` 链路只建立 `link_id` 正向。
- 同一方向重复配置抛 `ValueError`；`duplex` 与任一已占用方向冲突时抛 `ValueError`；两条互为反向的 `simplex` 链路允许共存。
- `seed` 用于初始化丢包随机数生成器，保证批量仿真可复现。
- 保存链路配置的深拷贝作为 `_base_links` 基线快照，供 `reset()` 恢复使用。
- 节点和链路数量在此固定，运行期不增删。

```python
def reset() -> None
```
- 清空所有在途队列和收件箱。
- 将所有链路配置（含 latency_ms、loss_rate、status、fault_until_s）恢复到 `init` 时的基线，丢弃运行期注入的故障和 QoS 修改。
- 随机数生成器重置到初始 seed，`_time_s` 重置为 0。
- 清空上一轮发送帧相位；运行级发送帧频限制保持不变。
- 实现依赖 `init` 时保存的 `_base_links` 快照（对链路配置的深拷贝）。

```python
def close() -> None
```
- 释放内部资源。

### 5.2 拓扑管理

```python
def update_topology(config: dict) -> None
```
- 只更新已有链路的 `latency_ms` 和 `loss_rate`；不可增删链路和节点。
- 不修改 `status` 和 `fault_until_s`，运行期注入的故障状态不受影响；config 中若含 `status` 字段，忽略。
- `link_id` 归一化规则同 `inject_link_qos`：`duplex` 正向和反向等价，同时更新两条方向记录；`simplex` 只更新精确方向；未知 `link_id` 抛 `ValueError`。
- 同一次调用中若多个条目解析到同一方向记录，抛 `ValueError`；两条互为反向的 `simplex` 可在同一次调用中分别更新。
- **原子性**：先完整校验所有链路（QoS 合法性、链路存在性、重复检测），全部通过后再统一写入；任何校验失败均不修改当前状态。
- 已在途消息按原延迟继续推进，不受本次更新影响。
- 运行期修改的 QoS **不会**写入 `_base_links` 基线；调用 `reset()` 后恢复 `init` 时的参数。

### 5.3 主循环

```python
def send(messages: list[MessageEnvelope]) -> None
```
- 仿真控制将算法本拍 outbox 写入通信模块。
- 对每条消息依次执行以下步骤：
  1. 校验 `source`：若不在节点列表中，整条消息静默丢弃，跳过其余步骤。
  2. 展开 `target`：
     - `"broadcast"` → 展开为所有节点（除 source 自身）
     - `list[str]` → 去重保序后展开（重复项不报错）
     - 普通 `str` → 单播，直接处理
  3. 过滤非法目标：过滤掉未知节点和 source 自身（自发自收），静默丢弃。
  4. 对剩余每个目标节点独立执行：
     a. 链路不存在（未配置）：静默丢弃。
     b. 链路 `status == "lost"`：静默丢弃。
     c. 若设置运行级发送帧频，则按消息仿真时间戳限制该方向链路的发送节拍；同一批次、同一链路、同一时间戳的消息视为同一发送帧，共享限流结论。
     d. 按 `loss_rate` 随机决定是否丢弃；通过帧频限制但被概率丢弃的报文仍占用本次发送帧。
     e. 未丢弃：创建 `InFlightMessage`（`remaining_s = latency_ms / 1000.0`），写入该链路的在途队列。所有消息最早在下一个 `tick` 才入 inbox，保证帧因果关系。

```python
def tick(dt_s: float) -> None
```

- `dt_s` 必须为非 `bool` 的有限数字且 `> 0`，否则抛 `ValueError`（NaN/Inf/bool/非数字类型统一拒绝；零值会触发零延迟消息的特殊行为，负值导致仿真时间倒退）。
- `_time_s += dt_s`，累加内部仿真时间。
- 检查所有链路的 `fault_until_s`，到期则将 `status` 恢复为 `normal`。
- 遍历所有链路的在途队列，每条消息 `remaining_s -= dt_s`。
- `remaining_s <= 0` 的消息移入对应目标节点的收件箱；同一链路内保证 FIFO，跨链路同 tick 到达的顺序不保证。

```python
def read_inbox(node_id: str) -> list[MessageEnvelope]
```
- 返回该节点收件箱中的所有消息，并清空收件箱。
- 仿真控制将返回值作为算法下一轮的输入 Inbox。

### 5.4 状态查询

```python
def read_link_states() -> list[LinkState]
```
- 返回所有链路的当前状态，供仿真控制生成 `SimulationSnapshot`。
- 返回通信内部方向级链路状态：`duplex` 返回两条独立的 `LinkState`（如 `"A01-A02"` 和 `"A02-A01"`），`simplex` 只返回配置方向的一条 `LinkState`。
- 返回列表按 `link_id` 字典序排序，保证快照和测试结果稳定。

### 5.5 故障注入（加扰模块调用）

```python
def set_uncertainty_frame_rate_hz(frame_rate_hz: float) -> None
```

- 设置本次运行的全链路发送帧频限制，参数必须为有限正数。
- 该参数属于初始化不确定性基线；`reset()` 只清除上一轮限流相位，不撤销该值。
- 仅限制通信出口，不修改算法控制周期，也不修改通信模块的时延队列推进频率。

```python
def set_uncertainty_loss_rate(loss_rate: float) -> None
```

- 设置本次运行的全链路丢包率，同时更新运行态和 `reset()` 恢复基线。

```python
def set_uncertainty_latency_ms(latency_ms: float) -> None
```

- 设置本次运行的全链路时延，同时更新运行态和 `reset()` 恢复基线。
- 参数必须为有限非负数，仅影响设置后新发送的消息。

```python
def inject_link_fault(link_id: str, status: str, duration_s: float | None = None) -> None
```
- 将指定链路的 `status` 设为 `normal` / `lost`；其他值抛 `ValueError`。
- `lost` 时该链路之后的所有消息静默丢弃；已在途消息继续推进（物理上已发出）。
- `duration_s` 仅对 `status="lost"` 有效；`status="normal"` 时传入非 `None` 的 `duration_s` 抛 `ValueError`。
- `duration_s is not None` 且为非有限数（`NaN`/`Inf`）或 `<= 0` 时抛 `ValueError`（`None` 表示永久故障，不受此约束）。
- `duration_s=None` → 永久故障，直到手动调用 `inject_link_fault(link_id, "normal")` 恢复；手动恢复时同时清除 `fault_until_s`。
- `duration_s=10.0` → 内部设 `fault_until_s = self._time_s + duration_s`，`tick` 到期后自动将 `status` 恢复为 `normal` 并清除 `fault_until_s`。
- `link_id` 格式校验：必须恰好含一个 `-`，否则抛 `ValueError`；格式合法但未找到对应链路抛 `KeyError`。
- `duplex` 链路正向或反向 `link_id` 等价，故障同时作用于两个方向；`simplex` 只作用于精确方向，反向未配置时视为未知链路并抛 `KeyError`。

```python
def inject_link_qos(link_id: str, latency_ms: float | None, loss_rate: float | None) -> None
```

- 动态调整指定链路的延迟或丢包率，`None` 表示不改动该参数；非法值抛 `ValueError`。`latency_ms` / `loss_rate` 非 `None` 时须为有限数字（非 bool、非 NaN/Inf）且满足范围约束，否则抛 `ValueError`。
- 两个参数均为 `None` 时为 no-op，正常返回，不报错。
- `link_id` 归一化规则同 `inject_link_fault`：`duplex` 接受正向或反向且两者等价；`simplex` 只接受精确方向。
- 仅影响注入后新入队的消息，已在途消息不受影响。

## 6. 内部实现要点

### 6.1 数据布局

```
links: dict[(src, dst), LinkConfig]                 # 拓扑表，O(1) 查链路
in_flight: dict[(src, dst), list[InFlightMessage]]  # per-link 在途队列
inbox: dict[node_id, list[MessageEnvelope]]          # per-node 收件箱
nodes: list[str]                                     # 所有节点 ID，广播时使用
rng: numpy.random.Generator                          # 丢包随机数生成器
_seed: int                                           # 初始 seed，供 reset() 重新初始化 rng
_time_s: float                                       # 内部仿真时间，tick 时累加，用于故障到期判断
uncertainty_frame_rate_hz: float | None               # 全链路运行级发送帧频限制
last_frame_sent_s: dict[(src, dst), float]            # 各方向链路最近一次通过限流的帧时间
_base_links: dict[(src, dst), LinkConfig]            # init 时的链路配置快照（深拷贝），reset() 从此恢复
```

节点 ID 从 `config["nodes"]` 每项的 `node_id` 字段提取，`init` 时动态展开：

```python
nodes     = [n["node_id"] for n in config["nodes"]]
inbox     = {nid: [] for nid in nodes}
in_flight = {}  # 按需创建，key 为 (src, dst)
```

### 6.2 链路的处理

链路按 `direction` 展开为通信内部方向级记录：

```
config link_id "A01-A02", direction="duplex"
  →  links[("A01","A02")] = LinkConfig(canonical_link_id="A01-A02", direction="duplex", ...)
     links[("A02","A01")] = LinkConfig(canonical_link_id="A01-A02", direction="duplex", ...)

config link_id "A01-A02", direction="simplex"
  →  links[("A01","A02")] = LinkConfig(canonical_link_id="A01-A02", direction="simplex", ...)
```

`inject_*` 接口的 `link_id` 参数按方向解析：

```python
def _resolve_pair(link_id: str) -> list[tuple]:
    if link_id.count("-") != 1:
        raise ValueError(link_id)   # 0 个或 ≥2 个 - 均为非法格式
    a, b = link_id.split("-")
    key = (a, b)
    if key not in links:
        raise KeyError(link_id)
    if links[key].direction == "duplex":
        return link_index[links[key].canonical_link_id]  # 返回两个方向
    return [key]  # simplex 只返回精确方向
```

配置层（init / update_topology）按方向记录判重：`duplex` 占用两个方向；`simplex` 只占用自身方向，互为反向的两条 `simplex` 链路可共存。

### 6.3 send 流程

```python
for msg in messages:
    # 1. 校验 source
    if msg.source not in inbox:
        continue                              # 未知 source，丢弃

    # 2. 展开并规范化 target
    if msg.target == "broadcast":
        targets = [nid for nid in nodes if nid != msg.source]
    elif isinstance(msg.target, list):
        targets = list(dict.fromkeys(msg.target))  # 去重保序
    else:
        targets = [msg.target]

    # 3. 过滤非法目标
    targets = [dst for dst in targets
               if dst in inbox and dst != msg.source]

    for dst in targets:
        key = (msg.source, dst)
        cfg = links.get(key)

        if cfg is None:
            continue                              # 未配置链路，丢弃

        if cfg.status == "lost":
            continue                              # 断链，丢弃

        if rng.random() < cfg.loss_rate:
            continue                              # 命中丢包，丢弃

        # 替换 target 为实际目标节点 ID（MessageEnvelope 是 frozen dataclass，用 replace 创建新实例）
        delivered = dataclasses.replace(msg, target=dst)

        # 所有消息统一进在途队列，下一 tick 才入 inbox，保证帧因果关系
        delay_s = cfg.latency_ms / 1000.0
        in_flight.setdefault(key, []).append(InFlightMessage(delivered, delay_s))
```

### 6.4 tick 流程

```python
# 1. 累加内部时间
self._time_s += dt_s

# 2. 检查故障到期
for cfg in links.values():
    if cfg.fault_until_s is not None and self._time_s >= cfg.fault_until_s:
        cfg.status = "normal"
        cfg.fault_until_s = None

# 3. 推进在途消息
for key, queue in list(in_flight.items()):
    remaining = []
    for item in queue:
        item.remaining_s -= dt_s
        if item.remaining_s <= 0:
            inbox[key[1]].append(item.envelope)
        else:
            remaining.append(item)
    if remaining:
        in_flight[key] = remaining
    else:
        del in_flight[key]   # 清理空队列，避免长时间运行内存积累
```

## 7. 配置 Schema

通信模块从整体配置中读取以下字段：

```json
{
  "nodes": [
    { "node_id": "A01", "role": "leader" },
    { "node_id": "A02", "role": "wingman" },
    { "node_id": "A03", "role": "wingman" }
  ],
  "links": [
    {
      "link_id": "A01-A02",
      "latency_ms": 18.0,
      "loss_rate": 0.01,
      "status": "normal"
    }
  ]
}
```

未在 `links` 中配置的节点对视为不通，消息静默丢弃。

### 字段约束

**nodes 约束**

| 字段 | 类型 | 约束 | 缺省值 |
|------|------|------|--------|
| `node_id` | `str` | 非空；不含 `-`；不等于保留字 `"broadcast"`；`nodes` 列表内不可重复，重复抛 `ValueError` | 必填 |

**links 约束**

| 字段 | 类型 | 约束 | 缺省值 |
|------|------|------|--------|
| `link_id` | `str` | 格式 `"节点A-节点B"`，恰好含一个 `-`；两端节点须已在 `nodes` 中；不允许自环（A == B） | 必填 |
| `direction` | `str` | `"duplex"` \| `"simplex"`；`duplex` 双向，`simplex` 仅 `link_id` 正向 | `"duplex"` |
| `latency_ms` | `float` | 有限数字（finite），`>= 0` | 必填 |
| `loss_rate` | `float` | 有限数字（finite），`[0.0, 1.0]` | 必填 |
| `status` | `str` | `"normal"` \| `"lost"`（仅 `init` 校验；`update_topology` 忽略此字段，不校验） | `"normal"` |

`latency_ms` / `loss_rate` / `link_id` 不满足约束时，`init` 和 `update_topology` 均抛 `ValueError`。`direction` 和 `status` 约束仅在 `init` 时生效；`update_topology` 忽略这两个字段，即使传入非法值也不报错。

## 8. 错误处理策略

| 场景 | 处理方式 |
|------|----------|
| `init`：`config['nodes']` 缺失或不是列表；节点/链路条目不是 dict；必填字段（`node_id`、`link_id`、`latency_ms`、`loss_rate`）缺失或类型错误 | 抛 `ValueError` |
| `update_topology`：链路条目不是 dict；必填字段缺失或类型错误 | 抛 `ValueError` |
| `init`：`direction` 不在 `{"duplex", "simplex"}` | 抛 `ValueError` |
| `init` / `update_topology`：`latency_ms` / `loss_rate` 为非有限数（NaN/Inf）、`latency_ms < 0` 或 `loss_rate` 不在 `[0.0, 1.0]` | 抛 `ValueError` |
| `inject_link_qos`：`latency_ms` / `loss_rate` 非 `None` 且为非数字类型（含 `bool`）或非有限数或超出范围 | 抛 `ValueError` |
| `inject_link_fault`：`duration_s` 非 `None` 且为非有限数（NaN/Inf）或 `<= 0` | 抛 `ValueError` |
| `init`：`link_id` 两端节点相同（自环） | 抛 `ValueError` |
| `update_topology`：`link_id` 不在 init 基线内 | 抛 `ValueError` |
| `tick`：`dt_s` 为非数字类型（含 `bool`）、非有限数（NaN/Inf）或 `<= 0` | 抛 `ValueError` |
| `inject_link_fault`：`status` 不在 `{"normal", "lost"}` | 抛 `ValueError` |
| `inject_link_fault` / `inject_link_qos`：`link_id` 格式非法（不含或含多个 `-`） | 抛 `ValueError` |
| `inject_link_fault` / `inject_link_qos`：格式合法但未知 `link_id` | 抛 `KeyError` |
| `read_inbox`：未知 `node_id` | 抛 `KeyError` |
| `send`：未知 `source` 节点 | 静默丢弃 |
| `send`：未知 `target` 节点 | 静默丢弃 |
| `send`：`source == target`（自发自收） | 静默丢弃 |
| `send`：`target` 列表中有重复项 | 去重后处理，不报错 |
| `send`：空消息列表 | 无操作，正常返回 |

> 分界原则：初始化阶段配置错误快速失败（抛异常）；运行期非法消息静默丢弃（不中断仿真主循环）；`inject_*` 接口属于编程调用，未知 `link_id` 视为编程错误快速失败。

## 9. 主控接入约定

`CommunicationChannel` 已由 `src/runner/sim_control.py` 接入主仿真流程。主控侧需要承担以下适配职责：

- `init` 时将整体配置中的 `nodes` / `links` 传入通信模块，`seed` 用于通信丢包随机数。
- 算法 outbox 不过滤 `broadcast`，直接交给 `CommunicationChannel.send()`，由通信模块按链路和 QoS 展开投递。
- 加扰命令中 `link_fault` 映射为 `inject_link_fault(link_id, "lost")`；`link_loss` 映射为 `inject_link_qos(..., loss_rate=...)`，到期后恢复原丢包率。
- 通信模块 `read_link_states()` 返回方向级记录；`SimulationController` 生成 `SimulationSnapshot` 时按原始配置链路折叠回配置级记录，并补回 `direction` 字段供 GUI 使用。
- 主循环当前每 2 个仿真步调用一次 `comm.tick(step_s * 2)`；配置 `latency_ms` 时需考虑该推进粒度。

## 10. 关联代码

- `src/environment/comm.py`
- `src/common/envelope.py`
