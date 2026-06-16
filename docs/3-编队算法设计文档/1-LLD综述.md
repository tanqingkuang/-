# 编队算法 LLD 综述

> 本册统管三个 LLD（实体组 / 算法库 / 流程库）的**接口契约**与**统一数据契约**，并汇总跨文档 TODO。
> 架构见 `0-HLD.md`；各模块实现见对应 LLD；方案组合见 `5-用例-领航跟随保持.md`。

## 1. 统一实体契约（对仿真控制）

仿真控制对所有实体用同一套契约，不区分协调 / 节点（见 `../1-仿真控制HLD.md` §8.4）：

```python
class FormationAlgorithm:
    def init(entity_id: str, config: dict[str, object]) -> None: ...  # config.entity_type ∈ {flight, coordination}
    def step(context: FormationAlgorithmContext) -> FormationAlgorithmOutput: ...
    def declared_topics() -> list[MessageTopicSchema]: ...
    def reset() -> None: ...
    def close() -> None: ...
```

- 实体总数 = **飞行实体 ×N** + **非飞行协调实体 ×0/1**；由 `config.entity_type` 区分（`flight` 绑 `aircraft_id`、产 control；`coordination` 不绑、不产 control）。

## 2. 数据契约（编队算法 ↔ 仿真控制）

代码实现可用 dataclass / TypedDict 承载。锚定领航-跟随、3 自由度。

### 2.1 NavState — 注入算法的本机导航视图

由仿真控制从 `../2-模型迭代HLD.md` 的 `AircraftState` 投影得到，只保留真实机载导航可得量；**不含**模型内回路状态 `ax/ay/az(+rate)` 与质点输入 `nx/nz/phi`。噪声 / 漂移叠加在此投影层。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `node_id` | `str` | 本机 ID |
| `x_m / y_m / altitude_m` | `float` | 东 / 北 / 天 位置（E/N/U，沿用 `AircraftState` 工程字段名） |
| `speed_mps` | `float` | 速度 V |
| `theta_rad` | `float` | 航迹倾角 |
| `psi_rad` | `float` | 航向（`0`=东，`π/2`=北，与 2- 一致） |

> 速度分量 `vx/vy/vz` 可由 `V/θ/ψ` 派生，按需提供，不单列字段。

### 2.2 FormationAlgorithmContext — `step` 入参

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `time_s` | `float` | 当前仿真时间 |
| `dt_s` | `float` | 本实体步长（多速率下为该实体节拍） |
| `self_state` | `NavState` | 本机导航视图 |
| `inbox` | `list[MessageEnvelope]` | 上一轮派发到本实体的消息（僚机：长机广播；分布式：邻居） |
| `mission_command` | `MissionCommand \| None` | 仅协调单元消费：场景 / UI 经仿真控制注入的任务触发；本轮恒"保持"，可为 `None` |

> 静态参数（控制增益、队形几何表、`entity_type` / 槽位号等）走 `init(entity_id, config)`，不进 context。

### 2.3 FormationAlgorithmOutput — `step` 返回

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `control` | `AccelerationCommand \| None` | ENU 三轴期望加速度，对齐 2- 的 `AccelerationCommand{ax/ay/az_cmd_mps2}`；`entity_type==coordination` 时为 `None` |
| `outbox` | `list[MessageEnvelope]` | 本实体要发的消息 |
| `status` | `AlgorithmStatus` | 摘要：当前模态、关键误差（侧偏 / 待飞距等），供日志与控制回报 |

### 2.4 消息 envelope

通信功能只认通用 `MessageEnvelope{topic, source, target, timestamp, payload}`，不解析 payload。各方法的 payload 由算法插件自声明——领航-跟随的 `lead_broadcast` payload 见 `5-用例-领航跟随保持.md`。

## 3. 三个 LLD 的接口规范

各 LLD 内部单元统一遵循一份小协议（既服务落盘 / 重置，又对应 C 的结构体生命周期函数）：

| 模块 | 单元形态 | 小协议 |
| --- | --- | --- |
| **实体组**（`2-实体组LLD`） | 实体类，持状态；实例化并组合库单元；`step` 编排 | `init / step / read_state / reset / close` |
| **算法库**（`3-算法库LLD`） | 类，**不感知状态机**的纯计算；实例态在 `self`、无全局态；按需声明 working-state | `init(params) / step(inputs)->outputs / reset` |
| **流程库**（`4-流程库LLD`） | 类，**感知状态机**的流程 | 同上小协议 |

- 可重入 = 实例化（N 僚机 = N 实例）；C 移植 = 对象 ↔ 结构体 + 接收结构体指针的函数。
- 实体内单元间**显式传参（链式）**，不引入共享黑板（本轮单模态）。

## 4. TODO 总表（跨文档）

| 项 | 归属 | 触发 |
| --- | --- | --- |
| 编排 / 执行抽离 | 实体组 / 流程库 | 出现模态决策 / 异构僚机时 |
| 黑板 / 动态数据上下文 | 实体组 | 与"编排抽离"连体，重连出现再评估 |
| 算法库各单元数学（方程 / 参数 / 限幅） | `3-算法库LLD` | 实现阶段逐个补 |
| 流程库各单元实现 | `4-流程库LLD` | 实现阶段逐个补 |
| 扩展性压测 | 全模块 | 加第二个方案时 |

**已定（非 TODO）**：统一实体契约（§1）、数据契约（§2）、control = `AccelerationCommand`（ENU）、长机→僚机 payload（见 5-用例）、静态数据走构造函数入参。
</content>
