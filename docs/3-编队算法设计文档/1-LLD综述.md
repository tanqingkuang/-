# 编队算法 LLD 综述

> 本册统管三个 LLD（实体组 / 算法库 / 流程库）的**接口契约**与**统一数据契约**，并汇总跨文档 TODO。
> 架构见 `0-HLD.md`；各模块实现见对应 LLD；方案组合见 `5-用例-领航跟随保持.md`。

# 一、对外 API（编队算法 ↔ 仿真控制）

> 把编队算法当黑盒：仿真控制唯一需要知道的就是这套接口与流动的数据。

## 1.1 统一实体契约（对仿真控制）

仿真控制对所有实体用同一套契约，不区分协调 / 节点（见 `../1-仿真控制HLD.md` §8.4）：

```python
class FormationAlgorithm:
    def init(entity_id: str, config: dict[str, object]) -> None: ...  # config.entity_type ∈ {flight, coordination}
    def step(context: FormationAlgorithmContext) -> FormationAlgorithmOutput: ...
    def reset() -> None: ...
    def close() -> None: ...
```

- 实体总数 = **飞行实体 ×N** + **非飞行协调实体 ×0/1**；由 `config.entity_type` 区分（`flight` 绑 `aircraft_id`、产 control；`coordination` 不绑、不产 control）。

## 1.2 数据契约（编队算法 ↔ 仿真控制）

代码实现可用 dataclass / TypedDict 承载。锚定领航-跟随、3 自由度。

### 1.2.1 NavState — 注入算法的本机导航视图

由仿真控制从 `../2-模型迭代HLD.md` 的 `AircraftState` 投影得到，只保留真实机载导航可得量；**不含**模型内回路状态 `ax/ay/az(+rate)` 与质点输入 `nx/nz/phi`。噪声 / 漂移叠加在此投影层。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `node_id` | `str` | 本机 ID，沿用 `AircraftState.node_id`；对 `flight` 实体即其 `aircraft_id`（亦即 `init` 的 `entity_id`） |
| `x_m / y_m / altitude_m` | `float` | 东 / 北 / 天 位置（E/N/U，沿用 `AircraftState` 工程字段名） |
| `speed_mps` | `float` | 速度 V |
| `theta_rad` | `float` | 航迹倾角 |
| `psi_rad` | `float` | 航向（`0`=东，`π/2`=北，与 2- 一致） |

### 1.2.2 FormationAlgorithmContext — `step` 入参

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `time_s` | `float` | 当前仿真时间 |
| `dt_s` | `float` | 本拍步长（本轮所有实体同 20 Hz，见 `0-HLD.md` §4 调度；真·多速率留半物理） |
| `self_state` | `NavState` | 本机导航视图 |
| `inbox` | `list[MessageEnvelope]` | 上一轮派发到本实体的消息 |
| `mission_command` | `MissionCommand \| None` | **仅注入承载协调能力的实体**（本轮长机）；其任务编排产 `Mode` 并经广播下发。飞行僚机**不消费**它，僚机 `Mode` 来自 leader 广播（`ParsedInbox.task`）。本轮恒"保持"，可为 `None` |

> 静态参数（增益、队形几何表、`entity_type` / 槽位号等）走 `init`，不进 context。

### 1.2.3 FormationAlgorithmOutput — `step` 返回

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `control` | `AccelerationCommand \| None` | ENU 三轴期望加速度，对齐 2- 的 `AccelerationCommand{ax/ay/az_cmd_mps2}`；`coordination` 时 `None` |
| `outbox` | `list[MessageEnvelope]` | 本实体要发的消息 |
| `status` | `AlgorithmStatus` | 摘要：当前模态、关键误差，供日志与控制回报 |

### 1.2.4 消息 envelope

通信功能只认通用 `MessageEnvelope{topic, source, target, timestamp, payload}`，不解析 payload。各方法的 payload 由配对的算法插件自行约定并拥有（领航-跟随见 `5-用例-领航跟随保持.md`）。

---

# 二、对内接口（实体内 单元 ↔ 单元）

> 翻进黑盒内部：库单元的统一框架、同族可换机制，以及各单元 `u/y` 的具体类型。

## 2.1 通用单元模板：`init / reset / step(u)->y / read_state`

算法库 / 流程库的每个单元都是一个类，遵循同一套模板：

```python
class Unit:                  # Protocol / 抽象基类
    def init(self, params) -> None: ...   # 拉起：注入静态参数（增益、几何表…）
    def reset(self) -> None: ...          # 回到初始（清内部状态）
    def step(self, u) -> "y": ...         # 推进一拍：吃输入 u，更新内部状态并返回本拍输出 y
    def read_state(self) -> dict: ...     # 仅供落盘 / 快照：读内部状态 x（≠ 输出 y）
    # 可选：close()
```

- **`init` / `reset` / `read_state` 是通用类型接口**：所有单元一致，实体可泛型地拉起、重置、遍历快照。
- **设计的真正内容在 `step` 的输入 `u` 与返回 `y`**——即 §2.4 逐单元定义的输入 / 输出。
- **`step(u)->y` 合一**（推进 + 返回本拍输出），与实体 `step(ctx)->Output` 上下一致：实体内单元按拓扑序逐个调用、每个 `step` 完即用其返回，不存在"只观察不推进"，故不另设 `get`。`read_state` 只服务落盘 / 快照（读内部状态 x，**不是**输出 y）。

## 2.2 抽象基类与策略（同族可换）

**同一"族"（同 `u`、同 `y`）的多个实现，用一个抽象基类约束**——这是策略模式的基础：实体只依赖基类，换实现（甚至运行期）不动实体代码。例（u/y 详见第二章）：

| 抽象基类（族） | 库 | 同族可换实现（示例） |
| --- | --- | --- |
| `PositionSolve`（位置解算） | 算法 | 航线插值（长机）/ 槽位几何（僚机）/（未来）Dubins |
| `DeviationCalc`（误差解算） | 算法 | 不同偏差口径 |
| `Tracking`（轨迹 / 编队跟踪） | 算法 | 不同通道组合（垂向 TECS / 单通道 PID…），**组合控制算法而成** |
| `ControlLaw`（控制算法） | 算法 | PID / L1 / ADRC（原子控制律，被 `Tracking` 组合调用） |
| `TrajectoryPlan`（轨迹规划） | 流程 | 航线推进（长机）/ 槽位选择（僚机）——**mode-aware** |

> **算法 / 流程分界 = 是否碰 `Mode`**（见 `0-HLD.md` 原则 9）：`Tracking`/`ControlLaw` 虽"不同模态用法不同"，但选哪个由外部流程切，自身不读状态机，故归算法。

> 因为"换实现不动实体"靠的就是同族 `u/y` 一致，所以 `u/y` 必须**提前定死**——这是 §2.4 的任务。

## 2.3 实体挂接与遍历

- **挂接（mount）**：实体 `init` 时按 config 实例化所需单元类、作为自身成员；同族可挂多个实例（如三轴各一个），各持各的状态、互不干扰。
- **编排**：实体 `step()` 内按**固定拓扑序**执行各单元；每个单元的 `u` 由 `step()` **显式组装**——可取自 `ctx` 字段与**一个或多个**上游单元的 `y`（本轮是一张编译期定死的静态 DAG，非严格线性链）。本轮单模态、显式接线，不引入共享黑板（理由见 `0-HLD.md` 原则 8）。
- **遍历**：实体持单元集合，`reset` / `read_state` 泛型遍历所有挂接单元。

```python
# 实体 step() 编排示意（统一 step(u)->y 模板；具体单元见 5-用例）
def step(self, ctx):
    parsed = self.rx.step(ctx.inbox)                       # 流程：收发(收)
    mode   = self.orch.step(parsed)                        # 流程：任务编排(僚机 Mode 来自广播, 恒"保持")
    plan   = self.planner.step((mode, parsed))             # 流程：轨迹规划
    target = self.possolve.step((plan, parsed.leader_nav)) # 算法：位置解算
    dev    = self.devcalc.step((ctx.self_state, target))   # 算法：误差解算
    accel  = self.tracker.step((dev, ctx.self_state))      # 算法：跟踪(内部调控制算法)
    return FormationAlgorithmOutput(control=accel, outbox=[], status=self._status())
```

## 2.4 各单元的 `u` / `y`（本轮领航-跟随）

> **逐模块展开中**：下表先列出本轮领航-跟随用到的单元与所属族；每个单元的 `u`（`step` 入参）/ `y`（`step` 返回）在评审到对应模块时逐一定稿。中间类型（`Target` / `Deviation` 等）随之确定。

| 单元 | 库 | 抽象基类（族） | `u`（`step` 入参） | `y`（`step` 返回） | 状态 |
| --- | --- | --- | --- | --- | --- |
| 收发处理(收) | 流程 | — | `inbox: list[MessageEnvelope]` | `ParsedInbox` | ⏳ 待展开 |
| 任务编排 | 流程 | — | `mission_command`（长机）/ `ParsedInbox.task`（僚机） | `Mode` | ⏳ 待展开 |
| 轨迹规划 | 流程 | `TrajectoryPlan` | `(Mode, ParsedInbox \| self_state)` | `Plan` | ⏳ 待展开 |
| 位置解算 | 算法 | `PositionSolve` | `(Plan, leader_nav?)` | `Target` | ⏳ 待展开 |
| 误差解算 | 算法 | `DeviationCalc` | `(self_state, Target)` | `Deviation` | ⏳ 待展开 |
| 跟踪 | 算法 | `Tracking` | `(Deviation, self_state)` | `AccelerationCommand` | ⏳ 待展开 |
| 控制算法 | 算法 | `ControlLaw` | 控制误差 | 控制量（被跟踪组合） | ⏳ 待展开 |
| 队形规划(广播) | 流程 | — | `(self_state, Mode, 队形, 槽位分配)` | `MessageEnvelope` | ⏳ 待展开 |

**待定中间类型**（随单元定稿）：`Plan`、`Target`、`Deviation`、`ParsedInbox`、`Mode`、`AlgorithmStatus`、`MissionCommand`。

---

## 2.5 TODO 总表（跨文档）

| 项 | 归属 | 触发 |
| --- | --- | --- |
| 各单元 `u/y` 定稿 + 中间类型 | 本册 §2.4 / 算法库 / 流程库 | 逐模块评审 |
| 编排 / 执行抽离 | 实体组 / 流程库 | 出现模态决策 / 异构僚机时 |
| 黑板 / 动态数据上下文 | 实体组 | 与"编排抽离"连体，重连出现再评估 |
| 算法库各单元数学（方程 / 参数 / 限幅） | `3-算法库LLD` | 实现阶段逐个补 |
| 流程库各单元实现 | `4-流程库LLD` | 实现阶段逐个补 |
| 扩展性压测（u/y、抽象基类按第二方案校验） | 全模块 | 加第二个方案时 |

**已定（非 TODO）**：统一实体契约（§1.1）、数据契约（§1.2）、单元通用模板 `init/reset/step(u)->y/read_state`（§2.1）、策略基类机制（§2.2）、control = `AccelerationCommand`（ENU）、算法 / 流程分界＝是否碰 `Mode`、轨迹生成拆 `轨迹规划`(流程)+`位置解算`(算法)、跟踪拆 `跟踪`(组合)+`控制算法`(PID/L1/ADRC 原子)、静态数据走构造函数入参。
