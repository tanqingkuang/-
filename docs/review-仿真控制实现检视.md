# 代码检视：仿真控制 `SimulationController` 首版实现

> 检视对象：未提交改动
> - `src/runner/sim_control.py`（重写，+939 行）
> - `编队仿真.app/Contents/Resources/appsrc/src/runner/sim_control.py`（同步副本）
> - `tests/llt/test_sim_control.py`（新增，8 用例，全部通过）
>
> 设计依据：`docs/0-架构HLD.md`、`docs/1-仿真控制HLD.md`、`docs/7-UIHLD.md`
> 检视日期：2026-06-15

## 概览

本次改动用一个完整的 `SimulationController` 门面替换了原 `sim_control.py` 占位文件，并新增低层测试。实现严格对照 `docs/1-仿真控制HLD.md` 的应用层契约：

- ✅ 数据契约（`RunState`/`ControlReport`/`EventLevel`/`DisturbanceType`/`ResultCode`、`NodeState`/`LinkState`/`SimulationSnapshot`/`SimulationEvent`/`CommandResult`/`DisturbanceCommand`）与 HLD §4 完全一致。
- ✅ 应用层接口（§5）齐全：`load_config`/`get_snapshot`/`start`/`pause`/`step`/`reset`/`close`/`set_playback_rate`/`inject_disturbance`/`subscribe_snapshot`/`get_recent_events`/`run_until_complete`，签名与语义吻合，含 `Subscription`。
- ✅ 多速率调度（§7）：编队 10 tick、通信/快照 2 tick、落盘 10 tick、显示 10Hz wall-clock 节流，结构正确。
- ✅ 边界遵守良好：动态扰动统一经 `_DisturbanceEngine` 转发到 model/comm，控制器**不**直接调用 `inject_wind`/`inject_link_fault`（符合 §3、§7「加扰是动态扰动唯一通道」）。无 PySide6 依赖。
- ✅ 内部模块接口（§8）按 stub 形式落地，便于后续替换真实模型/通信/算法。
- ✅ 快照字段与 `docs/7-UIHLD.md` §7 的 UI 依赖清单一致。

整体质量高、可读性好、契约对齐到位。以下问题按严重度排列。

## 关键问题（建议修复）

### 1. `start()` 恢复时存在 worker 线程竞态 → 可能 RUNNING 卡死

`pause()` 只把状态置 `PAUSED` 让 worker 自然退出循环，不 join。`start()` 通过 `worker.is_alive()` 判断是否新建线程（`sim_control.py:575`）。

竞态窗口：worker 已通过 `if run_state != "RUNNING": break` 判断、正在退出但 `is_alive()` 仍为 `True`；此时 `start()` 置 `RUNNING` 后认为线程还活着、不新建 —— 旧线程随即死亡，结果状态为 `RUNNING` 但无推进线程，仿真冻结。

**建议**：用明确的「worker 应运行」标志/重新校验，或在 `start()` 中以 CAS 方式确保始终有存活 worker。

### 2. 双份源码副本会漂移 —— `编队仿真.app/.../appsrc/src/runner/sim_control.py`

本次同时改了 `.app` 包内的源码拷贝。经核对：在 HEAD 处该副本**已经**与 `src/` 不一致，本次改动只是又把两者改成一致。把构建产物（.app bundle 内的 appsrc）纳入 git 并手工同步，极易再次漂移，且 review/diff 噪声翻倍。

**建议**：将 `.app` 产物从版本控制移除并加入 `.gitignore`，由打包流程生成；若必须保留，应有脚本保证副本由 `src/` 派生而非手改。

## 设计偏差（建议确认/记录）

### 3. 提前引入后台线程，未走 HLD 的「命令事件队列」模型

HLD §10 明确：「核心逻辑按单线程状态机设计，先保证确定性……后续若引入后台线程，所有命令接口必须串行化进入仿真控制事件队列」。本实现首版即用 `threading.Thread` + `RLock`。命令确实被锁串行化，但不是 HLD 设想的事件队列，且引入了问题 #1 的竞态。这是可接受的工程取舍，但与 HLD 不符。

**建议**：在文档或 PR 描述里显式记录该偏差及理由。

### 4. `subscribe_snapshot` 未处理重复订阅

HLD §5.12 建议「同一订阅者多次订阅时返回同一订阅或替换旧订阅，避免重复刷新」。当前每次都分配新 id（`:697`），同一回调多订即多次刷新。首版可接受。

**建议**：至少加 TODO 或在 docstring 注明未实现去重。

## 正确性 / Stub 一致性细节

### 5. `_CommunicationEngine.tick` 丢弃配置的 `latency_ms`（`:342`）

每个 tick 用 `base_latency = 18.0 + index*5.0` 重算延迟，覆盖 `init` 时读入的 `latency_ms`，配置值在第一拍后即失效（`loss_rate` 则被保留）。stub 行为不一致；至少应保留配置基线。

### 6. `_DisturbanceEngine.tick` 空闲时每拍无条件 `clear_faults()`（`:448`）

无活动扰动时每个 tick 调用 `model.clear_faults()`，把**所有**节点 health 重置为 `normal` 并清风。对纯 stub 无害，但语义上「无动态扰动」不应等价于「强制所有节点健康」——若将来配置里设有静态故障节点，会被反复抹掉。

**建议**：仅在「上一拍有扰动、本拍变空」的边沿触发一次清理。

### 7. `inject_link_fault` 忽略 `params`（如 `loss_rate`）（`:357`）

退化丢包率硬编码 0.25、延迟 +40，未读取命令 `params.loss_rate`。HLD 示例命令 `{"type":"link_loss","params":{"loss_rate":0.3}}` 的强度参数不生效。stub 可接受。

**建议**：留 TODO。

### 8. `load_config` 释放锁做 IO 后未复检 `RUNNING`（`:534`）

先在锁内判 `ERR_BUSY`，释放锁读盘，再加锁 `_init_modules`，中途状态可能变化且不会重置 worker。窗口很窄、实际触发概率低，但严格起见可在重新加锁后复检状态。

## 测试覆盖

已覆盖核心 happy path 与若干状态约束（初始 UNLOADED、load→READY、手动 step 保持 PAUSED、run_until_complete→FINISHED、无配置 start 拒绝、FINISHED 需 reset、扰动事件、订阅/退订）。

**建议补充**：

- `pause()` 与 `start()` 续跑（含 worker 生命周期，可间接暴露问题 #1）。
- `reset()` 后时间归零、状态回 READY。
- 错误码路径：`ERR_CONFIG_NOT_FOUND`、`ERR_CONFIG_INVALID`、`set_playback_rate` 越界、`step(0)` → `ERR_INVALID_ARGUMENT`、`inject_disturbance` 非法 type。
- 订阅回调抛异常时不中断循环且记 `WARN`（HLD §5.12 约束）。
- `get_recent_events(min_level=...)` 过滤与 `limit` 截断。

## 小结

实现质量与契约对齐都很好，可作为首版合入的基础。合入前建议至少处理：

- **#1（线程竞态）**
- **#2（.app 双份源码）**

#3/#4 作为已知偏差记录，#5–#8 作为 stub 的 TODO 跟进。

| 编号 | 问题 | 严重度 | 建议 |
| --- | --- | --- | --- |
| 1 | `start()`/worker 线程竞态可能卡死 | 高 | 合入前修复 |
| 2 | `.app` 双份源码漂移 | 高 | 合入前处理 |
| 3 | 提前引入线程，未走事件队列 | 中 | 记录偏差 |
| 4 | 重复订阅未去重 | 中 | TODO |
| 5 | comm tick 丢弃配置延迟 | 低 | 修正 stub |
| 6 | 空闲每拍 clear_faults | 低 | 边沿触发 |
| 7 | link_fault 忽略 params | 低 | TODO |
| 8 | load_config IO 后未复检状态 | 低 | 可选加固 |

## 处理结果

| 编号 | 处理状态 | 回复 |
| --- | --- | --- |
| 1 | 已修改 | `start()` 在恢复运行前会先停止仍处于退出窗口的旧 worker，并在重新加锁后复检状态；worker 退出时也会清理自身引用，避免 `RUNNING` 无推进线程。 |
| 2 | 已修改 | `.app` 已加入 `.gitignore`，并通过 `git rm --cached` 从版本控制索引移除；磁盘上的 app bundle 保留可用，后续不再跟踪 bundle 内源码副本。 |
| 3 | 不修改 | 首版仍保留 worker + `RLock`，原因是当前 UI 需要后台推进且完整命令事件队列会扩大首版实现范围；本轮已修复 worker 生命周期竞态，后续再演进为事件队列。 |
| 4 | 已修改 | `subscribe_snapshot()` 增加同一 callback 去重，多次订阅不会形成重复刷新。 |
| 5 | 已修改 | `_CommunicationEngine` 保留配置中的 `latency_ms` / `loss_rate` 作为基线，`tick()` 不再覆盖配置延迟。 |
| 6 | 已修改 | `_DisturbanceEngine` 不再空闲每拍清理动态影响，只在扰动全部结束或收到 `clear` 时清理；模型健康状态恢复到配置基线而不是强制 `normal`。 |
| 7 | 已修改 | `link_loss` / `link_fault` 读取 `params.loss_rate` 和可选 `params.latency_delta_ms`。 |
| 8 | 已修改 | `load_config()` 完成 IO 后重新加锁复检 `closed` 和 `RUNNING` 状态，避免读配置期间状态变化后仍初始化模块。 |

补充测试：

- `pause()` 后 `start()` 续跑，验证后台 worker 能继续推进。
- `reset()` 后时间归零并回到 `READY`。
- 缺失 / 非法配置、非法倍速、非法单步、非法扰动类型等错误码路径。
- 订阅回调异常记录 `WARN`，重复订阅不重复刷新。
- `get_recent_events()` 的等级过滤和 limit 截断。
- 通信配置延迟保留、链路丢包扰动读取 `loss_rate`。
