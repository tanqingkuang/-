# UI 接入仿真控制检视

检视对象：未提交改动（`src/ui/gui/main_window.py`、`tests/llt/test_gui_view_interactions.py`）
参考设计：`docs/0-架构HLD.md`、`docs/1-仿真控制HLD.md`、`docs/7-UIHLD.md`
检视结论：方向正确，可合入；建议合入前修复主要问题 1、2。

## 概览

本组改动完成 `docs/1-仿真控制HLD.md` 第 12 节实施步骤 4：把 PySide6 UI 的 `MockSimulation` 替换为 `SimulationController` 适配器。

- 新增 `ControllerSimulationAdapter`，将控制器领域契约 `SimulationSnapshot` 转换为 UI 绘制用 `Snapshot`。
- 适配器维护渐隐轨迹缓存、有限差分速度、扰动显示文案。
- 修正运行状态按钮使能逻辑；用真实 `altitude` / `step` 替换 demo 假数据。
- 新增 `closeEvent`，窗口关闭时释放后台线程资源。

整体分层边界（UI 只面向仿真控制）守得很好。

## 符合 HLD 的地方

- **接口映射正确**：`load_config / start / pause / step / reset / set_playback_rate / inject_disturbance / close` 均通过控制器命令接口完成，UI 不直接写仿真状态（HLD §5.1、UIHLD §7）。
- **轨迹缓存在 UI 侧维护**：按仿真时间窗 `TRAIL_SECONDS` 裁剪，不随播放倍率变化（HLD §5.1、§11）。
- **`closeEvent` 调用 `sim.close()`**：停掉后台 worker 线程，符合 §5.9「窗口关闭时释放资源、不设计 stop()」。
- **`control_report` 直接取自快照**：弃用 mock 中不在 `ControlReport` 枚举内的「抗风/保链」，对齐 §4.1。
- **按钮使能按状态机收敛**：start 在非 FINISHED、step 在 READY/PAUSED、pause 在 RUNNING/PAUSED，与 §6 状态机一致。

## 主要问题

### 1. 扰动显示文案不会自动清除（功能回归）

位置：`main_window.py` `_visible_disturbance`

`self.disturbance` 在注入时被设置，但加扰模块按 `duration_s` 自动到期后（model/comm 效果被 `clear_faults` 复位），该字符串没有同步清除：

- `wind` 在快照里**没有任何可观测信号**，到期后 `_visible_disturbance` 回落到 `self.disturbance` → 顶部永远显示「风场」。
- `node_fault` / `link_loss` 到期后 health/link 复位为 normal，同样回落到 stale 的 `self.disturbance`。

原 `MockSimulation` 用 `disturbance_until` 做了到期清除，这里丢失了。

建议：注入时记录 `expire_at = snapshot.time_s + duration_s`，在 `_visible_disturbance` 里用当前快照 `time_s` 比较；或改为消费 `get_recent_events()` 中的「扰动结束」事件。

附注：`if self.disturbance == "清除扰动"` 是死代码——`clear` 分支映射成的是「无」，该比较永不成立。

### 2. 配置加载失败被静默吞掉（UX 正确性）

位置：`main_window.py` `_choose_config`

适配器把 `result.message` 存进 `_load_result_message` 但从不外露；`_choose_config` 无论 `result.code` 是否为 `OK` 都执行 `config_name.setText(...)` 并 `_log("Config", f"加载配置文件 ...")`。选了非法/不存在的配置时，用户看到「加载成功」假象，违背 §5.3「加载失败状态保持原值」应有的反馈。

建议：在 adapter 返回码或把消息透传给 UI，失败时记 WARN 日志并提示用户。

### 3. 节点健康/故障显示仍是硬编码，未用真实数据

位置：`main_window.py` 侧视图染色处与节点表填充处

控制器快照每个节点都带 `health`，但适配器转换成 UI `NodeState` 时**丢弃了 health**。结果侧视图染色和节点表「状态」列仍靠 `snapshot.disturbance == "节点故障" and node.node_id == "A02"` 这种硬编码判断；`_disturbance_command` 也把故障目标写死成 `A02`、丢包写死 `A01-A02` 来对齐显示。自洽但脆弱——换个目标节点或多节点故障，UI 仍只高亮 A02。

建议：给 UI `NodeState` 加 `health` 字段，让状态列和染色数据驱动，顺带修掉问题 1 对 node_fault 的依赖。

## 次要问题

- **`pause()` 语义重载**（`main_window.py` adapter `pause`）：PAUSED 时把 `pause()` 转成 `start()`（恢复），偏离 HLD「pause 严格 RUNNING→PAUSED、继续走 start()」。功能上沿袭 mock 且 pause 按钮在合法态才可用，可接受，建议加注释说明这是 UI 便利行为。
- **`advance()` 命名误导**：它并不推进，只是 `return self.snapshot()`（真正推进在 worker 线程）。建议改名为 `poll()` 或加 docstring 说明。
- **测试存在时序敏感**（`test_gui_view_interactions.py` `test_start_pause_drives_real_controller_snapshot`）：`time.sleep(0.04)` 后断言 `time_s > 0.0` 与 `run_state == "RUNNING"`，依赖 worker 线程在 40ms 内完成调度。本机稳定，CI 负载高时可能偶发失败。建议改为轮询等待（循环检查 `time_s > 0`，超时 1s）消除 flakiness。
- **默认配置每次构造都写临时文件**（`_write_default_config`）：`tempfile.gettempdir()` 下固定文件名，多实例/测试并发时会相互覆盖（内容相同所以无害），属隐藏副作用。可考虑放到 `tests/fixtures` 或仓库内 demo 配置，让 UI 与场景标签「三机楔形队形」一致。
- **`step_label` 初值**仍是「步长：0.1s」，真实步长 0.005s，首帧刷新后才纠正。无功能影响。

## 风险/正确性确认

- **线程安全** ✓：worker 改的是控制器内部状态，`get_snapshot()` 在锁内返回 frozen 快照；`_convert_snapshot` 只在 UI 线程调用，轨迹缓存无竞争。
- **有限差分速度** ✓：`dt` 用的是 sim-time 差，得到真实 m/s，而非 wall-clock 拍频伪量。
- **测试** ✓：`tests/llt/test_gui_view_interactions.py` 8/8 通过。

## 处置建议

| 优先级 | 问题 | 建议 |
| --- | --- | --- |
| 合入前 | 1. 扰动文案不清除 | 记录到期时间或消费扰动结束事件 |
| 合入前 | 2. 加载失败静默 | 透传返回码，失败时提示并记 WARN |
| 后续 | 3. health 硬编码 | 把 health 纳入 UI 数据流，染色/状态列数据驱动 |
| 后续 | 次要项 | 命名、注释、测试去抖、配置外置 |

## 处理结果

| 问题 | 处理状态 | 回复 |
| --- | --- | --- |
| 1. 扰动显示文案不清除 | 已修改 | `ControllerSimulationAdapter` 会消费 `get_recent_events()` 中的扰动注入 / 结束 / 清除事件，同步 UI 扰动文案；扰动结束后回到“无”。 |
| 2. 配置加载失败静默 | 已修改 | 新增 `_apply_config_path()`，只有 `load_config()` 返回 `OK` 才更新配置文件名并记录加载日志；失败时保留原配置名并写入 `WARN` 日志。 |
| 3. 节点 health 硬编码 | 已修改 | UI `NodeState` 增加 `health` 字段，俯视图 / 侧视图告警颜色、节点表状态列均改为由节点 health 驱动，不再写死 A02。 |
| pause 语义重载 | 不修改 | 保留为 UI 便利行为：PAUSED 状态下暂停按钮承担继续动作，内部仍调用控制器 `start()`；代码中已补充注释说明。 |
| `advance()` 命名误导 | 已修改 | 新增 `poll()` 表达“只读取最近快照不推进”，`_on_tick()` 改为调用 `poll()`；保留 `advance()` 兼容旧调用。 |
| 测试时序敏感 | 已修改 | `test_start_pause_drives_real_controller_snapshot` 改为轮询等待 controller 时间推进，避免固定 `sleep(0.04)` 的 CI 抖动。 |
| 默认配置临时文件 | 不修改 | 当前默认配置内容固定，且只作为 UI demo 启动配置；本轮不引入仓库 fixture，后续可在正式配置 schema 明确后移到 `configs/`。 |
| `step_label` 初值 | 已修改 | UI 初始化后的首帧快照会立即刷新为真实 `step_s`，并在截图 / offscreen 构造测试中覆盖。 |

对应测试：

- `test_disturbance_label_clears_after_duration`
- `test_config_load_failure_is_reported_without_replacing_label`
- `test_node_health_drives_table_status_and_warning_color_target`
- `test_start_pause_drives_real_controller_snapshot`
- 既有 `test_main_window_uses_simulation_controller_adapter`

## 二次复查（针对上述修改）

复查结论：三个主要问题已实质修复，方向正确；复查中发现一处新引入的口径不一致，已在本轮一并修复。测试 11/11 通过。

### 修改正确性

| 项 | 结论 | 说明 |
| --- | --- | --- |
| 问题 2 加载失败 | ✅ 修得干净 | `_apply_config_path` 仅在 `last_result_code == "OK"` 时更新文件名并记成功日志，否则记 WARN 且保留原配置名；失败时控制器状态不变，旧快照仍有效。 |
| 问题 3 health 驱动 | ✅ 基本到位 | 节点表状态列、两视图告警色均由 `node.health` 驱动，匹配 HLD §4.2 取值。 |
| 问题 1 扰动文案 | ✅ 逻辑正确 | 事件驱动方案对 wind（快照无可观测信号）也能靠「扰动结束」事件清除，确实修好了原回归；`step(3)` 用例确定性验证。 |
| sim_control 注入后刷新快照 | ✅ 安全且是改进 | 锁内重建快照、无竞争，使 READY/PAUSED 态注入也能即时反映 health/link，符合 §11 不可变快照原则。 |
| 测试去抖 / `poll()` 改名 | ✅ 合理 | 轮询等待替代固定 `sleep`，消除 CI 抖动。 |

### 新引入问题（本轮已修复）

- **auto-center 与染色 health 口径不一致**：`TopView._apply_auto_center` 原用 `health not in {"fault","lost"}`，而告警染色用 `health != "normal"`。又因 UI「节点故障」按钮注入的是 `mode="degraded"`，degraded 节点会被染告警色却仍参与自动居中，两处行为对不上。**已统一为 `health == "normal"`**（异常节点一律排除出居中集合），与染色/状态列口径一致。测试 11/11 通过。

### 遗留风险（非阻塞，建议留 TODO）

- **事件游标 + 环形缓冲不匹配（潜在正确性 bug）**：`_sync_disturbance_from_events` 用 `events[self._processed_event_count:]` 计数切片，而 `controller._events` 为 `deque(maxlen=1000)`。累计事件超过 1000 后 deque 丢头、`len(events)` 封顶，绝对下标漂移会导致新扰动事件整段漏处理、扰动文案不再更新。demo 事件稀疏短期不触发，但设计脆弱。建议控制器为事件加单调序号，或游标按事件标识/时间比较。
- **跨模块字符串耦合**：扰动同步依赖 `_DisturbanceEngine` 的中文 message 措辞（`注入扰动: wind`、`扰动结束: …`、`清除扰动`），对端改文案会静默失效。建议事件携带结构化 `type` 字段。
- **死代码**：适配器 `advance()` 已无调用方（`_on_tick` 改用 `poll()`）；`MockSimulation` 整类已无人引用。可后续清理或标注保留原因。

### 「不修改」决定评估

| 项 | 决定 | 评估 |
| --- | --- | --- |
| pause 语义重载 | 不改，加注释 | 合理。注释已补，pause 按钮仅在合法态可用。 |
| 默认配置临时文件 | 不改，待 schema 定稿 | 合理。同名覆盖因内容相同而无害，defer 没问题。 |
