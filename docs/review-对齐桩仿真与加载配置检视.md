# 代码检视：`799a99f` fix: align stub simulation with loaded config

> 检视对象：最后一个 commit `799a99f`
> 参考设计文档：`0-架构HLD.md`、`1-仿真控制HLD.md`、`7-UIHLD.md`
> 检视结论：**可合入**，建议合入前澄清问题 1。

## 1. 概览

此 commit 让桩仿真严格服从加载的配置，移除了三处隐式默认值，并使 GUI 在未加载配置时正确停留在 `UNLOADED` 状态。改动跨 5 个文件，逻辑与 `1-仿真控制HLD.md`、`7-UIHLD.md` 的契约高度一致。

| 文件 | 改动 |
| --- | --- |
| `configs/base.json` | 新增示例配置（节点、链路、时长、步长、seed） |
| `src/runner/sim_control.py` | 模型/通信/节点算法 stub 不再注入默认机与默认链路；运动学按航迹角投影 |
| `src/ui/gui/main_window.py` | 移除自动写入并加载默认配置；`UNLOADED` 态禁用按钮；命令结果码守卫与日志 |
| `tests/llt/test_sim_control.py` | 新增航迹角稳定、空配置不造机两项测试 |
| `tests/llt/test_gui_view_interactions.py` | 新增 `UNLOADED` 态校验、参考航线不绘制；其余用例补加载配置 |

测试结果：`tests/llt/test_sim_control.py` 与 `tests/llt/test_gui_view_interactions.py` 共 28 项全部通过。

## 2. 与设计文档的符合性（良好）

- **`UNLOADED` 初始态**：移除 `ControllerSimulationAdapter.__init__` 中的 `_write_default_config()` + 自动 `load_config()`，符合 HLD 5.2「构造函数不读取配置」。GUI 现以 `UNLOADED` 启动并禁用相关按钮，符合状态机契约（4.1 / 6）。
- **空配置不再凭空造机/造链路**：`_ModelEngine.init` / `_CommunicationEngine.init` 改为 `config.get("nodes", [])`（含 `None` 兜底），符合「配置属于输入」原则，并由新增 `test_empty_config_does_not_create_default_aircraft_or_links` 覆盖。
- **按钮使能逻辑**：`reset` / 扰动按钮在 `UNLOADED` 禁用、扰动按钮在 `FINISHED` 禁用，正确对应 HLD 5.8 / 5.11 的 `ERR_NO_CONFIG` / `ERR_INVALID_STATE`。
- **`_start` 守卫**：仅在 `last_result_code == "OK"` 时启动定时器，修复了此前命令失败仍启动 UI tick 的隐患。
- **运动学修正**：航迹角 `psi_v_deg` 改为状态量、位移按 `cos/sin(heading)` 投影，比旧的「由 lateral_rate 反算航迹角」更自洽；节点算法 stub 不再强行改写 lateral/climb。

## 3. 问题与建议

### 问题 1（中）：`configs/base.json` 是孤立文件，未被任何代码/文档引用

```
grep -rn "base.json|configs/" src/ tests/ docs/  → 无命中
```

它内容上替代了被删除的 `_write_default_config`，但既不是默认加载项、也无文档指引用户去加载它。

**建议**：要么在 UI/CLI 文档或 README 中注明「示例配置」，要么确认 `8-CLIHLD.md` 的 `--config` 默认值是否应指向它，避免成为悬空资产。

### 问题 2（低）：`cross_track_error_m` 初值在首步后被覆盖

`base.json` 给 A02/A03 设了 `cross_track_error_m: 46/-46`，但 `_ModelEngine.step` 仍执行 `state.cross_track_error_m = state.y_m`（=318/202），首步后初值即丢失，READY 快照与运行快照不一致。该行为在本次 commit 之前已存在，但本次让配置「成为真值源」后更显眼。

**建议**：后续把侧偏定义为相对参考线的偏差，而非直接取 `y_m`。

### 问题 3（低）：航线/参考线绘制条件耦合到 `snapshot.nodes`

`if self.snapshot.nodes: self._draw_route(...)`。航线是静态参考元素，用「有无节点」作为「是否已加载配置」的代理虽能通过测试，但语义略错位——若出现有链路无节点的配置则航线不绘。可接受，但更准确的判据是 `run_state != "UNLOADED"`。

### 问题 4（低）：测试辅助 `_count_pixels` 全像素双层遍历

逐像素扫描为 O(w·h)；当前画布小无碍，若视图尺寸增大会变慢。可改为对 `QImage` 做颜色直方图或采样。

## 4. 风险

无功能性回归风险。`_step` / `_pause` 未像 `_start` 那样按结果码守卫定时器，但对应按钮在非法态已禁用，UI 路径不可达，属可接受的不对称。

## 5. 结论

**可合入。** 建议合入前澄清 `configs/base.json` 的归属/用途（问题 1），其余为后续可跟进的小项。

## 6. 处理结果

| 编号 | 处理状态 | 回复 |
| --- | --- | --- |
| 1 | 已修改 | `README.md` 的“运行 GUI”章节已说明启动后默认 `UNLOADED`，并指引用户通过左侧“选择文件”加载 `configs/base.json` 示例配置；同时说明该配置包含三机楔形 stub 场景、三条链路、仿真时长和步长。 |
| 2 | 暂不修改 | `cross_track_error_m` 后续应按“侧偏相对哪条参考线”统一定义；当前直接改为不覆盖容易和正式路径/队形参考定义冲突，先作为模型/配置 schema 细化项保留。 |
| 3 | 暂不修改 | 未加载时不画参考线是当前 UI 语义；加载后但无节点的特殊配置不属于当前可用示例路径，后续若配置 schema 支持独立航线/参考线，再改为显式 `route` 或 `run_state` 判据。 |
| 4 | 暂不修改 | `_count_pixels` 只在 offscreen 回归测试里使用，当前尺寸下耗时可接受；若后续截图测试增多，再抽成采样或直方图工具。 |
