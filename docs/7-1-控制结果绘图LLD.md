# 控制结果显示 LLD

## 1. 定位与架构

本文档涵盖两个绘图组件：

| 组件 | 定位 | 数据来源 |
| --- | --- | --- |
| `LiveMonitorWindow` | 运行中实时监控三轴控制误差是否收敛、是否发散 | `SimulationController.get_snapshot()` 内存快照 |
| `OfflinePlotWindow` | 运行结束后加载日志文件、回放并分析控制误差时序曲线 | `logs/<run-id>/snapshots.jsonl` 落盘文件 |

实时窗口持有 `SimulationController` 引用，用 `QTimer`（100 ms）轮询 `get_snapshot()`。绘图时间统一取 `snapshot.time_s`，不走 JSONL，不解析文件。

离线窗口不持有控制器引用，不使用定时器；用户通过文件对话框选择 `snapshots.jsonl`，一次性加载全部帧，静态渲染完整时序曲线。

主窗口入口：标准菜单栏 **控制监控(&V)**，包含两个菜单项：

- **数据监控(&M)**：打开 `LiveMonitorWindow`
- **离线分析(&A)**：打开 `OfflinePlotWindow`

**图表库**：PySide6.QtCharts 作为正式 GUI 硬依赖，不做降级保护。

---

## 2. 坐标系约定

所有 `track_*` 通道统一使用 [`docs/0-坐标系约定.md`](0-坐标系约定.md) 定义的**制导地速航迹 FUR**（`enu_to_track` 输出）：

| 轴 | 方向 | 基向量 |
| --- | --- | --- |
| x（前向） | 沿本机速度向量方向 | `(cos_θ·cos_ψ, cos_θ·sin_ψ, sin_θ)` |
| y（法向/上向） | 垂直速度向量、指向上 | `(-sin_θ·cos_ψ, -sin_θ·sin_ψ, cos_θ)` |
| z（侧向右） | 水平向右，垂直航迹 | `(sin_ψ, -cos_ψ, 0)` |

θ 为航迹倾角，ψ 为水平航迹角（ENU，0° 朝东，90° 朝北，左转/逆时针为正）。近水平飞行时 y ≈ ENU 天向，z ≈ 水平右向。

控制误差默认以**目标/指令地速**为基向量计算，每帧更新；目标水平地速不足以定向时退回本机实际地速。该制导地速航迹系不得与动力学空速航迹系混用。

---

## 3. 显示通道定义

所有通道均显示**误差量**（无指令线），以 0 为视觉基准。误差符号约定：

- **位置误差** = cmd − actual，再投影到目标地速 FUR（前向正值表示实际落后于指令）
- **速度误差 x** = `cmd.vd − actual.vd`（标量地速之差）
- **速度误差 y/z** = 指令地速分量 − 实际地速分量，二者均投影到同一目标地速 FUR
- **航迹角偏差** = `actual_ψ − cmd_ψ`，归一化到 `(-180°, 180°]`

### 3.1 通道列表

| key | 通道名 | 单位 | 轴 | 字段/计算 | 默认 |
| --- | --- | --- | --- | --- | --- |
| `perr_x` | 前向位置误差 | m | 前向 x | `node.track_pos_err_x_m` | **开** |
| `verr_x` | 前向速度误差 | m/s | 前向 x | `node.track_vel_err_x_mps` | 关 |
| `perr_y` | 垂向位置误差 | m | 垂向 y | `node.track_pos_err_y_m` | **开** |
| `verr_y` | 垂向速度误差 | m/s | 垂向 y | `node.track_vel_err_y_mps` | 关 |
| `perr_z` | 侧向位置误差 | m | 侧向 z | `node.track_pos_err_z_m` | **开** |
| `verr_z` | 侧向速度误差 | m/s | 侧向 z | `node.track_vel_err_z_mps` | 关 |
| `hdg_dev` | 航迹角偏差 | ° | 侧向 z | `actual_ψ − cmd_ψ`（见 3.2） | 关 |

默认开启 3 个子图：前向、垂向、侧向位置误差。

### 3.2 航迹角偏差计算

```python
def _hdg_dev(n: NodeState) -> float | None:
    spd = math.hypot(n.cmd_vel_east_mps, n.cmd_vel_north_mps)
    if spd < 1e-3:
        return None   # 指令速度过小，航向不可定义，跳过本帧
    cmd_psi = math.degrees(math.atan2(n.cmd_vel_north_mps, n.cmd_vel_east_mps))
    return (n.psi_v_deg - cmd_psi + 180.0) % 360.0 - 180.0
```

结果在 `(-180°, 180°]`，无需 unwrap，每帧独立计算。

### 3.3 字段说明补充

- `perr_x`（前向位置误差）：僚机有意义（槽位前向偏差）；长机前向不做位置闭环时，该诊断通道输出 0。
- `verr_y`/`verr_z`：目标速度在自身地速 FUR 的 y/z 分量通常为 0，因此两者通常等于实际地速相应投影的相反数；语义仍统一为 `cmd−actual`。
- 航迹角偏差与 `verr_z` 存在强相关（`verr_z ≈ V·sin(Δψ)`），二者信息部分重叠。

---

## 4. 当前控制策略色条

固定显示在图表区顶部，不计入通道 checkbox，始终可见。

| 策略值 | 颜色 |
| --- | --- |
| `"待命"` | `#888888`（灰） |
| `"集结"` | `#2ecc71`（绿） |
| `"保持"` | `#4c8ef5`（蓝） |
| `"重构"` | `#c0392b`（红） |

策略值与 `ControlReport = Literal["待命", "集结", "保持", "重构"]` 完全一致。

---

## 5. 接口定义

### 5.1 `LiveMonitorWindow` 公开接口

```python
class LiveMonitorWindow(QDialog):

    def follow(self, ctrl: SimulationController) -> None:
        """仿真启动后由 MainWindow 调用。
        若 ctrl 与当前相同，只确保定时器运行；
        若是新 ctrl，清空旧缓冲区和节点列表，重新绑定并启动定时器。
        """

    def unfollow(self) -> None:
        """停止定时器，清空缓冲区和节点列表。仿真重置时由 MainWindow 调用。"""
```

### 5.2 内部轮询 `_poll()`

1. `snap = ctrl.get_snapshot()`，取 `t = snap.time_s`。
2. 更新策略色条（无论时间是否推进均刷新）。
3. 发现新节点 → 分配颜色 → 标记 `_rebuild_needed`。
4. 若 `_rebuild_needed`：刷新节点面板，重建图表，重填历史数据。
5. **去重**：若 `t <= _last_t`，跳过追加，直接返回。
6. `_last_t = t`，遍历节点追加各通道缓冲区。
7. 更新所有 series（`replace()`，仅保留窗口内点）、X 轴范围、Y 轴百分位范围、零基准线端点、当前值标签。

### 5.3 `MainWindow` 集成

```python
# __init__
self._live_monitor: LiveMonitorWindow | None = None

# _build_ui — 菜单
monitor_menu = self.menuBar().addMenu("控制监控(&V)")
monitor_menu.addAction("数据监控(&M)").triggered.connect(self._open_live_monitor)

# _open_live_monitor
def _open_live_monitor(self) -> None:
    from src.ui.gui.live_monitor import LiveMonitorWindow
    if self._live_monitor is None:
        self._live_monitor = LiveMonitorWindow(self)
    if self.sim.snapshot().run_state not in ("UNLOADED", "READY"):
        self._live_monitor.follow(self.sim.controller)
    self._live_monitor.show()
    self._live_monitor.raise_()

# _start — OK 分支
if self._live_monitor is not None:
    self._live_monitor.follow(self.sim.controller)

# _reset — 末尾
if self._live_monitor is not None:
    self._live_monitor.unfollow()

# _apply_config_path — OK 分支末尾（切配置时清空旧曲线，防止 controller 同对象导致 follow() 不刷新）
if self._live_monitor is not None:
    self._live_monitor.unfollow()

# closeEvent
if self._live_monitor is not None:
    self._live_monitor.close()
```

不修改：`sim_control.py`、`NodeState`、`SimulationSnapshot`。

---

## 6. 实现细节

### 6.1 通道数据结构

```python
@dataclass
class Ch:
    key:   str                                    # 缓冲区/series 键名
    label: str                                    # 侧边栏显示名
    unit:  str                                    # 单位（空串表示无单位）
    group: str                                    # 侧边栏分组标题
    on:    bool                                   # checkbox 默认状态
    act:   Callable[[NodeState], float | None]    # 数据提取器；None → 跳过本帧
```

### 6.2 缓冲区与 series 键

```python
# 缓冲区：ch.key -> node_id -> deque[(time_s, value)]
_bufs: dict[str, dict[str, deque[tuple[float, float]]]]

# series：(node_id, ch.key) -> QLineSeries
_series: dict[tuple[str, str], QLineSeries]

_MAX_PTS = int(120 * 10 * 1.2)  # 120 s × 10 Hz + 20% 余量
```

每个通道每个节点独立 deque，互不干扰。

### 6.3 Y 轴自适应（百分位截断）

使用窗口内所有可见节点数据的 **5%～95% 百分位**定轴，避免单个节点的离群大值撑开整个轴。Y 轴范围始终包含 0。

```python
def _apply_y_range(y_ax: QValueAxis, all_y: list[float]) -> None:
    if not all_y:
        y_ax.setRange(-1.0, 1.0)
        return
    lo = min(_percentile(all_y, 5),  0.0)
    hi = max(_percentile(all_y, 95), 0.0)
    margin = max((hi - lo) * 0.15, 0.5)
    y_ax.setRange(lo - margin, hi + margin)
```

离群点（超过 95 百分位）曲线会延伸至轴范围之外，仍可被观察到趋势。

### 6.4 图表布局

```text
LiveMonitorWindow（QDialog，默认 1200 × 760）
└── QVBoxLayout
    └── QHBoxLayout
        ├── 左侧边栏（固定宽 170）
        │   ├── QGroupBox "节点"（动态填充，默认全选）
        │   ├── QGroupBox "通道"（按轴分组，见 3.1）
        │   └── QGroupBox "时间窗口"（QComboBox：30s / 60s / 120s）
        └── 右侧图表区
            ├── 策略色条（QLabel，高 26）
            └── QScrollArea
                └── 每通道一行：QChartView + 当前值 QLabel 列
```

子图行高：

- 勾选通道 ≤ 4 个：各行 stretch=1，均分可用高度（最小 100 px）
- 勾选通道 ≥ 5 个：各行固定 155 px，超出时滚动条出现

最后一个可见子图显示 X 轴刻度标签，其余子图隐藏 X 轴刻度。

### 6.5 节点着色

```python
_PALETTE = ["#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]
```

每架机按发现顺序从调色板取色，实际值曲线为实线宽 2.0。

### 6.6 图表重建时机

- `follow()` 或 `unfollow()` 被调用
- 首次读到新节点 ID
- 节点 checkbox 或通道 checkbox 状态变化

重建后调用 `_repopulate()` 将缓冲区历史数据填入新 series，并立即刷新 Y 轴范围。

---

## 7. 依赖说明

```python
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore   import QMargins, QPointF, Qt, QTimer
from PySide6.QtGui    import QColor, QFont, QPainter, QPen
```

QtCharts 为硬依赖，代码中不做降级保护。

---

## 8. 文件变更清单（LiveMonitorWindow）

| 文件 | 操作 | 说明 |
| --- | --- | --- |
| `src/ui/gui/live_monitor.py` | 新建 | `LiveMonitorWindow` 全部实现 |
| `src/ui/gui/main_window.py` | 修改 | 菜单栏、`_open_live_monitor()`、`_start()`、`_reset()`、`closeEvent()`、`_apply_config_path()` |
| `requirements-gui.txt` | 修改 | 显式追加 `PySide6-Addons==6.11.1`（QtCharts 所在包） |
| `src/runner/sim_control.py` | 不修改 | 现有 `NodeState` 字段满足 V1 需求 |

---

## 9. OfflinePlotWindow 设计

### 9.1 定位

`OfflinePlotWindow`（`QDialog`）是纯离线可视化组件，不依赖仿真控制器，不使用定时器。用户选择 `snapshots.jsonl` 文件后，窗口一次性加载所有帧，静态渲染完整时序曲线，便于事后分析控制误差收敛情况。

### 9.2 数据源：snapshots.jsonl

`_DataLogger.write_snapshot()` 按 10 Hz（仿真时间）写入，每行一个 JSON 对象，字段与 `SimulationSnapshot` 一致，省略 `step_s`、`route`、`route_segments`。

典型帧格式（字段已精度截断）：

```json
{
  "time_s": 0.100,
  "duration_s": 120.0,
  "run_state": "RUNNING",
  "control_report": "保持",
  "nodes": [
    {
      "node_id": "A01", "role": "leader", "health": "normal",
      "x_m": 140.12, "y_m": 260.34, "altitude_m": 1200.00,
      "psi_v_deg": 89.73, "theta_deg": 0.00,
      "speed_mps": 8.02, "ground_speed_mps": 8.02,
      "vx_mps": 0.24, "vy_mps": 8.02, "vz_mps": 0.00,
      "nx": 0.0000, "ny": 1.0000, "nz": 0.0000, "n_normal": 1.0000,
      "phi_deg": 0.00, "psi_dot_deg_s": 0.00,
      "cmd_vel_east_mps": 0.00, "cmd_vel_north_mps": 8.00, "cmd_vel_up_mps": 0.00,
      "track_pos_err_x_m": 0.12, "track_pos_err_y_m": 0.00, "track_pos_err_z_m": -0.34,
      "track_vel_err_x_mps": 0.02, "track_vel_err_y_mps": 0.00, "track_vel_err_z_mps": 0.00,
      "cross_track_error_m": null, "distance_to_go_m": null
    }
  ],
  "links": [...],
  "cpu_utilization": 0.03
}
```

所有 `NodeState` 字段均存在；其中 `speed_mps` 是暂时保留的空速兼容字段，`ground_speed_mps/theta_deg/psi_v_deg/psi_dot_deg_s` 与 `vx_mps/vy_mps/vz_mps` 描述叠加风后的地速航迹。算法 `vd` 使用水平地速，不能与三维 `ground_speed_mps` 混用。`null` 对应 Python `None`，与 `NodeState.cross_track_error_m: float | None` 兼容。

### 9.3 通道与代码复用

`OfflinePlotWindow` 复用 `live_monitor.py` 中的纯工具定义，但通道列表独立维护：

```python
from src.ui.gui.live_monitor import Ch, _hdg_dev, _PALETTE, _apply_y_range
```

`OFFLINE_CHANNELS` 在 `offline_plot.py` 内独立定义（7 个通道，内容与 `live_monitor.CHANNELS` 相同），不从 `live_monitor` 导入 `CHANNELS`，以便后续扩展离线专属通道（如 `nx/ny/nz/n_normal`、`cross_track_error_m`）时不影响实时监控侧边栏。

### 9.4 NodeState 重建

JSONL 中每个节点 dict 直接用于重建 `NodeState`：

```python
from dataclasses import fields as dc_fields
from src.runner.sim_control import NodeState

_NODE_STATE_FIELDS = {f.name for f in dc_fields(NodeState)}

node_kw = {k: v for k, v in node_dict.items() if k in _NODE_STATE_FIELDS}
node = NodeState(**node_kw)
```

过滤未知键（JSONL 格式演进时的容错）。`NodeState` 所有无默认值字段（`node_id`、`role`、`health`、`x_m` 等 16 个）在规范 JSONL 中始终存在；`TypeError` 说明文件损坏，跳过该帧节点即可。

### 9.5 UI 布局

```text
OfflinePlotWindow（QDialog，默认 1200 × 760）
└── QVBoxLayout（root）
    ├── 顶栏（QWidget，固定高约 36）
    │   ├── QPushButton "打开日志…"（固定宽 90）
    │   └── QLabel path（伸展，显示文件名，tooltip 显示完整路径）
    ├── QFrame（水平分隔线）
    └── QHBoxLayout（body）
        ├── 左侧边栏（QWidget，固定宽 170）
        │   ├── QGroupBox "节点"（动态填充，同 LiveMonitorWindow）
        │   └── QGroupBox "通道"（按轴分组，同 LiveMonitorWindow）
        └── 右侧图表区（QWidget，stretch=1）
            └── QScrollArea
                └── 每通道一行：QChartView（rubber band zoom）
```

与 `LiveMonitorWindow` 的主要区别：

- 无策略色条（`control_report` 随时间变化，离线场景意义不大）
- 无"时间窗口"下拉框（X 轴固定为完整仿真时间范围）
- 无当前值标签（数据静态，末尾值无特殊意义）
- 顶栏替换为文件选择栏

### 9.6 图表行为

**X 轴**：加载完成后设为 `[t_min, t_max + Δ]`，其中 `Δ = 0.5 s`（留白避免最后一点被截断）。X 轴端点在数据加载后固定，不随任何操作变化（用户若缩放则由 QtCharts 内部管理）。

**Y 轴**：同 `_apply_y_range()`，5%～95% 百分位，始终包含 0。

**y=0 基准线**：灰色虚线，端点固定为 `[t_min, t_max + Δ]`，不需要动态更新。

**缩放交互**：每个 `QChartView` 启用 `RectangleRubberBand`——用户左键拖拽矩形框选区域放大；右键点击图表区重置该子图缩放（Qt 内置行为）。

**数据量**：10 Hz × 120 s = 1200 帧/节点，3 节点 × 7 通道 × 1200 点 = 25200 个 `QPointF`，一次性 `series.replace()` 填入，耗时可忽略。

### 9.7 数据结构

```python
# 节点表：node_id -> {color: str, visible: bool, cb: QCheckBox | None}
_nodes: dict[str, dict]

# 缓冲区：ch.key -> node_id -> list[(time_s, value)]
# 使用 list 而非 deque，加载后不再追加
_bufs: dict[str, dict[str, list[tuple[float, float]]]]

# 坐标轴范围
_t_min: float  # 所有帧中最小 time_s
_t_max: float  # 所有帧中最大 time_s

# series：(node_id, ch.key) -> QLineSeries
_series: dict[tuple[str, str], QLineSeries]

# 行缓存：ch.key -> (QChart, x_ax, y_ax, zero_series)
_rows: dict[str, tuple]
```

### 9.8 方法职责

| 方法 | 职责 |
| --- | --- |
| `_build_ui()` | 顶栏 + 侧边栏 + 图表区骨架，末尾调用 `_rebuild_charts()` |
| `_build_sidebar()` | 节点 GroupBox（动态）+ 通道 GroupBox（同 LiveMonitorWindow） |
| `_refresh_node_panel()` | 清空并重建节点 checkbox，同 LiveMonitorWindow |
| `_toggle_node(nid, visible)` | 切换节点可见性，调用 `_rebuild_charts()` |
| `_choose_file()` | `QFileDialog.getOpenFileName`，过滤 `*.jsonl`，调用 `_load_file()` |
| `_load_file(path)` | 解析 JSONL → 填充 `_nodes` + `_bufs` → 调用 `_refresh_node_panel()` + `_rebuild_charts()` |
| `_rebuild_charts()` | 按勾选通道和节点重建 `_series` 和 `_rows`，末尾调用 `_populate_series()` |
| `_make_chart(ch, node_ids, show_x)` | 创建单通道 `QChart`，返回 `(chart, x_ax, y_ax, zero_s)` |
| `_populate_series()` | 将 `_bufs` 数据填入 `_series`，刷新 X/Y 轴范围和零基准线端点 |

`_load_file()` 出错时在 path label 显示简短错误，不弹对话框，不打断 UI。

### 9.9 MainWindow 集成

```python
# __init__
self._offline_plot: "OfflinePlotWindow | None" = None

# _build_ui — 菜单（追加到 monitor_menu）
monitor_menu.addAction("离线分析(&A)").triggered.connect(self._open_offline_plot)

# _open_offline_plot（懒加载，与 _open_live_monitor 模式一致）
def _open_offline_plot(self) -> None:
    from src.ui.gui.offline_plot import OfflinePlotWindow
    if self._offline_plot is None:
        self._offline_plot = OfflinePlotWindow(self)
    self._offline_plot.show()
    self._offline_plot.raise_()

# closeEvent
if self._offline_plot is not None:
    self._offline_plot.close()
```

`OfflinePlotWindow` 不关联仿真生命周期（无 `follow`/`unfollow`），因此 `_start()`、`_reset()`、`_apply_config_path()` 均不需要修改。

### 9.10 注释覆盖率

`scripts/comment_coverage.py` 要求所有公开和私有函数/类均有 docstring。`offline_plot.py` 所有方法必须一一添加。

---

### 9.11 规划中的编队控制效果分析（暂不实现）

当前 V1 只实现"打开 JSONL 文件、按通道显示时序曲线"，属于基础可视化。后续需要扩展为**编队控制效果分析**，侧重点与实时监控不同：

| 维度 | 实时监控（LiveMonitorWindow） | 离线分析（OfflinePlotWindow 规划） |
| --- | --- | --- |
| 时间范围 | 滚动窗口（30/60/120 s） | 完整仿真时长 |
| 关注目标 | 当前是否发散 / 是否收敛 | 全程控制质量评估 |
| 指标形式 | 瞬时曲线 | 统计量 + 阶段分析 |
| 交互方式 | 实时自动刷新 | 静态，支持局部缩放 |

#### 9.11.1 误差收敛性分析

- **稳态误差**：仿真后段（如最后 10% 时长）各通道误差的均值和 RMS，每节点单独计算。
- **收敛时间**：误差首次进入并持续留在稳态区间（如 ±0.5 m）的时刻。
- **最大超调**：各通道误差的历史最大绝对值及发生时刻。

以上指标以数值表格形式展示，不依赖曲线图。

#### 9.11.2 队形精度分析

- **节点相对位置误差**：各节点相对长机的实际位置 vs 理论槽位，在航迹坐标系三轴分解。等同于 `track_pos_err_{x,y,z}_m`，但需区分集结阶段与保持阶段分段统计。
- **队形保持率**：保持阶段中，节点误差落在允许范围内的时间占比。
- **编队重构分析**：识别 `control_report == "重构"` 区间，统计重构持续时长和重构后误差恢复速率。

#### 9.11.3 航线跟踪质量（长机）

- **侧偏距时序**：`cross_track_error_m` 全程曲线，评估长机对规划航线的跟踪精度。
- **待飞距**：`distance_to_go_m` 曲线，反映长机是否按预期速率飞完航段。
- **航段切换时刻**：从快照检测 `route` 变化，在时序图上标记垂直分隔线。

#### 9.11.4 控制量历程

- **实际过载与滚转**：`nx/ny/nz`（空速航迹 FUR 有符号分量）、`n_normal`（法向合过载）、`phi_deg`（右倾为正）全程时序，评估控制响应平滑性和极值裕量。
- **速度指令 vs 实际速度**：`cmd_vel_{east,north,up}_mps` 与 `v{x,y,z}_mps` 对比曲线，反映指令跟踪能力。

#### 9.11.5 数据导出

- 将统计指标导出为 CSV，供外部工具（Excel、MATLAB、Python）进一步处理。
- 将当前可见曲线导出为 PNG，分辨率不低于 150 dpi。

#### 9.11.6 实现约束

- 以上功能均依赖已有 JSONL 字段，不需要修改 `NodeState` 或 `SimulationSnapshot`。
- 统计计算在 UI 线程同步完成（数据量小，不需要异步），但若加载时长超过 500 ms 应考虑进度提示。
- 导出功能依赖标准库（`csv`）和 `QChart.grab()`，不引入新的第三方依赖。

---

## 10. 文件变更清单（OfflinePlotWindow）

| 文件 | 操作 | 说明 |
| --- | --- | --- |
| `src/ui/gui/offline_plot.py` | 新建 | `OfflinePlotWindow` 全部实现 |
| `src/ui/gui/main_window.py` | 修改 | `_offline_plot` 字段、`_open_offline_plot()`、`monitor_menu` 追加菜单项、`closeEvent()` |
| `src/ui/gui/live_monitor.py` | 不修改 | `CHANNELS`、`_PALETTE`、`_apply_y_range` 由 offline_plot 直接 import |
| `src/runner/sim_control.py` | 不修改 | |
