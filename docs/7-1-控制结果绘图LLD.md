# 控制结果显示 LLD

## 1. 定位与架构

本阶段只实现 `LiveMonitorWindow`（`QDialog`），用于实时监控控制误差数据。

| 组件 | 定位 | 数据来源 |
| --- | --- | --- |
| `LiveMonitorWindow` | 运行中监控三轴控制误差是否收敛、是否发散 | `SimulationController.get_snapshot()` 内存快照 |

实时窗口持有 `SimulationController` 引用，用 `QTimer`（100 ms）轮询 `get_snapshot()`。绘图时间统一取 `snapshot.time_s`，不走 JSONL，不解析文件。

本阶段不实现离线绘图分析。

主窗口入口：新增标准菜单栏 **控制监控(&V)**，在其中添加"数据监控(&M)"菜单项。

**图表库**：PySide6.QtCharts 作为正式 GUI 硬依赖，不做降级保护。

---

## 2. 坐标系约定

所有通道数据统一在**苏联式航迹坐标系**（`enu_to_track` 输出）中定义：

| 轴 | 方向 | 基向量 |
| --- | --- | --- |
| x（前向） | 沿本机速度向量方向 | `(cos_θ·cos_ψ, cos_θ·sin_ψ, sin_θ)` |
| y（法向/上向） | 垂直速度向量、指向上 | `(-sin_θ·cos_ψ, -sin_θ·sin_ψ, cos_θ)` |
| z（侧向右） | 水平向右，垂直航迹 | `(sin_ψ, -cos_ψ, 0)` |

θ 为俯仰角，ψ 为水平航迹角（ENU，0° 朝东，90° 朝北）。近水平飞行时 y ≈ ENU 天向，z ≈ 水平右向。

坐标系以**本机实际速度**为基向量计算，每帧更新。

---

## 3. 显示通道定义

所有通道均显示**误差量**（无指令线），以 0 为视觉基准。误差符号约定：

- **位置误差** = cmd − actual（正值 = 实际落后于指令）
- **速度误差 x** = `cmd.vd − actual.vd`（标量地速之差）
- **速度误差 y/z** = 指令速度在实际航迹法向 / 侧向的投影分量（实际分量恒为 0，因航迹系以实际速度为基）
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

- `perr_x`（前向位置误差）：僚机有意义（槽位前向偏差），长机前向不做位置闭环，该值接近 0。
- `verr_y`/`verr_z`：实际速度在航迹 y/z 轴分量恒为 0（由坐标系定义决定），因此这两个误差等于指令速度在对应轴的投影；其控制意义是 PID 的速度前馈参考量，而非传统 cmd−actual 之差。
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

## 8. 文件变更清单

| 文件 | 操作 | 说明 |
| --- | --- | --- |
| `src/ui/gui/live_monitor.py` | 新建 | `LiveMonitorWindow` 全部实现 |
| `src/ui/gui/main_window.py` | 修改 | 菜单栏、`_open_live_monitor()`、`_start()`、`_reset()`、`closeEvent()`、`_apply_config_path()` |
| `requirements-gui.txt` | 修改 | 显式追加 `PySide6-Addons==6.11.1`（QtCharts 所在包） |
| `src/runner/sim_control.py` | 不修改 | 现有 `NodeState` 字段满足 V1 需求 |
