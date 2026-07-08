"""MainWindow 运行控制、配置、全屏和外部窗口动作。注意：只放事件处理流程。"""

from __future__ import annotations

from configparser import ConfigParser
import json
import math
import os
from pathlib import Path

from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtWidgets import QFileDialog, QTableWidgetItem, QVBoxLayout, QWidget

from src.data.geo import enu_to_geodetic
from src.ui.gui.avoidance_tools import _geo_origin_from_config_for_ui, parse_avoidance_config
from src.ui.gui.dialogs import StageFullscreenDialog
from src.ui.gui.simulation_adapter import link_direction_label
from src.ui.gui.theme_widgets import THEMES
from src.ui.gui.view_models import (
    APP_CONFIG_KEY_LAST_CONFIG,
    APP_CONFIG_SECTION,
    LinkState,
    NodeState,
    playback_rate_to_slider_value,
    Snapshot,
    slider_value_to_playback_rate,
    WORLD_HEIGHT,
    WORLD_WIDTH,
    default_project_root,
    leader_node_from,
    trail_seconds_for_duration,
)


class MainWindowActionMixin:
    """拆分主窗口事件处理逻辑。注意：由 MainWindow 继承使用。"""

    def _update_snapshot(self, snapshot: Snapshot, *, fit_top_view: bool = False, fit_side_view: bool = False) -> None:
        """更新 snapshot 状态。注意：保持界面显示和内部数据一致。"""
        # 左侧状态文本与时间轴。
        self.run_state_label.setText(snapshot.run_state)
        self.report_label.setText(f"回报：{snapshot.control_report}")
        self.timeline_label.setText(f"{snapshot.time:.1f} / {snapshot.duration:.0f}s")
        self.cpu_label.setText(f"CPU {snapshot.cpu_utilization * 100:.0f}%")
        self._sync_duration_input(snapshot)
        # 进度 = time/duration 换算到千分刻度；duration 为 0 时置 0 防除零。
        self.progress.setValue(round(snapshot.time / snapshot.duration * 1000) if snapshot.duration else 0)
        # 依据运行态启停各按钮：未加载配置(UNLOADED)时全部相关操作不可用。
        config_loaded = snapshot.run_state != "UNLOADED"
        self.play_button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        self.step_button.setEnabled(snapshot.run_state in {"READY", "PAUSED"})
        self.reset_button.setEnabled(config_loaded)
        # 仿真结束后禁止再注入扰动。
        for button in self.disturbance_buttons:
            button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        # 单个播放控制按钮始终显示“点下去会发生什么”。
        self.play_button.setText({"RUNNING": "暂停", "PAUSED": "继续"}.get(snapshot.run_state, "开始"))
        # 把快照下发给两视图与状态表；仅在需要时让视图自适应铺满。
        self.top_view.set_snapshot(snapshot, fit_view=fit_top_view)
        self.side_view.set_snapshot(snapshot)
        if fit_side_view:
            self.side_view.reset_view()
        self._sync_side_view_controls()
        self._update_tables(snapshot)
        self.features.on_snapshot_updated(self, snapshot)

    def _update_tables(self, snapshot: Snapshot) -> None:
        """更新 tables 状态。注意：保持界面显示和内部数据一致。"""
        # 节点表：前向保留待飞距(目标-本机)，高飘/右偏按飞行诊断习惯显示本机相对目标偏差。
        self.node_table.setRowCount(len(snapshot.nodes))
        for row, node in enumerate(snapshot.nodes):
            # 健康枚举翻译成中文；未知值原样显示。
            status = {"normal": "正常", "degraded": "降级", "fault": "故障", "lost": "失联"}.get(node.health, node.health)
            # 节点表固定展示五列；rally_phase 仅保留在快照中，不写入隐藏的越界列。
            values = [
                node.node_id,
                f"{node.track_pos_err_x:.1f}",
                f"{-node.track_pos_err_y:.1f}",
                f"{-node.track_pos_err_z:.1f}",
                status,
            ]
            for column, value in enumerate(values):
                self.node_table.setItem(row, column, self._centered_table_item(value))

        # 整体跟踪表：用长机代表当前全局航线跟踪情况，缺少显式长机时才回退首节点。
        self.overall_table.setRowCount(1 if snapshot.nodes else 0)
        if snapshot.nodes:
            leader = leader_node_from(snapshot.nodes)
            assert leader is not None
            side_offset = leader.cross_track_error
            if side_offset is None:
                side_offset = (leader.y - WORLD_HEIGHT / 2) * 0.8
            distance_to_go = leader.distance_to_go
            if distance_to_go is None:
                distance_to_go = max(0.0, (WORLD_WIDTH - leader.x) * 4)
            # 地速按水平面速度模长显示，不把垂向爬升率计入整体跟踪表。
            ground_speed = math.hypot(leader.vx, leader.vy)
            values = [
                f"{side_offset:.0f}",
                f"{distance_to_go:.0f}",
                f"{leader.altitude:.0f}",
                f"{ground_speed:.0f}",
                f"{leader.vertical_speed:.0f}",
            ]
            for column, value in enumerate(values):
                self.overall_table.setItem(0, column, self._centered_table_item(value))

        # 链路表：丢包率换算成百分比，ok 标志映射为正常/丢包文案。
        self.link_table.setRowCount(len(snapshot.links))
        for row, link in enumerate(snapshot.links):
            values = [
                f"{link.source}-{link.target}",
                link_direction_label(link.direction),
                f"{link.latency_ms}ms",
                f"{link.loss * 100:.0f}%",
                "正常" if link.ok else "丢包",
            ]
            for column, value in enumerate(values):
                self.link_table.setItem(row, column, self._centered_table_item(value))

    def _toggle_play_pause(self) -> None:
        """响应播放/暂停按钮。注意：按钮文案显示下一步动作。"""
        # RUNNING 下执行暂停，其余可用状态执行开始/继续；禁用态不会触发此槽。
        if self.sim.snapshot().run_state == "RUNNING":
            self._pause()
        else:
            self._start()

    def _start(self) -> None:
        """响应开始按钮并启动仿真。注意：需要同步按钮状态和日志。"""
        snapshot = self.sim.start()
        self._update_snapshot(snapshot)
        # 只有控制器确认 OK 才开启刷新定时器，避免空转。
        if self.sim.last_result_code == "OK":
            self.timer.start()
            self.features.on_controller_ready(self)
        self._log("UI", f"start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _pause(self) -> None:
        """响应暂停按钮并切换暂停状态。注意：暂停不清空当前快照。"""
        snapshot = self.sim.pause()
        # 暂停/继续切换：停或起刷新定时器与运行态保持一致。
        if snapshot.run_state == "PAUSED":
            self.timer.stop()
        elif snapshot.run_state == "RUNNING":
            self.timer.start()
        self._update_snapshot(snapshot)
        self._log("UI", f"pause/start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _step(self) -> None:
        """响应单步按钮并推进一拍。注意：单步后界面需要立即刷新。"""
        # 单步前先停掉自动刷新，确保只前进一拍。
        self.timer.stop()
        snapshot = self.sim.single_step()
        self._update_snapshot(snapshot)
        self._log("UI", f"step -> {self.sim.last_result_code}, state={snapshot.run_state}")
        if self.sim.last_result_code == "OK":
            self.features.on_controller_ready(self)

    def _reset(self) -> None:
        """响应重置按钮并恢复初始状态。注意：保留当前配置路径。"""
        self.timer.stop()
        snapshot = self.sim.reset()
        # 重置后队形回到初值，请求俯视图与侧视图重新自适应铺满。
        self._update_snapshot(snapshot, fit_top_view=True, fit_side_view=True)
        if self.sim.last_result_code == "OK":
            self.features.control_monitor.reset_if_open(self)
        else:
            self.features.on_controller_unavailable()
        self._log("SimControl", f"reset -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _on_tick(self) -> None:
        """处理 tick 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 定时轮询最新快照刷新界面（不推进仿真，推进在控制器线程内进行）。
        snapshot = self.sim.poll()
        self._update_snapshot(snapshot)
        # 进入非运行态(就绪/暂停/结束)就停掉定时器，省去无谓刷新。
        if snapshot.run_state in {"READY", "PAUSED", "FINISHED"}:
            self.timer.stop()

    def _refresh_formation_options(self) -> None:
        """按当前配置刷新“场景/队形”下拉选项。注意：程序回填期间屏蔽信号，避免误触发切换。"""
        names = self.sim.formation_names()
        combo = self.scenario_select
        with QSignalBlocker(combo):
            combo.clear()
            combo.addItems([f"{index}: {name}" for index, name in enumerate(names)])
            # 选中当前初始队形；无选项则保持空。
            if names:
                combo.setCurrentIndex(self.sim.formation_index(), emit=False)
        # 无队形或单队形时禁用（无从切换）。
        combo.setEnabled(len(names) > 1)

    def _on_scenario_selected(self) -> None:
        """响应“场景/队形”下拉选择并下发热切换命令。注意：失败时记录控制器返回信息。"""
        index = self.scenario_select.currentIndex()
        if index < 0:
            return
        snapshot = self.sim.switch_formation(index)
        self._update_snapshot(snapshot)
        self._log("Formation", f"切换队形 -> index={index}, {self.sim.last_result_code}, state={snapshot.run_state}")

    def _inject_disturbance(self, kind: str) -> None:
        """响应扰动按钮并下发扰动命令。注意：失败时需要记录控制器返回信息。"""
        messages = {
            "wind": "注入风场脉冲",
            "fault": "注入 A02 控制效率下降",
            "loss": "注入链路丢包",
            "clear": "清除运行期扰动",
        }
        snapshot = self.sim.inject_disturbance(kind)
        self._update_snapshot(snapshot)
        self._log("Disturb", f"{messages[kind]} -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _load_demo_config(self, filename: str) -> None:
        """加载 configs/ 目录下的预置演示配置。注意：文件不存在时记录告警。"""
        path = self.project_root / "configs" / filename
        if not path.exists():
            self._log("WARN", f"演示配置不存在：{path}")
            return
        self._apply_config_path(str(path))

    def _choose_config(self) -> None:
        """处理 config 选择流程。注意：用户取消时不改变当前配置。"""
        # 起始目录优先用上次配置所在目录，过滤常见配置扩展名。
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择配置文件",
            str(self._config_dialog_start_dir()),
            "Config (*.yaml *.yml *.json)",
        )
        # 用户取消则 path 为空，保持现状不变。
        if not path:
            return
        self._apply_config_path(path)

    def _apply_config_path(self, path: str, *, remember: bool = True) -> None:
        """应用 config path 设置。注意：只修改对应显示或运行参数。"""
        # 切换配置前停掉定时器，加载后请求自适应铺满新场景。
        self.timer.stop()
        snapshot = self.sim.load_config(path)
        if self.sim.last_result_code == "OK":
            # 切换配置必须重新应用半程尾迹；即使新旧配置时长相同，也不能沿用用户上次手动值。
            self._trail_duration_basis = None
            self._sync_speed_controls(self.sim.speed)
            # 按新配置刷新队形下拉框选项。
            self._refresh_formation_options()
            # 先注入障碍再铺满，使自适应视野包含障碍区。
            self._set_obstacles_from_config(path)
        self._update_snapshot(snapshot, fit_top_view=True, fit_side_view=True)
        if self.sim.last_result_code == "OK":
            # 成功：更新配置名标签/提示，并按需把该路径记入 config.ini。
            config_path = Path(path).resolve()
            self.current_config_path = config_path
            self._set_top_view_geo_origin_from_config(str(config_path))
            display_path = self._display_config_path(config_path)
            self.config_name.setText(display_path)
            self.config_name.setToolTip(display_path)
            self._log("Config", f"加载配置文件 {display_path}")
            # remember=False 用于“自动加载上次配置”场景，避免重复写回。
            if remember:
                self._save_last_config_path(config_path)
            self.features.on_controller_unavailable()
        else:
            # 失败只记录告警，不改动当前已加载配置。
            self._log("WARN", f"加载配置失败 {Path(path).name}: {self.sim.last_result_message}")

    def _set_top_view_geo_origin_from_config(self, path: str) -> None:
        """刷新俯视图点击坐标 origin。注意：无经纬航线时清空，避免沿用旧配置 origin。"""
        # origin 来自基础航线第一个经纬航点；旧 ENU 配置没有 origin，不能反推经纬度。
        self._top_view_geo_origin = _geo_origin_from_config_for_ui(path)
        self.top_view_coordinate.clear()
        if self._top_view_geo_origin is None:
            self.top_view_coordinate.setPlaceholderText("当前配置无经纬 origin")
        else:
            self.top_view_coordinate.setPlaceholderText("单击画布显示经纬度")

    def _on_top_view_point_clicked(self, east_m: float, north_m: float) -> None:
        """处理俯视图单击坐标。注意：只显示经纬度，不修改仿真状态。"""
        if self._top_view_geo_origin is None:
            # 失败提示也放进同一个输入框，避免用户误复制上一配置遗留坐标。
            self.top_view_coordinate.setText("当前配置无经纬 origin")
            self.top_view_coordinate.selectAll()
            return
        latitude_deg, longitude_deg = enu_to_geodetic(east_m, north_m, self._top_view_geo_origin)
        self.top_view_coordinate.setText(f"{longitude_deg:.7f}, {latitude_deg:.7f}")
        # 自动选中，用户单击后可直接 Ctrl+C 复制数字。
        self.top_view_coordinate.setFocus(Qt.FocusReason.OtherFocusReason)
        self.top_view_coordinate.selectAll()

    def _sync_speed_controls(self, speed: float) -> None:
        """同步 speed controls 显示。注意：程序设置滑条时不重复下发倍率。"""
        # 配置加载后控制器已持有倍率，这里只让滑条和文本追上当前真实倍率。
        slider_value = playback_rate_to_slider_value(speed)
        with QSignalBlocker(self.speed_slider):
            self.speed_slider.setValue(slider_value)
        self.speed_label.setText(f"{speed:.1f}x")

    def _config_dialog_start_dir(self) -> Path:
        """处理 dialog start dir 配置路径。注意：兼容源码运行和打包运行路径。"""
        # config.ini 里存的是相对项目根的路径；据此推出对话框起始目录。
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return self.project_root
        config_path = (self.project_root / relative_path).resolve()
        candidate = config_path.parent
        # 目录已不存在则退回项目根，避免对话框打开到无效位置。
        return candidate if candidate.exists() else self.project_root

    def _display_config_path(self, path: Path) -> str:
        """生成 config path 显示文本。注意：仅用于界面展示。"""
        # 优先展示相对项目根的路径，无法相对化时退而显示文件名。
        relative_path = self._relative_to_project_root(path)
        return relative_path if relative_path is not None else path.name

    def _load_last_config_from_state(self) -> None:
        """加载上次使用的配置路径。注意：路径不存在时回退到默认配置。"""
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return
        config_path = (self.project_root / relative_path).resolve()
        # 记录的配置文件可能已被删除/移动，缺失时仅告警不报错。
        if not config_path.exists():
            self._log("WARN", f"config.ini 指向的配置不存在：{relative_path}")
            return
        # remember=False：这是自动恢复，不需要再次写回。
        self._apply_config_path(str(config_path), remember=False)

    def _read_last_config_path(self) -> str | None:
        """读取 last config path 数据。注意：缺省或失败时应使用安全兜底。"""
        # 文件不存在直接返回空（首次运行属正常情况）。
        if not self.config_state_path.exists():
            return None
        parser = ConfigParser()
        try:
            parser.read(self.config_state_path, encoding="utf-8")
        except OSError as exc:
            # 读失败不致命，记录告警并按“无记录”处理。
            self._log("WARN", f"读取 config.ini 失败：{exc}")
            return None
        # 取 [config] last_config；空白串归一化为 None。
        value = parser.get(APP_CONFIG_SECTION, APP_CONFIG_KEY_LAST_CONFIG, fallback="").strip()
        return value or None

    def _save_last_config_path(self, path: Path) -> None:
        """保存 last config path 数据。注意：写入失败不应影响主仿真流程。"""
        # 只记录相对路径，便于项目整体移动后仍能定位；不可相对化则放弃记忆。
        relative_path = self._relative_to_project_root(path)
        if relative_path is None:
            self._log("WARN", "配置路径无法相对到程序目录，未更新 config.ini")
            return
        parser = ConfigParser()
        parser[APP_CONFIG_SECTION] = {APP_CONFIG_KEY_LAST_CONFIG: relative_path}
        # 确保父目录存在再写。
        self.config_state_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.config_state_path.open("w", encoding="utf-8") as handle:
                parser.write(handle)
        except OSError as exc:
            # 写盘失败不应中断主流程，记录告警即可。
            self._log("WARN", f"写入 config.ini 失败：{exc}")

    def _relative_to_project_root(self, path: Path) -> str | None:
        """计算 to project root 相对路径。注意：路径不可相对化时返回原始路径。"""
        try:
            # Windows 上跨盘符无法相对化会抛 ValueError。
            relative_path = os.path.relpath(path.resolve(), self.project_root)
        except ValueError:
            return None
        # relpath 仍返回绝对路径（如不同盘）则视为不可相对化。
        if os.path.isabs(relative_path):
            return None
        try:
            # 统一用正斜杠存储，跨平台一致。
            return Path(relative_path).as_posix()
        except ValueError:
            return None

    def _on_speed_changed(self, value: int) -> None:
        """处理 speed changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 滑块位置映射到离散倍率档位，低倍率细调，高倍率按大步长跳转。
        speed = slider_value_to_playback_rate(value)
        self.sim.set_speed(speed)
        self.speed_label.setText(f"{speed:.1f}x")

    def _on_segment_lock_changed(self) -> None:
        """处理 segment lock changed 信号回调。注意：只改变侧视图显示方式。"""
        checked = self.segment_lock.isChecked()
        previous_angle = self.side_view.current_view_angle_deg()
        if self.segment_lock.isEnabled():
            self._segment_lock_preferred = checked
        if not checked:
            self.side_view.view_angle_deg = previous_angle
        self.side_view.set_segment_locked(checked)
        self._sync_side_view_controls()

    def _on_view_angle_changed(self, value: int) -> None:
        """处理 view angle changed 信号回调。注意：航段锁定时滑条只显示自动值。"""
        with QSignalBlocker(self.view_angle_input):
            self.view_angle_input.setValue(value)
        with QSignalBlocker(self.view_angle_slider):
            self.view_angle_slider.setValue(value)
        if not self.segment_lock.isChecked():
            self.side_view.set_view_angle_deg(float(value))

    def _sync_side_view_controls(self) -> None:
        """同步侧视图控制状态。注意：程序刷新控件时不触发用户回调。"""
        lock_available = self.side_view.lock_available()
        locked = lock_available and self._segment_lock_preferred

        with QSignalBlocker(self.segment_lock):
            self.segment_lock.setEnabled(lock_available)
            self.segment_lock.setChecked(locked)
        if self.side_view.segment_locked != locked:
            self.side_view.set_segment_locked(locked)

        angle = round(self.side_view.current_view_angle_deg()) % 360
        with QSignalBlocker(self.view_angle_input):
            self.view_angle_input.setEnabled(not locked)
            self.view_angle_input.setValue(angle)
        with QSignalBlocker(self.view_angle_slider):
            self.view_angle_slider.setEnabled(not locked)
            self.view_angle_slider.setValue(angle)

    def _on_duration_changed(self) -> None:
        """处理 duration changed 信号回调。注意：只在非运行态下更新控制器时长。"""
        try:
            duration_s = float(self.duration_input.text())
        except ValueError:
            self._log("WARN", f"非法仿真时长：{self.duration_input.text()}")
            self._sync_duration_input(self.sim.snapshot())
            return
        snapshot = self.sim.set_duration(duration_s)
        self._update_snapshot(snapshot)
        if self.sim.last_result_code == "OK":
            try:
                self._persist_config_duration(duration_s)
            except Exception as exc:  # noqa: BLE001
                self._log("WARN", f"写入配置时长失败：{exc}")
            self._log("Config", f"设置仿真时长 {duration_s:g}s")
        else:
            self._log("WARN", f"设置仿真时长失败：{self.sim.last_result_message}")
            self._sync_duration_input(self.sim.snapshot())

    def _persist_config_duration(self, duration_s: float) -> None:
        """把当前仿真时长写回配置文件。注意：只更新 duration_s 字段。"""
        if self.current_config_path is None:
            raise ValueError("未加载配置文件")
        path = self.current_config_path
        suffix = path.suffix.lower()
        text = path.read_text(encoding="utf-8")
        if suffix == ".json":
            config = json.loads(text)
            if not isinstance(config, dict):
                raise ValueError("config root must be an object")
            config["duration_s"] = duration_s
            path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            return
        if suffix in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover - 依赖运行环境
                raise ValueError("YAML config requires PyYAML") from exc
            config = yaml.safe_load(text)
            if not isinstance(config, dict):
                raise ValueError("config root must be an object")
            config["duration_s"] = duration_s
            path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
            return
        raise ValueError("config must be .json, .yaml, or .yml")

    def _sync_duration_input(self, snapshot: Snapshot) -> None:
        """同步 duration input 显示。注意：加载配置后以控制器快照为准。"""
        if snapshot.run_state == "UNLOADED":
            self.duration_input.setEnabled(False)
            return
        self.duration_input.setText(self._format_duration_text(snapshot.duration))
        self.duration_input.setEnabled(snapshot.run_state in {"READY", "PAUSED"})
        self._sync_trail_seconds_for_duration(snapshot.duration)

    def _sync_trail_seconds_for_duration(self, duration_s: float) -> None:
        """按飞行时长同步默认尾迹长度。注意：同一时长不覆盖用户临时手动值。"""
        if getattr(self, "_trail_duration_basis", None) == duration_s:
            return
        seconds = trail_seconds_for_duration(duration_s)
        # 长时长配置的一半可能超过旧 600s 上限，先放宽范围再写入值，避免被控件截断。
        with QSignalBlocker(self.trail_seconds_input):
            self.trail_seconds_input.setRange(0.0, max(600.0, seconds))
            self.trail_seconds_input.setValue(seconds)
        # 记录本次同步的飞行时长；用户随后手动调尾迹时，不会被每帧刷新覆盖。
        self._trail_duration_basis = duration_s
        self._apply_trail_seconds(seconds, refresh_features=False)

    @staticmethod
    def _format_duration_text(duration_s: float) -> str:
        """格式化仿真时长文本。注意：整数秒不显示小数。"""
        if math.isfinite(duration_s) and duration_s.is_integer():
            return str(int(duration_s))
        return f"{duration_s:.3f}".rstrip("0").rstrip(".")

    def _set_theme(self, theme_key: str) -> None:
        """切换界面主题。注意：只改变显示，不改变仿真状态。"""
        self.theme_key = theme_key
        self.theme = THEMES[self.theme_key]
        self.light_theme_action.setChecked(theme_key == "light")
        self.dark_theme_action.setChecked(theme_key == "dark")
        self._apply_theme()
        theme_text = "浅色模式" if theme_key == "light" else "深色模式"
        self._log("UI", f"切换主题：{theme_text}")

    def _on_auto_center_changed(self) -> None:
        """处理 auto center changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 同步开关状态到两个视图，并立即用当前快照触发一次居中重排。
        checked = self.auto_center.isChecked()
        snapshot = self.sim.snapshot()
        self.top_view.auto_center = checked
        self.side_view.auto_center = checked
        self.top_view.set_snapshot(snapshot)
        self.side_view.set_snapshot(snapshot)
        self._sync_side_view_controls()

    def _on_grid_changed(self) -> None:
        """处理 grid changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 网格开关对俯视图与侧视图统一生效，并各自重绘。
        show_grid = self.grid_toggle.isChecked()
        self.top_view.show_grid = show_grid
        self.side_view.show_grid = show_grid
        self.top_view.viewport().update()
        self.side_view.update()

    def _on_link_visibility_changed(self) -> None:
        """处理通信链路显示开关。注意：只影响俯视图渲染，不修改链路状态。"""
        # 链路表与仿真数据保持不变，仅隐藏或恢复俯视图中的连线。
        self.top_view.show_links = self.legend_link.isChecked()
        self.top_view.viewport().update()

    def _on_trail_seconds_changed(self) -> None:
        """处理尾迹长度输入。注意：0 表示关闭尾迹显示与缓存。"""
        self._apply_trail_seconds(self.trail_seconds_input.value())

    def _apply_trail_seconds(self, seconds: float, *, refresh_features: bool = True) -> None:
        """把尾迹时长下发给数据源和视图。注意：输入框同步时也复用该路径。"""
        # 数据源负责缓存裁剪，视图负责按当前秒数即时隐藏或淡出已有快照。
        self.sim.set_trail_seconds(seconds)
        self.top_view.trail_seconds = seconds
        self.side_view.trail_seconds = seconds
        self.top_view.viewport().update()
        self.side_view.update()
        if refresh_features:
            # 手动修改尾迹时没有完整 _update_snapshot 流程，需要主动推送裁剪后的 3D payload。
            self.features.on_snapshot_updated(self, self.sim.snapshot())

    def _disable_auto_center(self) -> None:
        """关闭自动居中选项。注意：用户手动平移或缩放后应避免自动抢回视图。"""
        # 取消勾选会再次触发 _on_auto_center_changed，从而同步关闭俯视图自动居中。
        if self.auto_center.isChecked():
            self.auto_center.setChecked(False)

    def _reset_view(self) -> None:
        """响应重置视图按钮。注意：同时重置俯视图和侧视图显示范围。"""
        # 俯视图重置会经信号链触发侧视图自适应，这里再补一次重绘保证及时刷新。
        self.top_view.reset_view()
        self.side_view.update()

    def _toggle_fullscreen(self) -> None:
        """切换仿真画布全屏状态。注意：需要保存并恢复原布局。"""
        # 以是否已存在全屏窗口为标志在进入/退出之间切换。
        if self._stage_fullscreen_dialog is not None:
            self._exit_stage_fullscreen()
        else:
            self._enter_stage_fullscreen()

    def _enter_stage_fullscreen(self) -> None:
        """进入 stage fullscreen 模式。注意：需要保存退出时恢复的界面状态。"""
        # 前置条件：stage/布局存在且当前未全屏。
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is not None:
            return

        index = self.main_layout.indexOf(self.stage)
        if index < 0:
            return

        # 记录 stage 在主布局中的位置与拉伸系数，供退出时原样还原。
        self._stage_layout_index = index
        self._stage_layout_stretch = self.main_layout.stretch(index)
        self.main_layout.removeWidget(self.stage)

        # 用占位控件顶住原位，避免左右面板布局塌陷。
        self._stage_placeholder = QWidget()
        self.main_layout.insertWidget(self._stage_layout_index, self._stage_placeholder, self._stage_layout_stretch)

        # 把 stage 移入无边框全屏对话框（reparent 到对话框布局）。
        dialog = StageFullscreenDialog(self)
        dialog_layout = QVBoxLayout(dialog)
        dialog_layout.setContentsMargins(0, 0, 0, 0)
        dialog_layout.setSpacing(0)
        dialog_layout.addWidget(self.stage)
        self._stage_fullscreen_dialog = dialog
        self._set_fullscreen_button_state(True)
        dialog.showFullScreen()

    def _exit_stage_fullscreen(self) -> None:
        """退出 stage fullscreen 模式。注意：需要恢复进入前的布局状态。"""
        # 前置条件：处于全屏态。
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is None:
            return

        # 先把 stage 从对话框取出，再销毁对话框。
        dialog = self._stage_fullscreen_dialog
        dialog.layout().removeWidget(self.stage)
        dialog.hide()
        dialog.deleteLater()
        self._stage_fullscreen_dialog = None

        # 移除并销毁占位控件。
        if self._stage_placeholder is not None:
            placeholder_index = self.main_layout.indexOf(self._stage_placeholder)
            if placeholder_index >= 0:
                self.main_layout.removeWidget(self._stage_placeholder)
            self._stage_placeholder.deleteLater()
            self._stage_placeholder = None

        # 把 stage 插回原位置（用 min 兜底防止索引越界）并还原拉伸系数。
        insert_index = min(self._stage_layout_index, self.main_layout.count())
        self.main_layout.insertWidget(insert_index, self.stage, self._stage_layout_stretch)
        self._set_fullscreen_button_state(False)
        self.stage.show()
        # reparent 后强制重绘两视图，避免残留旧画面。
        self.top_view.update()
        self.side_view.update()

    def _set_fullscreen_button_state(self, active: bool) -> None:
        """设置 fullscreen button state 状态。注意：保持控件状态和内部标志同步。"""
        if self.fullscreen_button is None:
            return
        # 全屏时显示“退出”图标/提示，否则显示“进入全屏”。
        self.fullscreen_button.setText("↙" if active else "⛶")
        self.fullscreen_button.setToolTip("退出全屏" if active else "全屏显示")
        self.fullscreen_button.setAccessibleName("退出全屏" if active else "全屏显示")

    def _open_offline_plot(self) -> None:
        """打开离线控制误差回放窗口。"""
        self.features.control_monitor.open_offline_plot(self)

    def _open_data_analysis_window(self) -> None:
        """打开离线控制效果数据分析窗口。"""
        self.features.data_analysis.open(self)

    def _open_situation3d_window(self) -> None:
        """打开 3D 态势窗口。注意：重复触发复用同一个窗口实例。"""
        self.features.situation3d.open(self)

    def _update_situation3d_snapshot(self, snapshot: Snapshot) -> None:
        """同步 3D 态势窗口数据。注意：窗口未打开时不产生额外 QML 更新。"""
        self.features.situation3d.update_snapshot(self, snapshot)

    def _open_live_monitor(self) -> None:
        """打开实时控制监控窗口。"""
        self.features.control_monitor.open_live_monitor(self)

    def _log(self, source: str, message: str) -> None:
        """追加一条界面日志。注意：日志容量由日志面板负责裁剪。"""
        self.log_dialog.append(self.sim.time, source, message)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """处理窗口关闭事件。注意：关闭前需要释放控制器资源。"""
        # 关窗前停定时器并释放控制器资源，避免后台线程泄漏。
        self.timer.stop()
        self.features.close()
        self.sim.close()
        super().closeEvent(event)
