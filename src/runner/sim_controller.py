"""SimulationController 主体。注意：公开入口由 sim_control.py 兼容导出。"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import replace
from typing import Callable

from src.algorithm.context.leaf_types import PosInEarthS, PosTrackDiagS, WayLineS, WayPointInputS, to_display_inputs
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.common.envelope import MessageEnvelope
from src.environment.comm import CommunicationChannel
from src.environment.model import AccelerationCommand, ModelIterator, node_id_from_config
from src.runner.sim_control_constants import (
    _COMM_DECIMATION,
    _DEFAULT_ALGORITHM_DECIMATION,
    _LOG_SAMPLE_PERIOD_S,
    _MAX_PLAYBACK_RATE,
    _MIN_PLAYBACK_RATE,
    _TIME_EPSILON_S,
)
from src.runner.sim_control_loop import SimulationControllerLoopMixin
from src.runner.sim_control_modules import _ConfigLoader, _DataLogger, _DisturbanceEngine, _NodeAlgorithm
from src.runner.sim_control_routes import (
    _build_formation_comm_init,
    _build_leader_route,
    _build_rally_approach_speed,
    _build_rally_route,
    _build_rally_task_init,
    _build_vel_cmd_limit,
    _leader_id_from_nodes,
    _motion_from_aircraft_state,
    _route_point_from_config,
    _route_state_from_wayline,
)
from src.runner.sim_control_snapshot import SimulationControllerSnapshotMixin
from src.runner.sim_control_types import (
    CommandResult,
    ControlReport,
    DisturbanceCommand,
    EventLevel,
    LinkState,
    NodeState,
    RouteState,
    RunState,
    SimulationEvent,
    SimulationSnapshot,
    Subscription,
    _ConfiguredLink,
)

class SimulationController(SimulationControllerLoopMixin, SimulationControllerSnapshotMixin):
    """顶层仿真编排门面。注意：对 GUI/CLI 暴露统一控制接口。"""

    # 该类保留公开控制接口和跨模块状态，具体循环与快照组装由 mixin 承担。
    # 所有可变运行状态都受 _lock 保护；耗时 IO 和线程 join 必须放在锁外。
    # _model/_comm/_disturbance/_logger 是长期子模块，load_config/reset 只重建其内部状态。
    # _leader_route_override 只表示 GUI 采用的避障航线覆盖，新配置加载时必须清除。
    # _latest_snapshot 是 GUI/订阅者读取的最后稳定快照，不要求每次查询都推进时间。
    # 订阅回调在锁外通知，避免 UI 回调反向调用控制器造成重入死锁。
    _EVENT_BUFFER_SIZE = 1000
    _DISPLAY_REFRESH_S = 0.1

    def __init__(self) -> None:
        """初始化 SimulationController 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._lock = threading.RLock()
        # 基础运行子模块：构造阶段只创建对象，不加载配置、不启动线程。
        self._config_loader = _ConfigLoader()
        self._model = ModelIterator()
        self._comm = CommunicationChannel()
        self._disturbance = _DisturbanceEngine()
        self._logger = _DataLogger()
        # 配置派生状态：load_config 成功后由 _init_modules_unlocked 统一填充。
        self._node_algorithms: dict[str, _NodeAlgorithm] = {}
        self._node_roles: dict[str, str] = {}
        self._configured_links: list[_ConfiguredLink] = []
        self._leader_route: list[WayPointInputS] | None = None
        self._display_route: list[WayLineS] | None = None  # 显示用航线(WayLineS)，仅供 GUI 画航段
        # 避障”采用”的长机航线覆盖：非 None 时替换配置生成的长机航线（reset 保留，load_config 清除）。
        self._leader_route_override: list[WayPointInputS] | None = None
        self._formation_completed_analysis: object | None = None  # FormationAnalysisS；集结完成后锁存
        self._formation_names: list[str] = []  # 各队形名字（供界面下拉框显示，索引=队形序号）
        self._formation_index: int = 0  # 当前/初始队形索引，供界面下拉框预选
        # 控制输出缓存按节点 ID 存放，模型 tick 前后都能生成一致快照。
        self._current_controls: dict[str, AccelerationCommand] = {}
        self._control_diagnostics: dict[str, PosTrackDiagS] = {}
        # 时间与调度参数来自配置，默认值只用于 UNLOADED/空快照阶段。
        self._config: dict[str, object] | None = None
        self._seed = 0
        self._duration_s = 0.0
        self._step_s = 0.005
        self._time_s = 0.0
        self._tick_index = 0
        self._next_log_sample_time_s = _LOG_SAMPLE_PERIOD_S
        self._playback_rate = 1.0
        self._cpu_utilization = 0.0
        self._algorithm_decimation = _DEFAULT_ALGORITHM_DECIMATION
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        # 对外状态和事件缓存用于 GUI 状态栏、日志窗口和测试断言。
        self._run_state: RunState = "UNLOADED"
        self._control_report: ControlReport = "待命"
        self._latest_snapshot = self._make_snapshot_for_empty_controller()
        self._events: deque[SimulationEvent] = deque(maxlen=self._EVENT_BUFFER_SIZE)
        # 订阅者使用 callback 去重，避免 GUI 重复订阅造成多次刷新。
        self._subscribers: dict[int, Callable[[SimulationSnapshot], None]] = {}
        self._subscriber_ids_by_callback: dict[Callable[[SimulationSnapshot], None], int] = {}
        self._next_subscription_id = 1
        # 后台线程只负责自动推进；单步和 reset 仍在调用线程内完成。
        self._last_display_wall_s = 0.0
        self._worker: threading.Thread | None = None
        self._stop_requested = threading.Event()
        self._closed = False

    @property
    def playback_rate(self) -> float:
        """返回当前播放倍率。注意：只反映墙钟调度倍率，不改变仿真步长。"""

        with self._lock:
            return self._playback_rate

    def load_config(self, path: str) -> CommandResult:
        """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""

        # 先做轻量前置校验：已关闭或运行中不允许加载新配置。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
        # 文件读取与解析放在锁外（可能耗时 IO），按异常类型映射结果码。
        try:
            config = self._config_loader.load(path)
        except FileNotFoundError:
            return CommandResult("ERR_CONFIG_NOT_FOUND", f"config not found: {path}")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return CommandResult("ERR_CONFIG_INVALID", str(exc))

        # 再次持锁并复检状态（IO 期间状态可能变化），随后初始化模块。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
            # 新配置：清除上一个配置遗留的避障航线覆盖，回到该配置的原始长机航线。
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001 - 首版统一映射模块初始化失败
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 加载成功转入 READY/待命，准备 start。
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", f"配置已加载: {path}")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "config loaded")

    def get_snapshot(self) -> SimulationSnapshot:
        """获取当前仿真快照。注意：该操作不推进仿真时间。"""

        with self._lock:
            if self._config is not None and self._run_state == "RUNNING":
                # 显式查询应返回当前状态；调用频率由 UI 计时器或外部调用方控制。
                self._latest_snapshot = self._make_snapshot_unlocked()
            return self._latest_snapshot

    def start(self) -> CommandResult:
        """启动或继续 SimulationController 的运行流程。注意：重复调用应保持状态一致。"""

        should_stop_worker = False
        # 第一段持锁：做状态前置校验，并判断是否需要先回收残留旧线程。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            # 已结束必须先 reset 才能重跑；运行中重复 start 视为幂等成功。
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            should_stop_worker = self._worker is not None and self._worker.is_alive()

        # 停线程需阻塞 join，必须在锁外做，避免与 _run_loop 持锁互锁。
        if should_stop_worker:
            self._stop_worker()

        # 第二段持锁：释锁期间状态可能变化，重新校验后再真正启动。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            # 切到运行态，清停止标志并拉起后台线程开始自动推进。
            self._run_state = "RUNNING"
            self._control_report = self._derive_control_report_unlocked()
            self._cpu_utilization = 0.0
            self._stop_requested.clear()
            self._start_worker_unlocked()
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "started")

    def pause(self) -> CommandResult:
        """暂停 SimulationController 的运行流程。注意：只暂停调度，不清空当前状态。"""

        with self._lock:
            # 运行->暂停：仅改状态与回报，不动模型数据，便于随后 step 或继续。
            if self._run_state == "RUNNING":
                self._run_state = "PAUSED"
                self._control_report = "保持"
                self._cpu_utilization = 0.0
                self._latest_snapshot = self._make_snapshot_unlocked()
                snapshot = self._latest_snapshot
            elif self._run_state == "PAUSED":
                # 重复暂停幂等返回成功。
                return CommandResult("OK", "already paused")
            else:
                # READY/FINISHED/UNLOADED 下暂停无意义，报状态错误。
                return CommandResult("ERR_INVALID_STATE", "pause requires RUNNING or PAUSED")
        # 注意：后台线程在下一圈检测到非 RUNNING 会自行退出，这里不显式停线程。
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "paused")

    def step(self, count: int = 1) -> CommandResult:
        """推进 SimulationController 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""

        if count < 1:
            return CommandResult("ERR_INVALID_ARGUMENT", "count must be >= 1")
        snapshots_to_notify: list[SimulationSnapshot] = []
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before step")
            # 单步只在非自动运行时允许：RUNNING 下须先 pause，FINISHED 下须先 reset。
            if self._run_state == "RUNNING":
                return CommandResult("ERR_INVALID_STATE", "pause before manual step")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before stepping")
            # 单步语义即"暂停态下手动推进 count 个 tick"。
            self._run_state = "PAUSED"
            self._control_report = "保持"
            for _ in range(count):
                try:
                    # force_snapshot 保证每个手动步都产出快照，便于逐帧观察。
                    snapshot = self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
                if snapshot is not None:
                    snapshots_to_notify.append(snapshot)
                # 中途到达总时长即停止剩余步进。
                if self._run_state == "FINISHED":
                    break
            # 若全程无新快照，至少回传一帧最近快照以刷新 UI。
            if not snapshots_to_notify:
                snapshots_to_notify.append(self._latest_snapshot)
        for snapshot in snapshots_to_notify:
            self._notify_subscribers(snapshot)
        return CommandResult("OK", "stepped")

    def reset(self) -> CommandResult:
        """复位 SimulationController 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""

        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before reset")
            # 取出当前配置副本，重置=用同一配置重新初始化所有模块（时间归零）。
            config = dict(self._config)
        # 先停后台线程（锁外），再持锁重建模块，避免线程与重建竞争。
        self._stop_worker()
        with self._lock:
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 重置后回到 READY/待命，等待再次 start。
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "仿真已重置")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "reset")

    def apply_avoidance_route(self, route: list[WayPointInputS]) -> CommandResult:
        """采用一条避障规划航线，替换长机航线并重置到 READY。注意：运行中需先暂停。"""
        if not isinstance(route, list) or len(route) < 2:
            return CommandResult("ERR_CONFIG_INVALID", "avoidance route must be a list of at least 2 WayPointInputS")
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before applying a route")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before applying a route")
            config = dict(self._config)
        # 先停后台线程（锁外），再持锁带覆盖重建模块（时间归零，等价一次 reset）。
        self._stop_worker()
        with self._lock:
            self._leader_route_override = route
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "已采用避障航线")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "avoidance route applied")

    def clear_avoidance_route(self) -> CommandResult:
        """清除避障航线覆盖，恢复配置原始长机航线并重置到 READY。"""
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config first")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before clearing the route")
            if self._leader_route_override is None:
                return CommandResult("OK", "no avoidance route to clear")
            config = dict(self._config)
        self._stop_worker()
        with self._lock:
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "已清除避障航线")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "avoidance route cleared")

    def close(self) -> None:
        """释放 SimulationController 持有的资源。注意：关闭后不应继续调用运行接口。"""

        # 先停后台线程，再持锁逐个关闭子系统并清空订阅，最后置已关闭标志。
        self._stop_worker()
        with self._lock:
            self._logger.flush()
            self._logger.close()
            self._model.close()
            self._comm.close()
            self._disturbance.close()
            for algorithm in self._node_algorithms.values():
                algorithm.close()
            self._node_algorithms.clear()
            self._subscribers.clear()
            self._subscriber_ids_by_callback.clear()
            # 置位后所有控制接口都将拒绝服务。
            self._closed = True

    def set_playback_rate(self, rate: float) -> CommandResult:
        """设置播放倍率。注意：只影响墙钟调度，不改变仿真步长。"""

        # 倍率限定在允许范围内，仅改墙钟节流，不改仿真步长（结果可复现）。
        if not _MIN_PLAYBACK_RATE <= rate <= _MAX_PLAYBACK_RATE:
            return CommandResult(
                "ERR_INVALID_ARGUMENT",
                f"rate must be in [{_MIN_PLAYBACK_RATE}, {_MAX_PLAYBACK_RATE}]",
            )
        with self._lock:
            self._playback_rate = float(rate)
            if self._config is not None:
                # reset 会用当前配置副本重建模块，需同步运行期倍率避免回退到文件默认值。
                self._config["playback_rate"] = self._playback_rate
        return CommandResult("OK", "playback rate updated")

    def set_duration(self, duration_s: float) -> CommandResult:
        """设置仿真总时长。注意：只允许在未自动运行时修改。"""

        # 总时长必须是正有限值，避免进度条和结束条件进入不可判定状态。
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            return CommandResult("ERR_INVALID_ARGUMENT", "duration_s must be positive")
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before setting duration")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_INVALID_STATE", "pause before setting duration")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before setting duration")
            # 缩短到当前时间之前会制造“时间回退但模型未回滚”的不一致快照，必须拒绝。
            if duration_s + _TIME_EPSILON_S < self._time_s:
                return CommandResult("ERR_INVALID_ARGUMENT", "duration_s must not be before current time")
            self._duration_s = float(duration_s)
            self._config["duration_s"] = self._duration_s
            # 若总时长刚好等于当前时间，应立即按新的边界结束。
            if self._time_s >= self._duration_s:
                self._time_s = self._duration_s
                self._run_state = "FINISHED"
                self._control_report = "保持"
            self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "duration updated")

    def inject_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> CommandResult:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""

        # 先把 dict/对象统一规范为 DisturbanceCommand，非法参数提前返回。
        try:
            normalized = self._normalize_disturbance(command)
        except (TypeError, ValueError) as exc:
            return CommandResult("ERR_INVALID_ARGUMENT", str(exc))
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before disturbance")
            # 仿真结束后不再接受扰动（轨迹已定型）。
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "disturbance is not accepted after finish")
            # 以当前仿真时间为基准注入扰动并记录事件。
            event = self._disturbance.inject(normalized, self._time_s)
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
            self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "disturbance injected")

    def get_formation_names(self) -> list[str]:
        """返回当前配置的队形名字列表。注意：索引即 switch_formation 的整型队形号；未加载配置时为空。"""
        with self._lock:
            return list(self._formation_names)

    def get_formation_index(self) -> int:
        """返回当前队形索引。注意：供界面下拉框预选；初值来自配置 initial_index，随 switch_formation 更新。"""
        with self._lock:
            return self._formation_index

    def switch_formation(self, index: int) -> CommandResult:
        """运行时热切换编队队形（改长机保持任务的目标队形索引）。注意：不重建模块、不复位时间，下一算法拍生效。"""
        # index 必须是合法整型且落在已配置队形范围内。
        try:
            index = int(index)
        except (TypeError, ValueError):
            return CommandResult("ERR_INVALID_ARGUMENT", "formation index must be an integer")
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before switching formation")
            if not self._formation_names:
                return CommandResult("ERR_INVALID_STATE", "no formation configured")
            if index < 0 or index >= len(self._formation_names):
                return CommandResult("ERR_INVALID_ARGUMENT", f"formation index out of range: {index}")
            # 定位长机保持任务；只有 HOLD 场景的长机实体持有可切换的 Hold 任务。
            leader_id = next((nid for nid, role in self._node_roles.items() if role == "leader"), None)
            algorithm = self._node_algorithms.get(leader_id) if leader_id is not None else None
            task = getattr(getattr(algorithm, "_entity", None), "_task", None)
            setter = getattr(task, "set_pattern_index", None)
            if setter is None:
                return CommandResult("ERR_INVALID_STATE", "current scenario does not support formation switch")
            # 改长机目标队形索引：广播随下一拍下发，僚机从已下发的槽位表切到对应行。
            setter(index)
            self._formation_index = index
            self._append_event_unlocked("INFO", "SimControl", f"切换队形: {self._formation_names[index]}")
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "formation switched")

    def subscribe_snapshot(self, callback: Callable[[SimulationSnapshot], None]) -> Subscription:
        """订阅快照刷新回调。注意：回调应快速返回，避免阻塞仿真线程。"""

        with self._lock:
            # 同一回调去重：已订阅则复用其 ID，避免重复登记导致多次触发。
            subscription_id = self._subscriber_ids_by_callback.get(callback)
            if subscription_id is None:
                subscription_id = self._next_subscription_id
                self._next_subscription_id += 1
                self._subscribers[subscription_id] = callback
                self._subscriber_ids_by_callback[callback] = subscription_id
            snapshot = self._latest_snapshot

        def unsubscribe() -> None:
            """取消订阅回调。注意：回调不存在时应保持幂等。"""
            with self._lock:
                # 双向映射一并清理，pop 默认值保证重复取消不报错。
                removed = self._subscribers.pop(subscription_id, None)
                if removed is not None:
                    self._subscriber_ids_by_callback.pop(removed, None)

        # 订阅时立即推一帧当前快照，让新订阅者无需等待下一 tick 即可初始化显示。
        try:
            callback(snapshot)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._append_event_unlocked("WARN", "SimControl", f"snapshot callback failed: {exc}")
        return Subscription(unsubscribe)

    def get_recent_events(
        self,
        limit: int = 200,
        min_level: EventLevel | None = None,
    ) -> list[SimulationEvent]:
        """读取最近事件列表。注意：返回副本供 UI 展示。"""

        if limit < 1:
            return []
        # 数值化日志级别用于阈值过滤（仅返回不低于 min_level 的事件）。
        level_order = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40}
        min_value = level_order.get(min_level or "DEBUG", 10)
        with self._lock:
            events = [event for event in self._events if level_order[event.level] >= min_value]
            # 取最近 limit 条（事件队列已按时间追加）。
            return events[-limit:]

    def run_until_complete(self, config: object | str, *, seed: int | None = None) -> CommandResult:
        """同步运行到仿真结束。注意：主要供 CLI 或批处理使用。"""

        # config 可为文件路径（走 load_config）或内联 dict（直接校验+初始化）。
        if isinstance(config, str):
            result = self.load_config(config)
            if result.code != "OK":
                return result
        elif isinstance(config, dict):
            with self._lock:
                config_copy = dict(config)
                # 允许参数 seed 覆盖配置内 seed，便于批量复现实验。
                if seed is not None:
                    config_copy["seed"] = seed
                try:
                    self._config_loader.validate(config_copy)
                    self._init_modules_unlocked(config_copy)
                except Exception as exc:  # noqa: BLE001
                    return CommandResult("ERR_CONFIG_INVALID", str(exc))
                self._run_state = "READY"
                self._latest_snapshot = self._make_snapshot_unlocked()
        else:
            return CommandResult("ERR_INVALID_ARGUMENT", "config must be path or dict")

        # 同步推进：在当前线程持锁连续 tick 直到状态机离开 RUNNING（到时长结束）。
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before run")
            self._run_state = "RUNNING"
            self._control_report = self._derive_control_report_unlocked()
            while self._run_state == "RUNNING":
                try:
                    # force_snapshot 确保每帧都落日志/快照（批处理需完整轨迹）。
                    self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
        return CommandResult("OK", "finished")

    def _init_modules_unlocked(self, config: dict[str, object]) -> None:
        """在已持锁状态下初始化仿真模块。注意：不得在未加载配置时调用。"""
        # 缓存配置并读取核心运行参数（种子、总时长、步长、倍率）。
        self._config = dict(config)
        self._seed = int(config.get("seed", 0))
        self._duration_s = float(config.get("duration_s", 120.0))
        self._step_s = float(config.get("step_s", 0.005))
        self._playback_rate = float(config.get("playback_rate", 1.0))
        self._algorithm_decimation = int(config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION))
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        # 时间与计数归零，保证每次初始化都是干净起点。
        self._time_s = 0.0
        self._tick_index = 0
        self._next_log_sample_time_s = _LOG_SAMPLE_PERIOD_S
        self._last_display_wall_s = 0.0
        self._cpu_utilization = 0.0
        # 按依赖顺序初始化各子系统：先模型（提供初始状态），再通信，再扰动（依赖前两者）。
        self._model.init(config, self._seed)
        raw_links = list(config.get("links") or [])
        comm_config = {
            "nodes": list(config.get("nodes") or []),
            "links": raw_links,
        }
        self._comm.init(comm_config, self._seed)
        # 保存折叠前的链路声明，供快照阶段合并双向状态。
        self._configured_links = self._parse_configured_links(raw_links)
        # 扰动引擎需持有模型与通信句柄，故最后初始化并注入二者。
        self._disturbance.init(config, self._seed, self._model, self._comm)
        nodes = config.get("nodes") or []
        # 建立 node_id->角色映射；首节点缺省 leader，其余 wingman。
        self._node_roles = {
            node_id_from_config(node, i): str(
                node.get("role") or ("leader" if i == 0 else "wingman")
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 从模型读取各机初始状态，用于构造算法与初始长机运动基准。
        states = self._model.read_states()
        # 由拓扑与队形配置生成编队通信初始化信息（网络连接 + 槽位）。
        formation_comm_init = _build_formation_comm_init(list(nodes), raw_links, config)
        # 缓存队形名字供界面选择；索引即 switch_formation 下发的整型队形号。
        self._formation_names = list(formation_comm_init.formPat)
        self._formation_index = int(formation_comm_init.initialPattern)
        leader_id = _leader_id_from_nodes(list(nodes))
        initial_leader_state = states.get(leader_id)
        # 把长机初始状态转换为算法侧运动表示，供僚机持队参考；无长机则为 None。
        initial_leader_motion = (
            _motion_from_aircraft_state(initial_leader_state)
            if initial_leader_state is not None
            else None
        )
        # 避障”采用”后用覆盖航线替换配置航线；否则按配置生成。
        if self._leader_route_override is not None:
            leader_route = self._leader_route_override
        else:
            leader_route = _build_leader_route(config)
        self._leader_route = leader_route
        # 前向/垂向速度指令限幅(串级 P+PI 外环输出)，由配置注入各节点实体。
        vel_cmd_limit = _build_vel_cmd_limit(config)
        # 显示用航线(list[WayLineS])：只画航段几何，去掉交接半径 r(转弯信息)，与配置航线显示一致。
        if self._leader_route_override is not None:
            _display_wpi = to_display_inputs(self._leader_route_override)
        else:
            _display_wpi = _build_leader_route(config, insert_arcs=False)
        self._display_route = waypoint_inputs_to_waylines(_display_wpi) if len(_display_wpi) >= 2 else None
        # 集结场景额外参数：集结航线、任务配置、每机目标集结点。
        rally_route = _build_rally_route(config)
        rally_task_init = _build_rally_task_init(config, self._algorithm_period_s, list(nodes))
        rally_approach_speed = _build_rally_approach_speed(config)
        self._formation_completed_analysis = None
        rally_leader_id = _leader_id_from_nodes(list(nodes))
        _node_rally_targets: dict[str, PosInEarthS | None] = {
            node_id_from_config(node, i): (
                _route_point_from_config(node["rally_target"], f"nodes[{i}].rally_target")
                if isinstance(node.get("rally_target"), dict)
                else None
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 为每个节点创建算法适配器（角色决定实体类型）。
        self._node_algorithms = {
            node_id: _NodeAlgorithm(
                node_id,
                self._node_roles.get(node_id, "wingman"),
                formation_comm_init,
                initial_leader_motion,
                leader_route,
                self._algorithm_period_s,
                vel_cmd_limit,
                rally_route=rally_route,
                rally_cfg=rally_task_init,
                rally_target=_node_rally_targets.get(node_id),
                rally_leader_id=rally_leader_id,
                rally_approach_speed_mps=rally_approach_speed,
            )
            for node_id in states
        }
        # 控制指令缓存初始化为零加速度，首个算法 tick 前模型保持初值。
        self._current_controls = {
            node_id: AccelerationCommand()
            for node_id in states
        }
        self._control_diagnostics = {
            node_id: PosTrackDiagS()
            for node_id in states
        }
        # 仅重置内存日志；文件目录延迟到首次实际 tick 时创建，避免空 run 目录。
        self._logger.reset()

    def _tick_unlocked(self, *, force_snapshot: bool = False) -> SimulationSnapshot | None:
        """在已持锁状态下推进一个仿真 tick。注意：调用方负责锁和阶段检查。"""
        # 仅在运行/暂停态推进；其他状态直接回最近快照，不产生副作用。
        if self._run_state not in {"RUNNING", "PAUSED"}:
            return self._latest_snapshot
        self._ensure_logger_open_unlocked()
        step_s = self._step_s
        tick_index = self._tick_index
        algorithm_tick = tick_index % self._algorithm_decimation == 0

        # 分频调度：算法链路按配置分频运行，控制频率低于积分频率以降低算力开销。
        if algorithm_tick:
            self._run_formation_algorithms_unlocked()
        # 通信分频推进，传入累计步长以保持时延计时一致。
        if tick_index % _COMM_DECIMATION == 0:
            self._comm.tick(step_s * _COMM_DECIMATION)

        # 先把当前控制指令施加到模型（算法分频更新，未更新的 tick 沿用上次控制）。
        self._model.apply_controls(self._current_controls)
        # 推进扰动引擎并落地其产生的事件（如扰动到期）。
        for event in self._disturbance.tick(self._time_s, step_s):
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
        # 模型积分一步，随后推进仿真时间；用 min 夹住，确保不越过总时长。
        self._model.step(step_s)
        self._time_s = min(self._duration_s, self._time_s + step_s)
        self._tick_index += 1

        # 状态机收尾：到达总时长则置 FINISHED 并锁定回报；否则在运行态刷新回报文本。
        if self._time_s >= self._duration_s:
            self._run_state = "FINISHED"
            self._control_report = "保持"
        elif self._run_state == "RUNNING":
            self._control_report = self._derive_control_report_unlocked()

        should_refresh_display = self._should_refresh_display_unlocked() or self._run_state == "FINISHED"
        # 日志按仿真时间固定 10Hz 采样，保证不同播放倍率得到一致的离线数据点。
        should_log_snapshot = self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s
        # 快照生成按墙钟显示频率限流；日志采样点额外生成，避免漏记关键状态。
        snapshot: SimulationSnapshot | None = None
        if force_snapshot or should_refresh_display or should_log_snapshot:
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        # 关键数据定时记录固定 10Hz；若单个 tick 跨过多个采样点，只记录当前最新状态一次。
        if should_log_snapshot and snapshot is not None:
            if not self._logger.write_snapshot(snapshot):
                self._append_event_unlocked("WARN", "DataLogger", f"snapshot log failed: {self._logger.last_error_message}")
            while self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s:
                self._next_log_sample_time_s += _LOG_SAMPLE_PERIOD_S
        # 仅当强制产帧、达到显示刷新间隔或仿真结束时才回传快照，否则返回 None 抑制 UI 刷新。
        if force_snapshot or should_refresh_display:
            return self._latest_snapshot
        return None

    def _ensure_logger_open_unlocked(self) -> None:
        """确保当前运行已创建日志目录。注意：打开失败只记录 WARN，不阻断 tick。"""
        if self._config is None or self._logger.opened or self._logger._file_logging_disabled:
            return
        if not self._logger.open(f"run-{int(time.time())}", self._config):
            self._append_event_unlocked("WARN", "DataLogger", f"open log failed: {self._logger.last_error_message}")

    def _run_formation_algorithms_unlocked(self) -> None:
        """运行编队算法链路。注意：算法输入应使用当前模型状态快照。"""
        # 取一致的输入快照：所有节点基于同一时刻的模型状态与健康表计算，避免步内串扰。
        states = self._model.read_states()
        health_map = self._disturbance.read_health()
        controls: dict[str, AccelerationCommand] = {}
        diagnostics: dict[str, PosTrackDiagS] = {}
        outbox: list[MessageEnvelope] = []
        status_values: list[str] = []
        for node_id, state in states.items():
            # 每个节点先取走自己的收件箱（读取即清空），再驱动其算法一步。
            inbox = self._comm.read_inbox(node_id)
            output = self._node_algorithms[node_id].step(
                state, inbox, self._time_s, health_map.get(node_id, "normal")
            )
            controls[node_id] = output.control
            diagnostics[node_id] = replace(output.control_diag)
            # 汇总各节点待发消息，统一在本轮末尾交给通信模块。
            outbox.extend(output.outbox)
            status_values.append(output.status)
            # 集结完成首帧：锁存分析结果，供快照透传给 UI。
            if output.formation_analysis is not None:
                self._formation_completed_analysis = output.formation_analysis
        # 缓存本轮控制，供后续未跑算法的 tick 继续施加（保持-上次值语义）。
        self._current_controls = controls
        self._control_diagnostics = diagnostics
        self._model.apply_controls(controls)
        # 集中发送：消息在通信模块内按时延/丢包规则投递。
        self._comm.send(outbox)
        # 任一节点非正常组队（如重构）即把全局控制回报置为"重构"。
        if any(status != "forming" for status in status_values):
            self._control_report = "重构"

    def _notify_subscribers(self, snapshot: SimulationSnapshot) -> None:
        """通知所有快照订阅者。注意：回调异常不应破坏控制器状态。"""
        # 先在锁内拷贝订阅者列表，再在锁外回调，避免回调期间长时间持锁。
        with self._lock:
            subscribers = list(self._subscribers.values())
        for callback in subscribers:
            try:
                callback(snapshot)
            except Exception as exc:  # noqa: BLE001
                # 单个回调异常被隔离记录，不影响其他订阅者与仿真推进。
                with self._lock:
                    self._append_event_unlocked("WARN", "SimControl", f"snapshot callback failed: {exc}")

    def _append_event_unlocked(self, level: EventLevel, source: str, message: str) -> None:
        """在已持锁状态下追加事件文本。注意：事件列表会按容量裁剪。"""
        # 用当前仿真时间打戳，事件同时入内存队列并落日志。
        event = SimulationEvent(self._time_s, level, source, message)
        self._append_event_object_unlocked(event)
        self._logger.write_event(event)

    def _append_event_object_unlocked(self, event: SimulationEvent) -> None:
        """在已持锁状态下追加事件对象。注意：时间戳使用当前仿真时间。"""
        self._events.append(event)

    def _normalize_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> DisturbanceCommand:
        """规范化扰动命令。注意：兼容 GUI 和脚本的不同字段写法。"""
        # 已是结构化命令则直接透传。
        if isinstance(command, DisturbanceCommand):
            return command
        if not isinstance(command, dict):
            raise TypeError("command must be DisturbanceCommand or dict")
        # 校验扰动类型在允许集合内。
        command_type = command.get("type")
        if command_type not in {"wind", "node_fault", "link_loss", "link_fault", "clear"}:
            raise ValueError("invalid disturbance type")
        params = command.get("params", {})
        if not isinstance(params, dict):
            raise ValueError("params must be a dict")
        # duration_s 可缺省（None 表示持续到显式 clear）。
        duration = command.get("duration_s")
        return DisturbanceCommand(
            type=command_type,  # type: ignore[arg-type]
            target=str(command["target"]) if command.get("target") is not None else None,
            duration_s=float(duration) if duration is not None else None,
            params=dict(params),
        )
