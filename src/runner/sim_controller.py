"""SimulationController 主体。注意：公开入口由 sim_control.py 兼容导出。"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import replace
from typing import Callable

from src.algorithm.context.leaf_types import FormPosS, PosTrackDiagS, WayLineS, WayPointInputS, to_display_inputs
from src.algorithm.units.process.tra_plan.leader_route import waypoint_inputs_to_waylines
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
    _build_rally_join_geometry,
    _build_rally_task_init,
    _build_vel_cmd_limit,
    _leader_id_from_nodes,
    _motion_from_aircraft_state,
)
from src.runner.sim_control_snapshot import SimulationControllerSnapshotMixin
from src.runner.sim_control_types import (
    CommandResult,
    ControlReport,
    DisturbanceCommand,
    DisturbanceType,
    EventLevel,
    RunState,
    SimulationEvent,
    SimulationSnapshot,
    Subscription,
    TimedSnapshotCursor,
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
        # 定时快照游标代号独立于仿真时间；时间归零后旧游标也不能误跳过新样本。
        self._timed_snapshot_generation = 0
        # 配置派生状态：load_config 成功后由 _init_modules_unlocked 统一填充。
        self._node_algorithms: dict[str, _NodeAlgorithm] = {}
        self._node_roles: dict[str, str] = {}
        self._configured_links: list[_ConfiguredLink] = []
        self._leader_route: list[WayPointInputS] | None = None
        self._display_route: list[WayLineS] | None = None  # 显示用航线(WayLineS)，仅供 GUI 画航段
        self._blocked_display_route: list[WayLineS] | None = None  # 避障覆盖时保留的原配置航线，供 GUI 标示封锁状态。
        # 避障”采用”的长机航线覆盖：非 None 时替换配置生成的长机航线（reset 保留，load_config 清除）。
        self._leader_route_override: list[WayPointInputS] | None = None
        self._formation_names: list[str] = []  # 各队形名字（供界面下拉框显示，索引=队形序号）
        self._formation_index: int = 0  # 当前/初始队形索引，供界面下拉框预选
        self._formation_slots: list[list[FormPosS]] = []  # 各队形槽位表(FUR 标称坐标)，供快照透出
        # 文件日志开关：配置 log_enabled 决定默认值，override 供 ST/批处理强制开启。
        self._file_log_enabled = False
        self._file_log_override: bool | None = None
        # 各节点最近一次算法链路单步耗时（毫秒），按算法分频节拍更新。
        self._algo_step_ms: dict[str, float] = {}
        self._rally_geometry: dict[str, object] = {}  # RallyPlanGeometryState 按 node_id 索引，供 GUI 展示两个盘旋圆。
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
        self._playback_rate = 1.0
        self._cpu_utilization = 0.0
        self._algorithm_decimation = _DEFAULT_ALGORITHM_DECIMATION
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        # 日志采样周期取 10Hz 与算法周期中更快者；构造期默认值随配置加载刷新。
        self._log_sample_period_s = min(_LOG_SAMPLE_PERIOD_S, self._algorithm_period_s)
        self._next_log_sample_time_s = self._log_sample_period_s
        # 对外状态和事件缓存用于 GUI 状态栏、日志窗口和测试断言。
        self._run_state = RunState.UNLOADED
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

    def set_file_log_enabled(self, enabled: bool | None) -> None:
        """强制开启/关闭文件日志落盘。注意：None 表示跟随配置 log_enabled；对已打开的日志不生效。"""

        # override 独立于配置存放：load_config 会按配置刷新默认值，但不得覆盖调用方的强制意图。
        with self._lock:
            self._file_log_override = enabled

    def validate_file_log(self) -> CommandResult:
        """刷新并验收本次文件日志。注意：供依赖落盘产物的无界面入口在成功退出前调用。"""

        with self._lock:
            if not self._file_log_effective_unlocked():
                return CommandResult("ERR_LOG_FAILED", "文件日志未启用")
            self._logger.flush()
            if self._logger._file_logging_disabled:
                message = self._logger.last_error_message or "文件日志已因写入失败停用"
                return CommandResult("ERR_LOG_FAILED", message)
            if not self._logger.opened or self._logger.run_dir is None:
                return CommandResult("ERR_LOG_FAILED", "日志文件未打开")
            expected_files = ("config.json", self._logger.snapshot_filename, "events.jsonl")
            missing_files = [
                filename
                for filename in expected_files
                if not (self._logger.run_dir / filename).is_file()
            ]
            if missing_files:
                return CommandResult(
                    "ERR_LOG_FAILED",
                    f"日志文件缺失: {', '.join(missing_files)}",
                )
            if self._logger.persisted_snapshot_count != len(self._logger.snapshots):
                return CommandResult("ERR_LOG_FAILED", "快照日志未完整写入")
            if self._logger.persisted_event_count != len(self._logger.events):
                return CommandResult("ERR_LOG_FAILED", "事件日志未完整写入")
            if self._run_state != RunState.FINISHED or not self._logger.snapshots:
                return CommandResult("ERR_LOG_FAILED", "日志中缺少仿真终态快照")
            terminal_snapshot = self._logger.snapshots[-1]
            if (
                terminal_snapshot.run_state != RunState.FINISHED
                or abs(terminal_snapshot.time_s - self._duration_s) > _TIME_EPSILON_S
            ):
                return CommandResult("ERR_LOG_FAILED", "日志末帧不是完整仿真终态")
            return CommandResult("OK", "日志已完整落盘")

    def _file_log_effective_unlocked(self) -> bool:
        """返回当前生效的文件日志开关。注意：override 优先于配置默认值。"""
        if self._file_log_override is not None:
            return self._file_log_override
        return self._file_log_enabled

    def load_config(self, path: str, *, seed: int = 0) -> CommandResult:
        """读取仿真配置并按运行入参 seed 初始化。注意：配置内同名字段不参与选择。"""

        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            return CommandResult("ERR_INVALID_ARGUMENT", "seed must be a non-negative integer")

        # 先做轻量前置校验：已关闭或运行中不允许加载新配置。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == RunState.RUNNING:
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
            if self._run_state == RunState.RUNNING:
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
            # 新配置：清除上一个配置遗留的避障航线覆盖，回到该配置的原始长机航线。
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config, seed)
            except Exception as exc:  # noqa: BLE001 - 首版统一映射模块初始化失败
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 加载成功转入 READY/待命，准备 start。
            self._run_state = RunState.READY
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", f"配置已加载: {path}")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "config loaded")

    def get_snapshot(self) -> SimulationSnapshot:
        """获取当前仿真快照。注意：该操作不推进仿真时间。"""

        with self._lock:
            if self._config is not None and self._run_state == RunState.RUNNING:
                # 显式查询应返回当前状态；调用频率由 UI 计时器或外部调用方控制。
                self._latest_snapshot = self._make_snapshot_unlocked()
            return self._latest_snapshot

    def read_timed_snapshots(
        self,
        cursor: TimedSnapshotCursor | None,
    ) -> tuple[TimedSnapshotCursor, tuple[SimulationSnapshot, ...]]:
        """增量读取固定仿真时钟快照。注意：旧代游标会从新运行索引零重新开始。"""

        with self._lock:
            generation = self._timed_snapshot_generation
            snapshot_count = len(self._logger.snapshots)
            if cursor is None or cursor.run_generation != generation:
                # 首次读取或运行代变化时从零开始，确保新运行首批样本不会被旧索引跳过。
                start_index = 0
            else:
                # 防御外部构造的越界游标，避免负索引反向读取或超长索引永久漏样本。
                start_index = min(snapshot_count, max(0, int(cursor.next_index)))
            # 返回不可变容器副本；调用方无法增删 logger 持有的原始列表。
            snapshots = tuple(self._logger.snapshots[start_index:snapshot_count])
            next_cursor = TimedSnapshotCursor(generation, snapshot_count)
        return next_cursor, snapshots

    def start(self, *, auto_rally: bool = False) -> CommandResult:
        """启动或继续运行。注意：auto_rally 只在 READY 首次启动时原子触发集结。"""

        should_stop_worker = False
        # 第一段持锁：做状态前置校验，并判断是否需要先回收残留旧线程。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            # 已结束必须先 reset 才能重跑；运行中重复 start 视为幂等成功。
            if self._run_state == RunState.FINISHED:
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == RunState.RUNNING:
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
            if self._run_state == RunState.FINISHED:
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == RunState.RUNNING:
                return CommandResult("OK", "already running")
            # 切到运行态，清停止标志并拉起后台线程开始自动推进。
            previous_state = self._run_state
            self._run_state = RunState.RUNNING
            # PAUSED 表示继续既有任务，不能把入口参数误解为再次下发集结命令。
            if auto_rally and previous_state == RunState.READY:
                rally_result = self._start_rally_unlocked()
                if rally_result.code != "OK":
                    self._run_state = previous_state
                    self._control_report = self._derive_control_report_unlocked()
                    self._latest_snapshot = self._make_snapshot_unlocked()
                    return rally_result
            self._control_report = self._derive_control_report_unlocked()
            self._cpu_utilization = 0.0
            self._stop_requested.clear()
            self._start_worker_unlocked()
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "started")

    def start_rally(self) -> CommandResult:
        """开始集结流程。注意：只把待命集结节点切到 RALLY，不改变运行/暂停状态。"""
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before rally")
            if self._run_state == RunState.READY:
                return CommandResult("ERR_INVALID_STATE", "请先开始运行")
            if self._run_state == RunState.FINISHED:
                return CommandResult("ERR_INVALID_STATE", "集结已结束，请重置后重试")
            if self._run_state not in {RunState.RUNNING, RunState.PAUSED}:
                return CommandResult("ERR_INVALID_STATE", "当前状态不能开始集结")
            result = self._start_rally_unlocked()
            if result.code != "OK":
                return result
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "开始集结")

    def _start_rally_unlocked(self) -> CommandResult:
        """在已持锁状态下触发集结。注意：调用方负责校验控制器运行状态。"""

        rally_algorithms = {
            node_id: algorithm
            for node_id, algorithm in self._node_algorithms.items()
            if algorithm.is_rally_role()
        }
        if not rally_algorithms:
            return CommandResult("ERR_INVALID_STATE", "当前配置没有集结节点")
        # 首 tick 立即触发时先建立待命圆，避免 RallyJoinPos 直接沿旧点到圆切线启动。
        self._prime_rally_standby_unlocked(rally_algorithms)
        results = [algorithm.start_rally() for algorithm in rally_algorithms.values()]
        if not any(ok for ok, _message in results):
            message = next((message for _ok, message in results if message), "当前状态不能开始集结")
            return CommandResult("ERR_INVALID_STATE", message)
        self._control_report = self._derive_control_report_unlocked()
        self._append_event_unlocked("INFO", "SimControl", "开始集结")
        self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "开始集结")

    def pause(self) -> CommandResult:
        """暂停 SimulationController 的运行流程。注意：只暂停调度，不清空当前状态。"""

        with self._lock:
            # 运行->暂停：仅改状态与回报，不动模型数据，便于随后 step 或继续。
            if self._run_state == RunState.RUNNING:
                self._run_state = RunState.PAUSED
                self._control_report = self._derive_control_report_unlocked()
                self._cpu_utilization = 0.0
                self._latest_snapshot = self._make_snapshot_unlocked()
                snapshot = self._latest_snapshot
            elif self._run_state == RunState.PAUSED:
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
            if self._run_state == RunState.RUNNING:
                return CommandResult("ERR_INVALID_STATE", "pause before manual step")
            if self._run_state == RunState.FINISHED:
                return CommandResult("ERR_INVALID_STATE", "reset before stepping")
            # 单步语义即"暂停态下手动推进 count 个 tick"。
            self._run_state = RunState.PAUSED
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
                if self._run_state == RunState.FINISHED:
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
                self._init_modules_unlocked(config, self._seed)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 重置后回到 READY/待命，等待再次 start。
            self._run_state = RunState.READY
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
            if self._run_state == RunState.RUNNING:
                return CommandResult("ERR_BUSY", "pause or reset before applying a route")
            config = dict(self._config)
        # 先停后台线程（锁外），再持锁带覆盖重建模块（时间归零，等价一次 reset）。
        self._stop_worker()
        with self._lock:
            self._leader_route_override = route
            try:
                self._init_modules_unlocked(config, self._seed)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = RunState.READY
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
            if self._run_state == RunState.RUNNING:
                return CommandResult("ERR_BUSY", "pause or reset before clearing the route")
            if self._leader_route_override is None:
                return CommandResult("OK", "no avoidance route to clear")
            config = dict(self._config)
        self._stop_worker()
        with self._lock:
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config, self._seed)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = RunState.READY
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
            if self._run_state == RunState.RUNNING:
                return CommandResult("ERR_INVALID_STATE", "pause before setting duration")
            if self._run_state == RunState.FINISHED:
                return CommandResult("ERR_INVALID_STATE", "reset before setting duration")
            # 缩短到当前时间之前会制造“时间回退但模型未回滚”的不一致快照，必须拒绝。
            if duration_s + _TIME_EPSILON_S < self._time_s:
                return CommandResult("ERR_INVALID_ARGUMENT", "duration_s must not be before current time")
            self._duration_s = float(duration_s)
            self._config["duration_s"] = self._duration_s
            # 若总时长刚好等于当前时间，应立即按新的边界结束。
            if self._time_s >= self._duration_s:
                self._time_s = self._duration_s
                self._run_state = RunState.FINISHED
                self._control_report = self._derive_control_report_unlocked()
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
            if self._run_state == RunState.FINISHED:
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
            # 定位掌机任务；普通保持长机和集结长机都通过 set_pattern_index 切换队形。
            leader_id = next(
                (nid for nid, role in self._node_roles.items() if role in {"leader", "rally_leader"}),
                None,
            )
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

    def run_until_complete(self, config: object | str, *, seed: int = 0) -> CommandResult:
        """同步运行到仿真结束。注意：主要供 CLI 或批处理使用。"""

        # config 可为文件路径（走 load_config）或内联 dict（直接校验+初始化）。
        if isinstance(config, str):
            result = self.load_config(config, seed=seed)
            if result.code != "OK":
                return result
        elif isinstance(config, dict):
            if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
                return CommandResult("ERR_INVALID_ARGUMENT", "seed must be a non-negative integer")
            with self._lock:
                config_copy = dict(config)
                try:
                    self._config_loader.validate(config_copy)
                    self._init_modules_unlocked(config_copy, seed)
                except Exception as exc:  # noqa: BLE001
                    return CommandResult("ERR_CONFIG_INVALID", str(exc))
                self._run_state = RunState.READY
                self._latest_snapshot = self._make_snapshot_unlocked()
        else:
            return CommandResult("ERR_INVALID_ARGUMENT", "config must be path or dict")

        # 同步推进：在当前线程持锁连续 tick 直到状态机离开 RUNNING（到时长结束）。
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before run")
            self._run_state = RunState.RUNNING
            rally_algorithms = {
                node_id: algorithm
                for node_id, algorithm in self._node_algorithms.items()
                if algorithm.is_rally_role()
            }
            if rally_algorithms:
                # 批处理没有 GUI 按钮可点，集结配置进入 RUNNING 后自动触发一次开始集结。
                # 自动触发发生在首 tick 之前，必须与 GUI 等待一拍后的待命圆合同保持一致。
                self._prime_rally_standby_unlocked(rally_algorithms)
                results = [
                    algorithm.start_rally()
                    for algorithm in rally_algorithms.values()
                ]
                if any(ok for ok, _message in results):
                    # 与 GUI start_rally() 保持同类事件，方便日志侧统一检索。
                    self._append_event_unlocked("INFO", "SimControl", "开始集结")
            self._control_report = self._derive_control_report_unlocked()
            while self._run_state == RunState.RUNNING:
                try:
                    # 不强制产帧:日志按采样周期落盘与 force 无关(见 _tick_unlocked 的
                    # should_log_snapshot 分支),终态快照由 FINISHED 分支保底刷新;
                    # 全程持锁无人读中间快照,逐 tick 强制构建纯属浪费(约占批处理 1/3 耗时)。
                    self._tick_unlocked()
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
        return CommandResult("OK", "finished")

    def _init_modules_unlocked(self, config: dict[str, object], seed: int) -> None:
        """在已持锁状态下初始化仿真模块。注意：不得在未加载配置时调用。"""
        # 缓存配置并记录本次运行 seed；配置原值不作为输入，只在日志副本中被实际值覆盖。
        self._config = dict(config)
        self._seed = seed
        self._config["seed"] = seed
        self._duration_s = float(config.get("duration_s", 120.0))
        self._step_s = float(config.get("step_s", 0.005))
        self._playback_rate = float(config.get("playback_rate", 1.0))
        self._algorithm_decimation = int(config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION))
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        # 日志采样不得低于控制频率（docs/codex指标.md §6.2）：算法快于 10Hz 时对齐算法节拍，
        # 否则耗时/原始指令/饱和序列会隔拍丢失，P95/峰值/TV/占空比漏掉瞬态。
        self._log_sample_period_s = min(_LOG_SAMPLE_PERIOD_S, self._algorithm_period_s)
        # 时间与计数归零，保证每次初始化都是干净起点。
        self._time_s = 0.0
        self._tick_index = 0
        self._next_log_sample_time_s = self._log_sample_period_s
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
        # 缓存各队形槽位表（长机 FUR 标称坐标），供快照透出评测用槽位上下文。
        self._formation_slots = [list(row) for row in formation_comm_init.formPos]
        # 文件日志开关默认关闭：大数据量场景避免 10Hz JSON 序列化与磁盘 IO 拖慢仿真。
        self._file_log_enabled = bool(config.get("log_enabled", False))
        # 新配置清空耗时缓存，避免节点集合变化后残留旧值。
        self._algo_step_ms = {}
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
        # 覆盖航线生效时，原配置航线仍需作为已封锁参考线对外展示；重建时先清空避免 reset 后残留。
        self._blocked_display_route = None
        if self._leader_route_override is not None:
            # 用同一个 config 重新按"配置原始航线"规则生成一份，不复用 leader_route——
            # override 生效后 leader_route 已经是替换后的规划航线，不能再当作"原始航线"用。
            _blocked_wpi = _build_leader_route(config, insert_arcs=False)
            if len(_blocked_wpi) >= 2:
                self._blocked_display_route = waypoint_inputs_to_waylines(_blocked_wpi)
        # 集结场景复用任务航线首点和首段，构造任务配置及每机目标集结点。
        rally_task_init = _build_rally_task_init(config, self._algorithm_period_s, list(nodes))
        rally_approach_speed = _build_rally_approach_speed(config)
        # 集结辅助几何只保留两个盘旋圆，init 时按当前生效航线和已解析初始状态计算。
        self._rally_geometry = _build_rally_join_geometry(
            list(nodes), leader_route, formation_comm_init, rally_task_init, states
        )
        rally_leader_id = _leader_id_from_nodes(list(nodes))
        rally_layer_altitudes = self._build_rally_layer_altitudes(
            list(nodes), leader_route, formation_comm_init, rally_task_init, rally_leader_id
        )
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
                rally_cfg=rally_task_init,
                rally_leader_id=rally_leader_id,
                rally_approach_speed_mps=rally_approach_speed,
                rally_layer_altitude_m=rally_layer_altitudes.get(node_id),
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
        # 只有完整模块初始化成功到达末尾才换代，失败的配置加载不会制造空运行代。
        self._timed_snapshot_generation += 1

    def _build_rally_layer_altitudes(
        self,
        nodes: list[object],
        route: list[WayPointInputS],
        formation_comm_init: object,
        rally_task_init: object | None,
        rally_leader_id: str,
    ) -> dict[str, float]:
        """计算集结 JOINING 前的分层高度。注意：长机在基准层，僚机按槽位顺序上下交替。"""
        if not route or rally_task_init is None:
            return {}
        # separation=0 表示显式关闭高度分层，此时保持原始集结槽位高度。
        separation = max(0.0, float(getattr(rally_task_init, "altitude_separation_m", 60.0)))
        if separation <= 0.0:
            return {}
        # 先建立 node_id -> role，后续按角色过滤，避免把普通保持节点纳入高度层。
        roles = {
            node_id_from_config(node, index): str(node.get("role") or "")
            for index, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        rally_ids: list[str] = []
        # 长机固定放在第 0 层，后续僚机围绕它上下交替分层。
        if rally_leader_id and roles.get(rally_leader_id) == "rally_leader":
            rally_ids.append(rally_leader_id)
        target_pattern = int(getattr(rally_task_init, "targetPattern", 0))
        form_pos = getattr(formation_comm_init, "formPos", [])
        # 僚机顺序优先采用队形槽位顺序；这比配置节点顺序更贴近编队语义。
        slot_ids = [
            slot.id
            for slot in (form_pos[target_pattern] if 0 <= target_pattern < len(form_pos) else [])
            if roles.get(slot.id) == "rally_follower"
        ]
        for node_id in slot_ids:
            if node_id not in rally_ids:
                rally_ids.append(node_id)
        # 防御性兜底：槽位表缺失或角色节点未出现在队形中时，仍给它一个确定高度层。
        for node_id, role in roles.items():
            if role in {"rally_leader", "rally_follower"} and node_id not in rally_ids:
                rally_ids.append(node_id)
        base_h = route[0].pos.h
        multipliers = [0]
        layer = 1
        # 分层序列：0, +1, -1, +2, -2 ...，使相邻飞机尽量不在同一高度。
        while len(multipliers) < len(rally_ids):
            multipliers.extend([layer, -layer])
            layer += 1
        # 返回绝对高度，实体不再需要知道全队顺序或基准高度来源。
        return {
            node_id: base_h + multipliers[index] * separation
            for index, node_id in enumerate(rally_ids)
        }

    def _tick_unlocked(self, *, force_snapshot: bool = False) -> SimulationSnapshot | None:
        """在已持锁状态下推进一个仿真 tick。注意：调用方负责锁和阶段检查。"""
        # 仅在运行/暂停态推进；其他状态直接回最近快照，不产生副作用。
        if self._run_state not in {RunState.RUNNING, RunState.PAUSED}:
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
        if self._time_s + _TIME_EPSILON_S >= self._duration_s:
            # 浮点累计可能略小于精确边界，进入终态时统一钳到配置总时长。
            self._time_s = self._duration_s
            self._run_state = RunState.FINISHED
            self._control_report = self._derive_control_report_unlocked()
        elif self._run_state in {RunState.RUNNING, RunState.PAUSED}:
            self._control_report = self._derive_control_report_unlocked()

        should_refresh_display = (
            self._should_refresh_display_unlocked() or self._run_state == RunState.FINISHED
        )
        # 日志按仿真时间等间隔采样（10Hz 与算法频率取快者），不同播放倍率数据点一致。
        should_log_snapshot = self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s
        # 快照生成按墙钟显示频率限流；日志采样点额外生成，避免漏记关键状态。
        snapshot: SimulationSnapshot | None = None
        if force_snapshot or should_refresh_display or should_log_snapshot:
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        # 关键数据通常按采样周期记录；结束时额外强制落一帧，避免非采样点时长丢失末段。
        should_persist_snapshot = should_log_snapshot or self._run_state == RunState.FINISHED
        if should_persist_snapshot and snapshot is not None:
            if not self._logger.write_snapshot(snapshot):
                self._append_event_unlocked("WARN", "DataLogger", f"snapshot log failed: {self._logger.last_error_message}")
        # 若单个 tick 跨过多个采样点，只记录当前最新状态一次并推进全部已越过边界。
        if should_log_snapshot:
            while self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s:
                self._next_log_sample_time_s += self._log_sample_period_s
        # 仅当强制产帧、达到显示刷新间隔或仿真结束时才回传快照，否则返回 None 抑制 UI 刷新。
        if force_snapshot or should_refresh_display:
            return self._latest_snapshot
        return None

    def _ensure_logger_open_unlocked(self) -> None:
        """确保当前运行已创建日志目录。注意：打开失败只记录 WARN，不阻断 tick。"""
        # 文件日志默认关闭：只影响磁盘落盘，内存快照/事件仍正常记录（供尾迹与 GUI 日志窗口）。
        if not self._file_log_effective_unlocked():
            return
        if self._config is None or self._logger.opened or self._logger._file_logging_disabled:
            return
        if not self._logger.open(f"run-seed-{self._seed}-{int(time.time())}", self._config):
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
            # 逐节点计时算法单步耗时，作为评测"计算代价"证据随快照落盘。
            step_started = time.perf_counter()
            output = self._node_algorithms[node_id].step(
                state, inbox, self._time_s, health_map.get(node_id, "normal")
            )
            self._algo_step_ms[node_id] = (time.perf_counter() - step_started) * 1000.0
            controls[node_id] = output.control
            diagnostics[node_id] = replace(output.control_diag)
            # 汇总各节点待发消息，统一在本轮末尾交给通信模块。
            outbox.extend(output.outbox)
            status_values.append(output.status)
        # 缓存本轮控制，供后续未跑算法的 tick 继续施加（保持-上次值语义）。
        self._current_controls = controls
        self._control_diagnostics = diagnostics
        self._model.apply_controls(controls)
        # 集中发送：消息在通信模块内按时延/丢包规则投递。
        self._comm.send(outbox)
        # 任一节点非正常组队（如重构）即把全局控制回报置为"重构"。
        if any(status != "forming" for status in status_values):
            self._control_report = "重构"

    def _prime_rally_standby_unlocked(self, rally_algorithms: dict[str, _NodeAlgorithm]) -> None:
        """在首个仿真 tick 前只预热集结节点待命算法，不触碰通信和全局控制状态。"""
        # 已产生真实 tick 时，各实体已经通过正常调度建立待命几何，无需重复执行。
        if self._tick_index != 0:
            return
        # 运行模式是预热开关，不能仅凭 tick_index 推断节点仍在等待开始集结。
        # 重复 start_rally() 时节点已切到 ACTIVE，不能在同一仿真时刻额外推进一次算法。
        if not any(
            algorithm.current_rally_phase_str() == "LOCAL_LOITER"
            for algorithm in rally_algorithms.values()
        ):
            return
        states = self._model.read_states()
        health_map = self._disturbance.read_health()
        # 临时空 inbox 只供本次待命初始化；输出直接丢弃，不消费真实收件箱、不发送消息。
        for node_id, algorithm in rally_algorithms.items():
            if algorithm.current_rally_phase_str() != "LOCAL_LOITER":
                continue
            algorithm.step(states[node_id], [], self._time_s, health_map.get(node_id, "normal"))

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
        # 已是结构化命令也统一规范枚举，避免调用方手工构造时绕过类型校验。
        if isinstance(command, DisturbanceCommand):
            return DisturbanceCommand(
                type=DisturbanceType(command.type),
                target=command.target,
                duration_s=command.duration_s,
                params=dict(command.params),
            )
        if not isinstance(command, dict):
            raise TypeError("command must be DisturbanceCommand or dict")
        # 枚举是唯一允许集合；非法字符串在控制器边界转成参数错误。
        try:
            command_type = DisturbanceType(command.get("type"))
        except (TypeError, ValueError):
            raise ValueError("invalid disturbance type")
        params = command.get("params", {})
        if not isinstance(params, dict):
            raise ValueError("params must be a dict")
        # duration_s 可缺省（None 表示持续到显式 clear）。
        duration = command.get("duration_s")
        return DisturbanceCommand(
            type=command_type,
            target=str(command["target"]) if command.get("target") is not None else None,
            duration_s=float(duration) if duration is not None else None,
            params=dict(params),
        )
