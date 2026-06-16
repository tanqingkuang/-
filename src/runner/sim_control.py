"""Simulation control facade.

The controller implements the application contract described in
``docs/1-仿真控制HLD.md``. The UAV model is provided by
``src.environment.model``; communication, algorithm, disturbance, and logging
remain first-pass local implementations.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Literal

from src.common.envelope import MessageEnvelope
from src.environment.model import AccelerationCommand, AircraftState, ModelIterator, node_id_from_config


RunState = Literal["UNLOADED", "READY", "RUNNING", "PAUSED", "FINISHED"]
ControlReport = Literal["待命", "集结", "保持", "重构"]
EventLevel = Literal["DEBUG", "INFO", "WARN", "ERROR"]
DisturbanceType = Literal["wind", "node_fault", "link_loss", "link_fault", "clear"]
ResultCode = Literal[
    "OK",
    "ERR_NO_CONFIG",
    "ERR_CONFIG_NOT_FOUND",
    "ERR_CONFIG_INVALID",
    "ERR_INVALID_STATE",
    "ERR_INVALID_ARGUMENT",
    "ERR_BUSY",
    "ERR_MODULE_INIT_FAILED",
    "ERR_TICK_FAILED",
    "ERR_LOG_FAILED",
    "ERR_INTERNAL",
]


@dataclass(frozen=True)
class NodeState:
    """UI/CLI-facing state for one aircraft node."""

    node_id: str
    role: str
    health: str
    # ENU position: x=east, y=north, altitude=up.
    x_m: float
    y_m: float
    altitude_m: float
    psi_v_deg: float
    theta_deg: float
    speed_mps: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    nx: float
    nz: float
    phi_deg: float


@dataclass(frozen=True)
class LinkState:
    """UI/CLI-facing state for one communication link."""

    link_id: str
    direction: str
    latency_ms: float
    loss_rate: float
    status: str


@dataclass(frozen=True)
class SimulationSnapshot:
    """Complete realtime observation payload."""

    time_s: float
    duration_s: float
    step_s: float
    run_state: RunState
    control_report: ControlReport
    nodes: list[NodeState]
    links: list[LinkState]


@dataclass(frozen=True)
class SimulationEvent:
    """Recent event entry for UI log windows and CLI diagnostics."""

    time_s: float
    level: EventLevel
    source: str
    message: str


@dataclass(frozen=True)
class CommandResult:
    """Result of an application-layer command."""

    code: ResultCode
    message: str = ""


@dataclass(frozen=True)
class DisturbanceCommand:
    """Dynamic disturbance command accepted by ``inject_disturbance``."""

    type: DisturbanceType
    target: str | None = None
    duration_s: float | None = None
    params: dict[str, object] = field(default_factory=dict)


class Subscription:
    """Handle returned by ``subscribe_snapshot``."""

    def __init__(self, unsubscribe: Callable[[], None]) -> None:
        self._unsubscribe = unsubscribe
        self._active = True

    def unsubscribe(self) -> None:
        """Remove the callback from the controller."""

        if self._active:
            self._unsubscribe()
            self._active = False


@dataclass
class _NodeAlgorithmOutput:
    control: AccelerationCommand
    outbox: list[MessageEnvelope]
    status: str


class _ConfigLoader:
    """Minimal JSON/YAML loader for the first controller implementation."""

    def load(self, path: str) -> dict[str, object]:
        config_path = Path(path)
        if not config_path.exists():
            raise FileNotFoundError(path)
        text = config_path.read_text(encoding="utf-8")
        if config_path.suffix.lower() == ".json":
            data = json.loads(text)
        elif config_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover - depends on env
                raise ValueError("YAML config requires PyYAML") from exc
            data = yaml.safe_load(text)
        else:
            raise ValueError("config must be .json, .yaml, or .yml")
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        self.validate(data)
        return dict(data)

    def validate(self, config: dict[str, object]) -> None:
        duration_s = float(config.get("duration_s", 120.0))
        step_s = float(config.get("step_s", 0.005))
        playback_rate = float(config.get("playback_rate", 1.0))
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if step_s <= 0:
            raise ValueError("step_s must be positive")
        if not 0.1 <= playback_rate <= 10.0:
            raise ValueError("playback_rate must be in [0.1, 10.0]")
        nodes = config.get("nodes", [])
        links = config.get("links", [])
        model = config.get("model", {})
        if nodes is not None and not isinstance(nodes, list):
            raise ValueError("nodes must be a list")
        if links is not None and not isinstance(links, list):
            raise ValueError("links must be a list")
        ModelIterator._parse_model_config(model)


class _CommunicationEngine:
    """Deterministic communication stub with link status outputs."""

    def __init__(self) -> None:
        self._links: list[LinkState] = []
        self._base_links: list[LinkState] = []
        self._inbox: dict[str, list[MessageEnvelope]] = {}
        self._pending: list[MessageEnvelope] = []
        self._loss_until_s = 0.0
        self._active_loss_rate = 0.25
        self._active_latency_delta_ms = 40.0
        self._time_s = 0.0

    def init(self, topology_config: dict[str, object], qos_config: dict[str, object], seed: int) -> None:
        del qos_config, seed
        self._time_s = 0.0
        self._loss_until_s = 0.0
        self._active_loss_rate = 0.25
        self._active_latency_delta_ms = 40.0
        configured_links = topology_config.get("links", [])
        if configured_links is None:
            configured_links = []
        if not isinstance(configured_links, list):
            raise ValueError("links must be a list")
        links: list[LinkState] = []
        for index, link in enumerate(configured_links):
            if not isinstance(link, dict):
                raise ValueError("each link must be an object")
            link_id = str(link.get("link_id") or link.get("id") or "")
            if "-" not in link_id:
                source = str(link.get("source", f"A{index + 1:02d}"))
                target = str(link.get("target", f"A{index + 2:02d}"))
                link_id = f"{source}-{target}"
            links.append(
                LinkState(
                    link_id=link_id,
                    direction=str(link.get("direction", "duplex")),
                    latency_ms=float(link.get("latency_ms", 18.0 + index * 5.0)),
                    loss_rate=float(link.get("loss_rate", 0.01)),
                    status=str(link.get("status", "normal")),
                )
            )
        self._base_links = list(links)
        self._links = list(links)
        self._inbox = {}
        self._pending = []

    def read_inbox(self, node_id: str) -> list[MessageEnvelope]:
        messages = self._inbox.get(node_id, [])
        self._inbox[node_id] = []
        return messages

    def send(self, messages: list[MessageEnvelope]) -> None:
        self._pending.extend(messages)

    def tick(self, dt_s: float) -> None:
        self._time_s += dt_s
        for message in self._pending:
            self._inbox.setdefault(message.target, []).append(message)
        self._pending = []
        self._refresh_links()

    def _refresh_links(self) -> None:
        degraded = self._time_s < self._loss_until_s
        self._links = [
            LinkState(
                link_id=link.link_id,
                direction=link.direction,
                latency_ms=link.latency_ms + (self._active_latency_delta_ms if degraded else 0.0),
                loss_rate=(
                    self._active_loss_rate
                    if degraded
                    else max(0.0, min(1.0, link.loss_rate))
                ),
                status="degraded" if degraded else "normal",
            )
            for link in self._base_links
        ]

    def read_link_states(self) -> list[LinkState]:
        return list(self._links)

    def inject_link_fault(self, command: DisturbanceCommand, until_s: float) -> None:
        self._loss_until_s = until_s
        self._active_loss_rate = max(0.0, min(1.0, float(command.params.get("loss_rate", 0.25))))
        self._active_latency_delta_ms = float(command.params.get("latency_delta_ms", 40.0))
        self._refresh_links()

    def inject_link_qos(self, command: DisturbanceCommand, until_s: float) -> None:
        self.inject_link_fault(command, until_s)

    def reset(self) -> None:
        self._time_s = 0.0
        self.clear_faults()
        self._inbox = {}
        self._pending = []

    def clear_faults(self) -> None:
        self._loss_until_s = 0.0
        self._active_loss_rate = 0.25
        self._active_latency_delta_ms = 40.0
        self._refresh_links()

    def close(self) -> None:
        self._links = []
        self._base_links = []
        self._inbox = {}
        self._pending = []


class _NodeAlgorithm:
    """Simple per-node formation algorithm stub."""

    _VELOCITY_GAIN = 1.2

    def __init__(self, node_id: str, trim_velocity_mps: tuple[float, float, float]) -> None:
        self._node_id = node_id
        self._trim_velocity_mps = trim_velocity_mps

    def step(
        self,
        state: AircraftState,
        inbox: list[MessageEnvelope],
        time_s: float,
        health: str = "normal",
    ) -> _NodeAlgorithmOutput:
        del inbox
        trim_speed = sum(value * value for value in self._trim_velocity_mps) ** 0.5
        target_scale = 1.0
        if health != "normal" and trim_speed > 0.0:
            target_scale = min(1.0, 3.0 / trim_speed)
        target_velocity = tuple(
            target_scale * value
            for value in self._trim_velocity_mps
        )
        control = AccelerationCommand(
            self._VELOCITY_GAIN * (target_velocity[0] - state.vx_mps),
            self._VELOCITY_GAIN * (target_velocity[1] - state.vy_mps),
            self._VELOCITY_GAIN * (target_velocity[2] - state.vz_mps),
        )
        outbox = [
            MessageEnvelope(
                topic="node.status",
                source=self._node_id,
                target="broadcast",
                timestamp=time_s,
                payload={"health": health},
            )
        ]
        status = "reconfiguring" if health != "normal" else "forming"
        return _NodeAlgorithmOutput(control, outbox, status)

    def reset(self) -> None:
        return None

    def close(self) -> None:
        return None


class _DisturbanceEngine:
    """Dynamic disturbance stub."""

    def __init__(self) -> None:
        self._active: list[tuple[DisturbanceCommand, float]] = []
        self._model: ModelIterator | None = None
        self._comm: _CommunicationEngine | None = None
        self._node_health: dict[str, str] = {}
        self._baseline_health: dict[str, str] = {}

    def init(
        self,
        config: dict[str, object],
        seed: int,
        model: ModelIterator,
        comm: _CommunicationEngine,
    ) -> None:
        del seed
        self._active = []
        self._model = model
        self._comm = comm
        nodes = config.get("nodes") or []
        self._baseline_health = {
            node_id_from_config(node, i): str(node.get("health", "normal"))
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        self._node_health = dict(self._baseline_health)

    def read_health(self) -> dict[str, str]:
        return dict(self._node_health)

    def inject(self, command: DisturbanceCommand, current_time_s: float) -> SimulationEvent:
        if command.type == "clear":
            self.clear()
            return SimulationEvent(current_time_s, "INFO", "Disturbance", "清除扰动")
        until_s = current_time_s + float(command.duration_s or 0.0)
        self._active.append((command, until_s))
        self._apply(command, until_s)
        return SimulationEvent(current_time_s, "INFO", "Disturbance", f"注入扰动: {command.type}")

    def tick(self, time_s: float, dt_s: float) -> list[SimulationEvent]:
        del dt_s
        events: list[SimulationEvent] = []
        remaining: list[tuple[DisturbanceCommand, float]] = []
        had_expiry = False
        for command, until_s in self._active:
            if time_s > until_s:
                events.append(SimulationEvent(time_s, "INFO", "Disturbance", f"扰动结束: {command.type}"))
                had_expiry = True
                continue
            remaining.append((command, until_s))
        self._active = remaining
        if had_expiry:
            self._clear_dynamic_effects()
            for command, until_s in self._active:
                self._apply(command, until_s)
        return events

    def clear(self) -> None:
        self._active = []
        self._clear_dynamic_effects()

    def _apply(self, command: DisturbanceCommand, until_s: float) -> None:
        if command.type == "wind" and self._model is not None:
            self._model.inject_wind(command)
        elif command.type == "node_fault":
            target = str(ModelIterator._command_value(command, "target") or "")
            if target in self._node_health:
                params = ModelIterator._command_params(command)
                self._node_health[target] = str(params.get("mode", "degraded"))
        elif command.type in {"link_loss", "link_fault"} and self._comm is not None:
            self._comm.inject_link_fault(command, until_s)

    def _clear_dynamic_effects(self) -> None:
        if self._model is not None:
            self._model.clear_wind()
        self._node_health = dict(self._baseline_health)
        if self._comm is not None:
            self._comm.clear_faults()

    def reset(self) -> None:
        self.clear()

    def close(self) -> None:
        self._active = []
        self._model = None
        self._comm = None
        self._node_health.clear()
        self._baseline_health.clear()


class _DataLogger:
    """In-memory logger stub."""

    def __init__(self) -> None:
        self.snapshots: list[SimulationSnapshot] = []
        self.events: list[SimulationEvent] = []
        self.opened = False

    def open(self, run_id: str, config: dict[str, object]) -> None:
        del run_id, config
        self.opened = True

    def write_snapshot(self, snapshot: SimulationSnapshot) -> None:
        self.snapshots.append(snapshot)

    def write_event(self, event: SimulationEvent) -> None:
        self.events.append(event)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.opened = False


class SimulationController:
    """Top-level simulation orchestration facade."""

    _EVENT_BUFFER_SIZE = 1000
    _DISPLAY_REFRESH_S = 0.1

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._config_loader = _ConfigLoader()
        self._model = ModelIterator()
        self._comm = _CommunicationEngine()
        self._disturbance = _DisturbanceEngine()
        self._logger = _DataLogger()
        self._node_algorithms: dict[str, _NodeAlgorithm] = {}
        self._node_roles: dict[str, str] = {}
        self._current_controls: dict[str, AccelerationCommand] = {}
        self._config: dict[str, object] | None = None
        self._seed = 0
        self._duration_s = 0.0
        self._step_s = 0.005
        self._time_s = 0.0
        self._tick_index = 0
        self._playback_rate = 1.0
        self._run_state: RunState = "UNLOADED"
        self._control_report: ControlReport = "待命"
        self._latest_snapshot = self._make_snapshot_for_empty_controller()
        self._events: deque[SimulationEvent] = deque(maxlen=self._EVENT_BUFFER_SIZE)
        self._subscribers: dict[int, Callable[[SimulationSnapshot], None]] = {}
        self._subscriber_ids_by_callback: dict[Callable[[SimulationSnapshot], None], int] = {}
        self._next_subscription_id = 1
        self._last_display_wall_s = 0.0
        self._worker: threading.Thread | None = None
        self._stop_requested = threading.Event()
        self._closed = False

    def load_config(self, path: str) -> CommandResult:
        """Load, validate, and initialize a simulation configuration."""

        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
        try:
            config = self._config_loader.load(path)
        except FileNotFoundError:
            return CommandResult("ERR_CONFIG_NOT_FOUND", f"config not found: {path}")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return CommandResult("ERR_CONFIG_INVALID", str(exc))

        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001 - first version maps module init failures.
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", f"配置已加载: {path}")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "config loaded")

    def get_snapshot(self) -> SimulationSnapshot:
        """Return the latest complete snapshot without advancing simulation time."""

        with self._lock:
            return self._latest_snapshot

    def start(self) -> CommandResult:
        """Start or continue scheduled simulation execution."""

        should_stop_worker = False
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            should_stop_worker = self._worker is not None and self._worker.is_alive()

        if should_stop_worker:
            self._stop_worker()

        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            self._run_state = "RUNNING"
            self._control_report = "集结"
            self._stop_requested.clear()
            self._start_worker_unlocked()
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "started")

    def pause(self) -> CommandResult:
        """Pause scheduled execution."""

        with self._lock:
            if self._run_state == "RUNNING":
                self._run_state = "PAUSED"
                self._control_report = "保持"
                self._latest_snapshot = self._make_snapshot_unlocked()
                snapshot = self._latest_snapshot
            elif self._run_state == "PAUSED":
                return CommandResult("OK", "already paused")
            else:
                return CommandResult("ERR_INVALID_STATE", "pause requires RUNNING or PAUSED")
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "paused")

    def step(self, count: int = 1) -> CommandResult:
        """Advance ``count`` base ticks in READY/PAUSED state."""

        if count < 1:
            return CommandResult("ERR_INVALID_ARGUMENT", "count must be >= 1")
        snapshots_to_notify: list[SimulationSnapshot] = []
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before step")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_INVALID_STATE", "pause before manual step")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before stepping")
            self._run_state = "PAUSED"
            self._control_report = "保持"
            for _ in range(count):
                try:
                    snapshot = self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
                if snapshot is not None:
                    snapshots_to_notify.append(snapshot)
                if self._run_state == "FINISHED":
                    break
            if not snapshots_to_notify:
                snapshots_to_notify.append(self._latest_snapshot)
        for snapshot in snapshots_to_notify:
            self._notify_subscribers(snapshot)
        return CommandResult("OK", "stepped")

    def reset(self) -> CommandResult:
        """Reset current config and return to READY."""

        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before reset")
            config = dict(self._config)
        self._stop_worker()
        with self._lock:
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "仿真已重置")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "reset")

    def close(self) -> None:
        """Release resources. The controller instance must not be reused."""

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
            self._closed = True

    def set_playback_rate(self, rate: float) -> CommandResult:
        """Set wall-clock playback rate without changing simulation step size."""

        if not 0.1 <= rate <= 10.0:
            return CommandResult("ERR_INVALID_ARGUMENT", "rate must be in [0.1, 10.0]")
        with self._lock:
            self._playback_rate = float(rate)
        return CommandResult("OK", "playback rate updated")

    def inject_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> CommandResult:
        """Inject a dynamic disturbance command."""

        try:
            normalized = self._normalize_disturbance(command)
        except (TypeError, ValueError) as exc:
            return CommandResult("ERR_INVALID_ARGUMENT", str(exc))
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before disturbance")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "disturbance is not accepted after finish")
            event = self._disturbance.inject(normalized, self._time_s)
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
            self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "disturbance injected")

    def subscribe_snapshot(self, callback: Callable[[SimulationSnapshot], None]) -> Subscription:
        """Subscribe to display refresh snapshots."""

        with self._lock:
            subscription_id = self._subscriber_ids_by_callback.get(callback)
            if subscription_id is None:
                subscription_id = self._next_subscription_id
                self._next_subscription_id += 1
                self._subscribers[subscription_id] = callback
                self._subscriber_ids_by_callback[callback] = subscription_id
            snapshot = self._latest_snapshot

        def unsubscribe() -> None:
            with self._lock:
                removed = self._subscribers.pop(subscription_id, None)
                if removed is not None:
                    self._subscriber_ids_by_callback.pop(removed, None)

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
        """Return recent in-memory events."""

        if limit < 1:
            return []
        level_order = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40}
        min_value = level_order.get(min_level or "DEBUG", 10)
        with self._lock:
            events = [event for event in self._events if level_order[event.level] >= min_value]
            return events[-limit:]

    def run_until_complete(self, config: object | str, *, seed: int | None = None) -> CommandResult:
        """Run synchronously until FINISHED for CLI/batch usage."""

        if isinstance(config, str):
            result = self.load_config(config)
            if result.code != "OK":
                return result
        elif isinstance(config, dict):
            with self._lock:
                config_copy = dict(config)
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

        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before run")
            self._run_state = "RUNNING"
            self._control_report = "集结"
            while self._run_state == "RUNNING":
                try:
                    self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
        return CommandResult("OK", "finished")

    def _run_loop(self) -> None:
        current = threading.current_thread()
        try:
            while not self._stop_requested.is_set():
                start_wall_s = time.monotonic()
                with self._lock:
                    if self._run_state != "RUNNING":
                        break
                    try:
                        snapshot = self._tick_unlocked()
                    except Exception as exc:  # noqa: BLE001
                        self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                        self._run_state = "PAUSED"
                        snapshot = self._make_snapshot_unlocked()
                if snapshot is not None:
                    self._notify_subscribers(snapshot)
                with self._lock:
                    interval_s = self._step_s / self._playback_rate
                elapsed_s = time.monotonic() - start_wall_s
                time.sleep(max(0.0, interval_s - elapsed_s))
        finally:
            with self._lock:
                if self._worker is current:
                    self._worker = None

    def _stop_worker(self) -> None:
        self._stop_requested.set()
        worker = self._worker
        if worker is not None and worker.is_alive() and worker is not threading.current_thread():
            worker.join(timeout=2.0)
        self._worker = None
        self._stop_requested.clear()

    def _start_worker_unlocked(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        self._worker = threading.Thread(target=self._run_loop, name="SimulationController", daemon=True)
        self._worker.start()

    def _init_modules_unlocked(self, config: dict[str, object]) -> None:
        self._config = dict(config)
        self._seed = int(config.get("seed", 0))
        self._duration_s = float(config.get("duration_s", 120.0))
        self._step_s = float(config.get("step_s", 0.005))
        self._playback_rate = float(config.get("playback_rate", 1.0))
        self._time_s = 0.0
        self._tick_index = 0
        self._last_display_wall_s = 0.0
        self._model.init(config, self._seed)
        self._comm.init(config, dict(config.get("qos", {})) if isinstance(config.get("qos"), dict) else {}, self._seed)
        self._disturbance.init(config, self._seed, self._model, self._comm)
        nodes = config.get("nodes") or []
        self._node_roles = {
            node_id_from_config(node, i): str(
                node.get("role") or ("leader" if i == 0 else "wingman")
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        self._node_algorithms = {
            node_id: _NodeAlgorithm(
                node_id,
                (state.vx_mps, state.vy_mps, state.vz_mps),
            )
            for node_id, state in self._model.read_states().items()
        }
        self._current_controls = {
            node_id: AccelerationCommand()
            for node_id in self._model.read_states()
        }
        self._logger.open(f"run-{int(time.time())}", config)

    def _tick_unlocked(self, *, force_snapshot: bool = False) -> SimulationSnapshot | None:
        if self._run_state not in {"RUNNING", "PAUSED"}:
            return self._latest_snapshot
        step_s = self._step_s
        tick_index = self._tick_index

        if tick_index % 10 == 0:
            self._run_formation_algorithms_unlocked()
        if tick_index % 2 == 0:
            self._comm.tick(step_s * 2.0)

        self._model.apply_controls(self._current_controls)
        for event in self._disturbance.tick(self._time_s, step_s):
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
        self._model.step(step_s)
        self._time_s = min(self._duration_s, self._time_s + step_s)
        self._tick_index += 1

        if self._time_s >= self._duration_s:
            self._run_state = "FINISHED"
            self._control_report = "保持"
        elif self._run_state == "RUNNING":
            self._control_report = self._derive_control_report_unlocked()

        snapshot: SimulationSnapshot | None = None
        if force_snapshot or tick_index % 2 == 0 or self._run_state == "FINISHED":
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        if tick_index % 10 == 0 and snapshot is not None:
            self._logger.write_snapshot(snapshot)
        if self._should_refresh_display_unlocked() or self._run_state == "FINISHED":
            return self._latest_snapshot
        return None

    def _run_formation_algorithms_unlocked(self) -> None:
        states = self._model.read_states()
        health_map = self._disturbance.read_health()
        controls: dict[str, AccelerationCommand] = {}
        outbox: list[MessageEnvelope] = []
        status_values: list[str] = []
        for node_id, state in states.items():
            inbox = self._comm.read_inbox(node_id)
            output = self._node_algorithms[node_id].step(
                state, inbox, self._time_s, health_map.get(node_id, "normal")
            )
            controls[node_id] = output.control
            outbox.extend(output.outbox)
            status_values.append(output.status)
        self._current_controls = controls
        self._model.apply_controls(controls)
        routed = [
            message
            for message in outbox
            if message.target != "broadcast"
        ]
        self._comm.send(routed)
        if any(status != "forming" for status in status_values):
            self._control_report = "重构"

    def _make_snapshot_unlocked(self) -> SimulationSnapshot:
        health_map = self._disturbance.read_health()
        nodes = [
            NodeState(
                node_id=state.node_id,
                role=self._node_roles.get(state.node_id, "unknown"),
                health=health_map.get(state.node_id, "normal"),
                x_m=state.x_m,
                y_m=state.y_m,
                altitude_m=state.altitude_m,
                psi_v_deg=state.psi_v_deg,
                theta_deg=state.theta_deg,
                speed_mps=state.speed_mps,
                vx_mps=state.vx_mps,
                vy_mps=state.vy_mps,
                vz_mps=state.vz_mps,
                nx=state.nx,
                nz=state.nz,
                phi_deg=state.phi_deg,
            )
            for state in self._model.read_states().values()
        ]
        return SimulationSnapshot(
            time_s=self._time_s,
            duration_s=self._duration_s,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=nodes,
            links=self._comm.read_link_states(),
        )

    def _make_snapshot_for_empty_controller(self) -> SimulationSnapshot:
        return SimulationSnapshot(
            time_s=0.0,
            duration_s=0.0,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=[],
            links=[],
        )

    def _derive_control_report_unlocked(self) -> ControlReport:
        if any(h != "normal" for h in self._disturbance.read_health().values()):
            return "重构"
        return "集结"

    def _should_refresh_display_unlocked(self) -> bool:
        now_s = time.monotonic()
        if self._last_display_wall_s == 0.0 or now_s - self._last_display_wall_s >= self._DISPLAY_REFRESH_S:
            self._last_display_wall_s = now_s
            return True
        return False

    def _notify_subscribers(self, snapshot: SimulationSnapshot) -> None:
        with self._lock:
            subscribers = list(self._subscribers.values())
        for callback in subscribers:
            try:
                callback(snapshot)
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._append_event_unlocked("WARN", "SimControl", f"snapshot callback failed: {exc}")

    def _append_event_unlocked(self, level: EventLevel, source: str, message: str) -> None:
        event = SimulationEvent(self._time_s, level, source, message)
        self._append_event_object_unlocked(event)
        self._logger.write_event(event)

    def _append_event_object_unlocked(self, event: SimulationEvent) -> None:
        self._events.append(event)

    def _normalize_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> DisturbanceCommand:
        if isinstance(command, DisturbanceCommand):
            return command
        if not isinstance(command, dict):
            raise TypeError("command must be DisturbanceCommand or dict")
        command_type = command.get("type")
        if command_type not in {"wind", "node_fault", "link_loss", "link_fault", "clear"}:
            raise ValueError("invalid disturbance type")
        params = command.get("params", {})
        if not isinstance(params, dict):
            raise ValueError("params must be a dict")
        duration = command.get("duration_s")
        return DisturbanceCommand(
            type=command_type,  # type: ignore[arg-type]
            target=str(command["target"]) if command.get("target") is not None else None,
            duration_s=float(duration) if duration is not None else None,
            params=dict(params),
        )
