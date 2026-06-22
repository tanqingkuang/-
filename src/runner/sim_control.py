"""Simulation control facade.

The controller implements the application contract described in
``docs/1-仿真控制HLD.md``. The UAV model is provided by
``src.environment.model``; communication, algorithm, disturbance, and logging
remain first-pass local implementations.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, Literal

from src.algorithm.context.leaf_types import (
    CommDirE,
    FormCommInitS,
    FormPatE,
    FormPosS,
    FormSelfInitS,
    FormStageE,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
    RemoteCmdS,
    RouteS,
    VdInEarthS,
    WayLineS,
    WayPointS,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.follower import FollowerEntity
from src.algorithm.entity.leader_follower_hold.leader import LeaderEntity
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.common.envelope import MessageEnvelope
from src.environment.comm import CommunicationChannel
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
    cross_track_error_m: float | None = None
    distance_to_go_m: float | None = None


@dataclass(frozen=True)
class LinkState:
    """UI/CLI-facing state for one communication link."""

    link_id: str
    direction: str
    latency_ms: float
    loss_rate: float
    status: str


@dataclass(frozen=True)
class RouteState:
    """UI-facing reference route segment in ENU coordinates."""

    start_x_m: float
    start_y_m: float
    start_altitude_m: float
    end_x_m: float
    end_y_m: float
    end_altitude_m: float


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
    route: RouteState | None = None


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


@dataclass(frozen=True)
class _ConfiguredLink:
    link_id: str
    direction: str


_DEFAULT_TRIANGLE_WING_SLOTS: tuple[tuple[float, float, float], ...] = (
    (-54.0, 58.0, 0.0),
    (-54.0, -58.0, 0.0),
)


def _motion_from_aircraft_state(state: AircraftState) -> MotionProfS:
    ground_speed = (state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps) ** 0.5
    return MotionProfS(
        pos=PosInEarthS(state.x_m, state.y_m, state.altitude_m),
        vd=VdInEarthS(
            vEast=state.vx_mps,
            vNorth=state.vy_mps,
            vUp=state.vz_mps,
            vTheta=state.theta_rad,
            vPsi=state.psi_rad,
            vd=ground_speed,
        ),
    )


def _copy_motion(src: MotionProfS, dst: MotionProfS) -> None:
    dst.pos.east = src.pos.east
    dst.pos.north = src.pos.north
    dst.pos.h = src.pos.h
    dst.vd.vEast = src.vd.vEast
    dst.vd.vNorth = src.vd.vNorth
    dst.vd.vUp = src.vd.vUp
    dst.vd.vTheta = src.vd.vTheta
    dst.vd.vPsi = src.vd.vPsi
    dst.vd.vd = src.vd.vd


def _build_formation_comm_init(
    nodes: list[object],
    links: list[object],
    states: dict[str, AircraftState],
    config: dict[str, object] | None = None,
) -> FormCommInitS:
    network: list[NetWorkS] = []
    for link in links:
        if not isinstance(link, dict):
            continue
        link_id = str(link.get("link_id") or "")
        start_id, sep, end_id = link_id.partition("-")
        if not sep or not start_id or not end_id:
            continue
        direction = CommDirE.SIMPLEX if link.get("direction") == "simplex" else CommDirE.DUPLEX
        network.append(NetWorkS(start_id, end_id, direction))

    pattern, slots = _build_formation_slots(nodes, config)
    return FormCommInitS(
        netWork=network,
        formPat=[pattern],
        formPos=[slots],
    )


def _build_formation_slots(
    nodes: list[object],
    config: dict[str, object] | None,
) -> tuple[FormPatE, list[FormPosS]]:
    formation_config = (config or {}).get("formation")
    if formation_config is None:
        return FormPatE.TRIANGLE, _default_formation_slots(nodes)
    if not isinstance(formation_config, dict):
        raise ValueError("formation must be an object")

    pattern = _formation_pattern_from_config(formation_config.get("pattern", "TRIANGLE"))
    slot_config = formation_config.get("slots")
    if slot_config is None:
        return pattern, _default_formation_slots(nodes)
    if not isinstance(slot_config, list) or not slot_config:
        raise ValueError("formation.slots must be a non-empty list")

    known_node_ids = {
        node_id_from_config(node, index)
        for index, node in enumerate(nodes)
        if isinstance(node, dict)
    }
    slots_by_id: dict[str, FormPosS] = {}
    for index, slot in enumerate(slot_config):
        if not isinstance(slot, dict):
            raise ValueError(f"formation.slots[{index}] must be an object")
        node_id = str(slot.get("node_id", slot.get("id", "")))
        if not node_id:
            raise ValueError(f"formation.slots[{index}].node_id is required")
        if node_id in slots_by_id:
            raise ValueError(f"formation.slots contains duplicate node_id {node_id!r}")
        if known_node_ids and node_id not in known_node_ids:
            raise ValueError(f"formation.slots contains unknown node_id {node_id!r}")
        slots_by_id[node_id] = FormPosS(
            node_id,
            _float_from_keys(slot, "formation.slots", index, ("x_m", "x")),
            _float_from_keys(slot, "formation.slots", index, ("y_m", "y")),
            _float_from_keys(slot, "formation.slots", index, ("z_m", "z")),
        )

    missing = [node_id for node_id in known_node_ids if node_id not in slots_by_id]
    if missing:
        raise ValueError(f"formation.slots missing node_id {missing[0]!r}")

    ordered_slots: list[FormPosS] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        ordered_slots.append(slots_by_id[node_id_from_config(node, index)])
    return pattern, ordered_slots


def _default_formation_slots(nodes: list[object]) -> list[FormPosS]:
    leader_id = _leader_id_from_nodes(nodes)
    slots: list[FormPosS] = []
    wing_slot_index = 0
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        node_id = node_id_from_config(node, index)
        if node_id == leader_id:
            slots.append(FormPosS(node_id, 0.0, 0.0, 0.0))
        else:
            slot = _DEFAULT_TRIANGLE_WING_SLOTS[
                min(wing_slot_index, len(_DEFAULT_TRIANGLE_WING_SLOTS) - 1)
            ]
            slots.append(FormPosS(node_id, slot[0], slot[1], slot[2]))
            wing_slot_index += 1
    return slots


def _formation_pattern_from_config(raw_pattern: object) -> FormPatE:
    if isinstance(raw_pattern, str):
        try:
            return FormPatE[raw_pattern.strip().upper()]
        except KeyError as exc:
            raise ValueError(f"unknown formation.pattern {raw_pattern!r}") from exc
    try:
        return FormPatE(int(raw_pattern))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"unknown formation.pattern {raw_pattern!r}") from exc


def _float_from_keys(
    config: dict[str, object],
    prefix: str,
    index: int,
    keys: tuple[str, str],
) -> float:
    for key in keys:
        if key in config:
            return float(config[key])
    raise ValueError(f"{prefix}[{index}].{keys[0]} is required")


def _build_leader_route(config: dict[str, object] | None = None) -> RouteS:
    route_config = (config or {}).get("route")
    if route_config is None:
        return _default_leader_route()
    if not isinstance(route_config, dict):
        raise ValueError("route must be an object")

    segments = route_config.get("segments", route_config.get("lines"))
    if segments is None:
        return RouteS(lines=[_wayline_from_config(route_config, 0, "route")])
    if not isinstance(segments, list) or not segments:
        raise ValueError("route.segments must be a non-empty list")
    return RouteS(
        lines=[
            _wayline_from_config(segment, index, f"route.segments[{index}]", route_config)
            for index, segment in enumerate(segments)
        ]
    )


def _wayline_from_config(
    segment_config: object,
    index: int,
    field_name: str,
    route_defaults: dict[str, object] | None = None,
) -> WayLineS:
    if not isinstance(segment_config, dict):
        raise ValueError(f"{field_name} must be an object")
    defaults = route_defaults or {}
    speed = float(
        segment_config.get(
            "speed_mps",
            segment_config.get("vdCmd", defaults.get("speed_mps", defaults.get("vdCmd", 8.0))),
        )
    )
    radius = float(
        segment_config.get(
            "radius_m",
            segment_config.get("radius", defaults.get("radius_m", defaults.get("radius", 0.0))),
        )
    )
    if speed < 0.0:
        raise ValueError(f"{field_name}.speed_mps must be non-negative")
    if radius != 0.0:
        raise ValueError(f"{field_name}.radius_m must be 0 for straight route")
    start = _route_point_from_config(segment_config.get("start"), f"{field_name}.start")
    end = _route_point_from_config(segment_config.get("end"), f"{field_name}.end")
    if start.east == end.east and start.north == end.north and start.h == end.h:
        raise ValueError(f"{field_name} start and end must be different")
    return WayLineS(
        idx=index,
        start=WayPointS(idx=index, pos=start),
        end=WayPointS(idx=index + 1, pos=end),
        vdCmd=speed,
        radius=radius,
    )


def _default_leader_route() -> RouteS:
    return RouteS(
        lines=[
            WayLineS(
                idx=0,
                start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
                end=WayPointS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0)),
                vdCmd=8.0,
                radius=0.0,
            )
        ]
    )


def _route_point_from_config(raw: object, field_name: str) -> PosInEarthS:
    if not isinstance(raw, dict):
        raise ValueError(f"{field_name} must be an object")
    return PosInEarthS(
        float(raw.get("x_m", raw.get("east", 0.0))),
        float(raw.get("y_m", raw.get("north", 0.0))),
        float(raw.get("altitude_m", raw.get("h", 0.0))),
    )


def _leader_id_from_nodes(nodes: list[object]) -> str:
    for index, node in enumerate(nodes):
        if isinstance(node, dict) and str(node.get("role") or "") == "leader":
            return node_id_from_config(node, index)
    for index, node in enumerate(nodes):
        if isinstance(node, dict):
            return node_id_from_config(node, index)
    return ""


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
        _build_leader_route(config)
        _build_formation_comm_init(list(nodes or []), list(links or []), {}, config)
        ModelIterator._parse_model_config(model)



class _NodeAlgorithm:
    """Adapter from the portable formation entity API to SimulationController."""

    def __init__(
        self,
        node_id: str,
        role: str,
        comm_init: FormCommInitS,
        initial_leader_state: MotionProfS | None,
        leader_route: RouteS | None,
    ) -> None:
        self._node_id = node_id
        self._role = role
        self._leader_route = leader_route
        if role == "leader":
            self._entity: EntityBase = LeaderEntity()
        else:
            self._entity = FollowerEntity()
        self._entity.init(
            EntityInitS(
                selfInit=FormSelfInitS(node_id),
                commInit=comm_init,
                route=leader_route,
            )
        )
        if role != "leader" and initial_leader_state is not None and hasattr(self._entity, "cxt"):
            self._entity.cxt.cmd.stage = FormStageE.HOLD  # type: ignore[attr-defined]
            self._entity.cxt.cmd.pattern = FormPatE.TRIANGLE  # type: ignore[attr-defined]
            _copy_motion(initial_leader_state, self._entity.cxt.leaderState)  # type: ignore[attr-defined]

    def step(
        self,
        state: AircraftState,
        inbox: list[MessageEnvelope],
        time_s: float,
        health: str = "normal",
    ) -> _NodeAlgorithmOutput:
        entity_output = EntityOutputS()
        self._entity.step(
            EntityInputS(
                selfState=_motion_from_aircraft_state(state),
                inbox=inbox,
                remote=RemoteCmdS(FormStageE.HOLD),
            ),
            entity_output,
        )
        acc_cmd = entity_output.selfAccCmd or self._entity.cxt.selfAccCmd  # type: ignore[attr-defined]
        control = AccelerationCommand(
            acc_cmd.accEast,
            acc_cmd.accNorth,
            acc_cmd.accUp,
        )
        outbox = [
            replace(message, timestamp=time_s)
            for message in entity_output.outbox
        ]
        status = "reconfiguring" if health != "normal" else "forming"
        return _NodeAlgorithmOutput(control, outbox, status)

    def reset(self) -> None:
        return None

    def close(self) -> None:
        self._entity.close()

    def current_stage(self) -> FormStageE:
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None:
            return FormStageE.NONE
        return FormStageE(cxt.cmd.stage)

    def current_route(self) -> WayLineS | None:
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None or self._role != "leader":
            return None
        if (
            cxt.wayLine.end.pos.east == cxt.wayLine.start.pos.east
            and self._leader_route is not None
            and self._leader_route.lines
        ):
            return self._leader_route.lines[0]
        return cxt.wayLine


class _DisturbanceEngine:
    """Dynamic disturbance stub."""

    def __init__(self) -> None:
        self._active: list[tuple[DisturbanceCommand, float]] = []
        self._model: ModelIterator | None = None
        self._comm: CommunicationChannel | None = None
        self._node_health: dict[str, str] = {}
        self._baseline_health: dict[str, str] = {}
        self._faulted_links: set[str] = set()
        self._degraded_links: dict[str, float] = {}

    def init(
        self,
        config: dict[str, object],
        seed: int,
        model: ModelIterator,
        comm: CommunicationChannel,
    ) -> None:
        del seed
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
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
        elif command.type == "link_fault" and self._comm is not None:
            link_id = str(command.target or "")
            if link_id:
                try:
                    self._comm.inject_link_fault(link_id, "lost")
                    self._faulted_links.add(link_id)
                except (KeyError, ValueError):
                    pass
        elif command.type == "link_loss" and self._comm is not None:
            link_id = str(command.target or "")
            if link_id and link_id not in self._degraded_links:
                params = ModelIterator._command_params(command)
                rate_raw = params.get("loss_rate", 1.0)
                try:
                    rate = float(rate_raw) if isinstance(rate_raw, (int, float)) and not isinstance(rate_raw, bool) else 1.0
                    states = {s.link_id: s for s in self._comm.read_link_states()}
                    original = states[link_id].loss_rate if link_id in states else 0.0
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=rate)
                    self._degraded_links[link_id] = original
                except (KeyError, ValueError):
                    pass

    def _clear_dynamic_effects(self) -> None:
        if self._model is not None:
            self._model.clear_wind()
        self._node_health = dict(self._baseline_health)
        if self._comm is not None:
            for link_id in self._faulted_links:
                try:
                    self._comm.inject_link_fault(link_id, "normal")
                except (KeyError, ValueError):
                    pass
            for link_id, original_rate in self._degraded_links.items():
                try:
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=original_rate)
                except (KeyError, ValueError):
                    pass
        self._faulted_links = set()
        self._degraded_links = {}

    def reset(self) -> None:
        self.clear()

    def close(self) -> None:
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
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
        self._comm = CommunicationChannel()
        self._disturbance = _DisturbanceEngine()
        self._logger = _DataLogger()
        self._node_algorithms: dict[str, _NodeAlgorithm] = {}
        self._node_roles: dict[str, str] = {}
        self._configured_links: list[_ConfiguredLink] = []
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
            self._control_report = self._derive_control_report_unlocked()
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
            self._control_report = self._derive_control_report_unlocked()
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
        raw_links = list(config.get("links") or [])
        comm_config = {
            "nodes": list(config.get("nodes") or []),
            "links": raw_links,
        }
        self._comm.init(comm_config, self._seed)
        self._configured_links = self._parse_configured_links(raw_links)
        self._disturbance.init(config, self._seed, self._model, self._comm)
        nodes = config.get("nodes") or []
        self._node_roles = {
            node_id_from_config(node, i): str(
                node.get("role") or ("leader" if i == 0 else "wingman")
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        states = self._model.read_states()
        formation_comm_init = _build_formation_comm_init(list(nodes), raw_links, states, config)
        leader_id = _leader_id_from_nodes(list(nodes))
        initial_leader_state = states.get(leader_id)
        initial_leader_motion = (
            _motion_from_aircraft_state(initial_leader_state)
            if initial_leader_state is not None
            else None
        )
        leader_route = _build_leader_route(config)
        self._node_algorithms = {
            node_id: _NodeAlgorithm(
                node_id,
                self._node_roles.get(node_id, "wingman"),
                formation_comm_init,
                initial_leader_motion,
                leader_route,
            )
            for node_id in states
        }
        self._current_controls = {
            node_id: AccelerationCommand()
            for node_id in states
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
        self._comm.send(outbox)
        if any(status != "forming" for status in status_values):
            self._control_report = "重构"

    def _make_snapshot_unlocked(self) -> SimulationSnapshot:
        health_map = self._disturbance.read_health()
        route = self._make_route_snapshot()
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
                cross_track_error_m=self._cross_track_error(state, route),
                distance_to_go_m=self._distance_to_go(state, route),
            )
            for state in self._model.read_states().values()
        ]
        links = self._make_configured_link_snapshots()
        return SimulationSnapshot(
            time_s=self._time_s,
            duration_s=self._duration_s,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=nodes,
            links=links,
            route=route,
        )

    def _parse_configured_links(self, raw_links: list[object]) -> list[_ConfiguredLink]:
        configured: list[_ConfiguredLink] = []
        for link in raw_links:
            if not isinstance(link, dict) or not link.get("link_id"):
                continue
            configured.append(
                _ConfiguredLink(
                    link_id=str(link["link_id"]),
                    direction=str(link.get("direction") or "duplex"),
                )
            )
        return configured

    def _make_configured_link_snapshots(self) -> list[LinkState]:
        states = {state.link_id: state for state in self._comm.read_link_states()}
        links: list[LinkState] = []
        for configured in self._configured_links:
            ids = [configured.link_id]
            if configured.direction == "duplex":
                ids.append(self._reverse_link_id(configured.link_id))
            directional_states = [states[link_id] for link_id in ids if link_id in states]
            if not directional_states:
                continue
            status = "lost" if any(state.status == "lost" for state in directional_states) else directional_states[0].status
            links.append(
                LinkState(
                    link_id=configured.link_id,
                    direction=configured.direction,
                    latency_ms=max(state.latency_ms for state in directional_states),
                    loss_rate=max(state.loss_rate for state in directional_states),
                    status=status,
                )
            )
        return links

    def _make_route_snapshot(self) -> RouteState | None:
        for algorithm in self._node_algorithms.values():
            route = algorithm.current_route()
            if route is None:
                continue
            return RouteState(
                start_x_m=route.start.pos.east,
                start_y_m=route.start.pos.north,
                start_altitude_m=route.start.pos.h,
                end_x_m=route.end.pos.east,
                end_y_m=route.end.pos.north,
                end_altitude_m=route.end.pos.h,
            )
        return None

    @staticmethod
    def _cross_track_error(state: AircraftState, route: RouteState | None) -> float | None:
        if route is None:
            return None
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        normal_x = -dy / length
        normal_y = dx / length
        return (state.x_m - route.start_x_m) * normal_x + (state.y_m - route.start_y_m) * normal_y

    @staticmethod
    def _distance_to_go(state: AircraftState, route: RouteState | None) -> float | None:
        if route is None:
            return None
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        track_x = dx / length
        track_y = dy / length
        return max(0.0, (route.end_x_m - state.x_m) * track_x + (route.end_y_m - state.y_m) * track_y)

    @staticmethod
    def _reverse_link_id(link_id: str) -> str:
        src, sep, dst = link_id.partition("-")
        if not sep:
            return link_id
        return f"{dst}-{src}"

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
        stages = [
            algorithm.current_stage()
            for algorithm in self._node_algorithms.values()
        ]
        if any(stage == FormStageE.RECONFIG for stage in stages):
            return "重构"
        if any(stage == FormStageE.RALLY for stage in stages):
            return "集结"
        if any(stage == FormStageE.HOLD for stage in stages):
            return "保持"
        return "保持" if self._node_algorithms else "待命"

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
