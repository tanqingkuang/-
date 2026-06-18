"""Communication channel simulation."""

import copy
import dataclasses
import math

import numpy as np

from src.common.envelope import MessageEnvelope


@dataclasses.dataclass
class _LinkConfig:
    canonical_link_id: str
    direction: str
    latency_ms: float
    loss_rate: float
    status: str = "normal"
    fault_until_s: float | None = None


@dataclasses.dataclass
class _InFlightMessage:
    envelope: MessageEnvelope
    remaining_s: float


@dataclasses.dataclass(frozen=True)
class LinkState:
    link_id: str
    latency_ms: float
    loss_rate: float
    status: str


def _validate_qos(latency_ms: float | None, loss_rate: float | None) -> None:
    if latency_ms is not None:
        if not math.isfinite(latency_ms) or latency_ms < 0:
            raise ValueError(f"latency_ms must be a finite number >= 0, got {latency_ms}")
    if loss_rate is not None:
        if not math.isfinite(loss_rate) or not (0.0 <= loss_rate <= 1.0):
            raise ValueError(f"loss_rate must be a finite number in [0.0, 1.0], got {loss_rate}")


def _req_str(d: dict, key: str, where: str) -> str:
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    if not isinstance(v, str):
        raise ValueError(f"{where}: {key!r} must be str, got {type(v).__name__!r}")
    return v


def _req_num(d: dict, key: str, where: str) -> float:
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{where}: {key!r} must be a number, got {type(v).__name__!r}")
    return float(v)


def _check_finite_num(v: object, name: str) -> float:
    """Validate v is a non-bool, finite numeric value; return as float."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name} must be a number, got {type(v).__name__!r}")
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be a finite number, got {v!r}")
    return f


class CommunicationChannel:
    """Route messages by topology and QoS configuration."""

    def __init__(self) -> None:
        self._nodes: list[str] = []
        self._links: dict[tuple[str, str], _LinkConfig] = {}
        self._link_index: dict[str, list[tuple[str, str]]] = {}
        self._inbox: dict[str, list[MessageEnvelope]] = {}
        self._in_flight: dict[tuple[str, str], list[_InFlightMessage]] = {}
        self._rng: np.random.Generator = np.random.default_rng(0)
        self._seed: int = 0
        self._time_s: float = 0.0
        self._base_links: dict[tuple[str, str], _LinkConfig] = {}

    def init(self, config: dict, seed: int) -> None:
        nodes_raw = config.get("nodes")
        if not isinstance(nodes_raw, list):
            raise ValueError("config: 'nodes' must be a list")

        node_ids: list[str] = []
        seen: set[str] = set()
        for i, node in enumerate(nodes_raw):
            where = f"nodes[{i}]"
            if not isinstance(node, dict):
                raise ValueError(f"{where}: each node must be a dict, got {type(node).__name__!r}")
            nid: str = _req_str(node, "node_id", where)
            if not nid:
                raise ValueError("node_id must be non-empty")
            if "-" in nid:
                raise ValueError(f"node_id must not contain '-': {nid!r}")
            if nid == "broadcast":
                raise ValueError("'broadcast' is a reserved node_id")
            if nid in seen:
                raise ValueError(f"duplicate node_id: {nid!r}")
            seen.add(nid)
            node_ids.append(nid)

        links_raw = config.get("links", [])
        if not isinstance(links_raw, list):
            raise ValueError("config: 'links' must be a list")

        links: dict[tuple[str, str], _LinkConfig] = {}
        link_index: dict[str, list[tuple[str, str]]] = {}
        for i, link in enumerate(links_raw):
            where = f"links[{i}]"
            if not isinstance(link, dict):
                raise ValueError(f"{where}: each link must be a dict, got {type(link).__name__!r}")
            link_id: str = _req_str(link, "link_id", where)
            if link_id.count("-") != 1:
                raise ValueError(f"link_id must contain exactly one '-': {link_id!r}")
            a, b = link_id.split("-")
            if a not in seen or b not in seen:
                raise ValueError(f"link_id {link_id!r} references unknown node")
            if a == b:
                raise ValueError(f"self-loop link is not allowed: {link_id!r}")
            latency_ms: float = _req_num(link, "latency_ms", where)
            loss_rate: float = _req_num(link, "loss_rate", where)
            _validate_qos(latency_ms, loss_rate)
            raw_direction = link.get("direction", "duplex")
            if not isinstance(raw_direction, str):
                raise ValueError(f"invalid link direction {raw_direction!r}: {link_id!r}")
            if raw_direction not in ("duplex", "simplex"):
                raise ValueError(f"invalid link direction {raw_direction!r}: {link_id!r}")
            raw_status: str = link.get("status", "normal")
            if raw_status not in ("normal", "lost"):
                raise ValueError(f"invalid link status {raw_status!r}: {link_id!r}")
            keys = [(a, b), (b, a)] if raw_direction == "duplex" else [(a, b)]
            if any(key in links for key in keys):
                raise ValueError(f"duplicate link: {link_id!r}")
            for key in keys:
                links[key] = _LinkConfig(link_id, raw_direction, latency_ms, loss_rate, raw_status)
            link_index[link_id] = keys

        self._nodes: list[str] = node_ids
        self._links: dict[tuple[str, str], _LinkConfig] = links
        self._link_index: dict[str, list[tuple[str, str]]] = link_index
        self._inbox: dict[str, list[MessageEnvelope]] = {nid: [] for nid in node_ids}
        self._in_flight: dict[tuple[str, str], list[_InFlightMessage]] = {}
        self._rng: np.random.Generator = np.random.default_rng(seed)
        self._seed: int = seed
        self._time_s: float = 0.0
        self._base_links: dict[tuple[str, str], _LinkConfig] = copy.deepcopy(links)

    def reset(self) -> None:
        self._in_flight.clear()
        for lst in self._inbox.values():
            lst.clear()
        self._links = copy.deepcopy(self._base_links)
        self._rng = np.random.default_rng(self._seed)
        self._time_s = 0.0

    def close(self) -> None:
        self._links.clear()
        self._link_index.clear()
        self._in_flight.clear()
        for lst in self._inbox.values():
            lst.clear()

    def update_topology(self, config: dict) -> None:
        # Phase 1: validate all links and detect duplicates — no mutations yet.
        links_raw = config.get("links", [])
        if not isinstance(links_raw, list):
            raise ValueError("config: 'links' must be a list")
        pending: list[tuple[list[tuple[str, str]], float, float]] = []
        seen_keys: set[tuple[str, str]] = set()
        for i, link in enumerate(links_raw):
            where = f"links[{i}]"
            if not isinstance(link, dict):
                raise ValueError(f"{where}: each link must be a dict, got {type(link).__name__!r}")
            link_id: str = _req_str(link, "link_id", where)
            latency_ms: float = _req_num(link, "latency_ms", where)
            loss_rate: float = _req_num(link, "loss_rate", where)
            _validate_qos(latency_ms, loss_rate)
            keys = self._resolve_pair(link_id, key_error=False)
            if any(key in seen_keys for key in keys):
                raise ValueError(f"duplicate link in update_topology: {link_id!r}")
            seen_keys.update(keys)
            pending.append((keys, latency_ms, loss_rate))
        # Phase 2: apply atomically — only reached when all checks pass.
        for keys, latency_ms, loss_rate in pending:
            for key in keys:
                self._links[key].latency_ms = latency_ms
                self._links[key].loss_rate = loss_rate

    def send(self, messages: list[MessageEnvelope]) -> None:
        for msg in messages:
            if msg.source not in self._inbox:
                continue

            if msg.target == "broadcast":
                targets = [nid for nid in self._nodes if nid != msg.source]
            elif isinstance(msg.target, list):
                targets = list(dict.fromkeys(msg.target))
            else:
                targets = [msg.target]

            targets = [
                dst for dst in targets
                if dst in self._inbox and dst != msg.source
            ]

            for dst in targets:
                key = (msg.source, dst)
                cfg = self._links.get(key)
                if cfg is None:
                    continue
                if cfg.status == "lost":
                    continue
                if self._rng.random() < cfg.loss_rate:
                    continue
                delivered = dataclasses.replace(msg, target=dst)
                delay_s = cfg.latency_ms / 1000.0
                self._in_flight.setdefault(key, []).append(
                    _InFlightMessage(delivered, delay_s)
                )

    def tick(self, dt_s: float) -> None:
        dt_s = _check_finite_num(dt_s, "dt_s")
        if dt_s <= 0:
            raise ValueError(f"dt_s must be > 0, got {dt_s}")

        self._time_s += dt_s

        for cfg in self._links.values():
            if cfg.fault_until_s is not None and self._time_s >= cfg.fault_until_s:
                cfg.status = "normal"
                cfg.fault_until_s = None

        for key, queue in list(self._in_flight.items()):
            remaining: list[_InFlightMessage] = []
            for item in queue:
                item.remaining_s -= dt_s
                if item.remaining_s <= 0:
                    self._inbox[key[1]].append(item.envelope)
                else:
                    remaining.append(item)
            if remaining:
                self._in_flight[key] = remaining
            else:
                del self._in_flight[key]

    def read_inbox(self, node_id: str) -> list[MessageEnvelope]:
        if node_id not in self._inbox:
            raise KeyError(node_id)
        messages = self._inbox[node_id]
        self._inbox[node_id] = []
        return messages

    def read_link_states(self) -> list[LinkState]:
        states = [
            LinkState(
                link_id=f"{src}-{dst}",
                latency_ms=cfg.latency_ms,
                loss_rate=cfg.loss_rate,
                status=cfg.status,
            )
            for (src, dst), cfg in self._links.items()
        ]
        states.sort(key=lambda s: s.link_id)
        return states

    def inject_link_fault(
        self, link_id: str, status: str, duration_s: float | None = None
    ) -> None:
        if status not in ("normal", "lost"):
            raise ValueError(f"invalid status {status!r}; must be 'normal' or 'lost'")
        if status == "normal" and duration_s is not None:
            raise ValueError("duration_s must be None when status='normal'")
        if duration_s is not None:
            duration_s = _check_finite_num(duration_s, "duration_s")
            if duration_s <= 0:
                raise ValueError(f"duration_s must be > 0, got {duration_s}")

        fault_until = (self._time_s + duration_s) if duration_s is not None else None
        for key in self._resolve_pair(link_id, key_error=True):
            cfg = self._links[key]
            cfg.status = status
            cfg.fault_until_s = fault_until if status == "lost" else None

    def inject_link_qos(
        self,
        link_id: str,
        latency_ms: float | None,
        loss_rate: float | None,
    ) -> None:
        if latency_ms is None and loss_rate is None:
            return
        if latency_ms is not None:
            latency_ms = _check_finite_num(latency_ms, "latency_ms")
        if loss_rate is not None:
            loss_rate = _check_finite_num(loss_rate, "loss_rate")
        _validate_qos(latency_ms, loss_rate)
        for key in self._resolve_pair(link_id, key_error=True):
            cfg = self._links[key]
            if latency_ms is not None:
                cfg.latency_ms = latency_ms
            if loss_rate is not None:
                cfg.loss_rate = loss_rate

    def _resolve_pair(
        self, link_id: str, *, key_error: bool
    ) -> list[tuple[str, str]]:
        if link_id.count("-") != 1:
            raise ValueError(f"link_id must contain exactly one '-': {link_id!r}")
        a, b = link_id.split("-")
        key = (a, b)
        if key not in self._links:
            if key_error:
                raise KeyError(link_id)
            raise ValueError(f"unknown link_id: {link_id!r}")
        cfg = self._links[key]
        if cfg.direction == "duplex":
            return list(self._link_index[cfg.canonical_link_id])
        return [key]
