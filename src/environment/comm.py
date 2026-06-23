"""通信链路仿真模块。注意：负责拓扑、延迟、丢包和故障。"""

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
    """校验通信链路 QoS 配置。注意：非法配置应在初始化阶段尽早暴露。"""
    if latency_ms is not None:
        if not math.isfinite(latency_ms) or latency_ms < 0:
            raise ValueError(f"latency_ms must be a finite number >= 0, got {latency_ms}")
    if loss_rate is not None:
        if not math.isfinite(loss_rate) or not (0.0 <= loss_rate <= 1.0):
            raise ValueError(f"loss_rate must be a finite number in [0.0, 1.0], got {loss_rate}")


def _req_str(d: dict, key: str, where: str) -> str:
    """读取必填字符串字段。注意：缺失或类型错误会抛出配置异常。"""
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    if not isinstance(v, str):
        raise ValueError(f"{where}: {key!r} must be str, got {type(v).__name__!r}")
    return v


def _req_num(d: dict, key: str, where: str) -> float:
    """读取必填数值字段。注意：布尔值不视为合法数值。"""
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{where}: {key!r} must be a number, got {type(v).__name__!r}")
    return float(v)


def _check_finite_num(v: object, name: str) -> float:
    """校验数值是有限实数并转换为浮点数。注意：NaN 和无穷大不允许进入仿真。"""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name} must be a number, got {type(v).__name__!r}")
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be a finite number, got {v!r}")
    return f


class CommunicationChannel:
    """按拓扑和 QoS 配置路由消息。注意：链路状态会影响消息投递。"""

    def __init__(self) -> None:
        """初始化 CommunicationChannel 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
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
        """按配置初始化 CommunicationChannel。注意：调用方需先准备好必要依赖和输入数据。"""
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
        """复位 CommunicationChannel 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._in_flight.clear()
        for lst in self._inbox.values():
            lst.clear()
        self._links = copy.deepcopy(self._base_links)
        self._rng = np.random.default_rng(self._seed)
        self._time_s = 0.0

    def close(self) -> None:
        """释放 CommunicationChannel 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self._links.clear()
        self._link_index.clear()
        self._in_flight.clear()
        for lst in self._inbox.values():
            lst.clear()

    def update_topology(self, config: dict) -> None:
        # 阶段 1：先校验全部链路并检查重复项，此阶段不修改状态。
        """更新通信拓扑和链路配置。注意：运行中更新会重建链路状态和收件箱。"""
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
        # 阶段 2：所有检查通过后再原子应用配置。
        for keys, latency_ms, loss_rate in pending:
            for key in keys:
                self._links[key].latency_ms = latency_ms
                self._links[key].loss_rate = loss_rate

    def send(self, messages: list[MessageEnvelope]) -> None:
        """发送一批通信消息。注意：链路丢失或丢包时消息可能被丢弃。"""
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
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
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
        """读取指定节点的收件箱消息。注意：读取后会清空该节点当前缓存。"""
        if node_id not in self._inbox:
            raise KeyError(node_id)
        messages = self._inbox[node_id]
        self._inbox[node_id] = []
        return messages

    def read_link_states(self) -> list[LinkState]:
        """读取全部方向链路状态。注意：返回的是快照副本，供显示和控制器折叠使用。"""
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
        """注入链路故障或恢复链路。注意：目标链路会按单向链路 ID 解析。"""
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
        """注入链路 QoS 变化。注意：延迟和丢包率会覆盖该方向链路的运行参数。"""
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
        """解析链路 ID 对应的源节点和目标节点。注意：只支持当前链路命名约定。"""
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
