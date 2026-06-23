"""通信链路仿真模块。注意：负责拓扑、延迟、丢包和故障。"""

import copy
import dataclasses
import math

import numpy as np

from src.common.envelope import MessageEnvelope


@dataclasses.dataclass
class _LinkConfig:
    """单条单向链路的运行态配置。注意：双工链路会拆成两条方向相反的 _LinkConfig 共享同一 canonical_link_id。"""

    # canonical_link_id 是配置里写的原始链路 ID（如 "a-b"），用于把双向两条记录归并回同一物理链路。
    canonical_link_id: str
    # direction 取 "duplex"（双工，两个方向都建链）或 "simplex"（单工，仅 a->b）。
    direction: str
    latency_ms: float  # 单向传播时延，毫秒；决定消息在途时间。
    loss_rate: float  # 单向丢包概率，取值 [0,1]；发送时按该概率独立丢弃。
    status: str = "normal"  # "normal" 正常 / "lost" 链路中断（中断时整条链路不投递）。
    # fault_until_s：链路因注入故障而中断的截止仿真时刻；None 表示永久故障或未故障，到点后自动恢复。
    fault_until_s: float | None = None


@dataclasses.dataclass
class _InFlightMessage:
    """在途消息：已发出但尚未到达目标的报文及其剩余传播时间。"""

    envelope: MessageEnvelope
    # remaining_s 每个 tick 递减，<=0 时投递到目标收件箱，实现链路时延效果。
    remaining_s: float


@dataclasses.dataclass(frozen=True)
class LinkState:
    """对外暴露的单向链路状态快照（不可变）。注意：供显示和控制器折叠双向链路使用。"""

    link_id: str  # 形如 "src-dst" 的单向链路 ID。
    latency_ms: float
    loss_rate: float
    status: str


def _validate_qos(latency_ms: float | None, loss_rate: float | None) -> None:
    """校验通信链路 QoS 配置。注意：非法配置应在初始化阶段尽早暴露。"""
    # 时延必须有限且非负——负时延或 NaN 会破坏在途倒计时逻辑。
    if latency_ms is not None:
        if not math.isfinite(latency_ms) or latency_ms < 0:
            raise ValueError(f"latency_ms must be a finite number >= 0, got {latency_ms}")
    # 丢包率作为概率必须落在 [0,1]，否则与 rng.random() 比较失去物理意义。
    if loss_rate is not None:
        if not math.isfinite(loss_rate) or not (0.0 <= loss_rate <= 1.0):
            raise ValueError(f"loss_rate must be a finite number in [0.0, 1.0], got {loss_rate}")


def _req_str(d: dict, key: str, where: str) -> str:
    """读取必填字符串字段。注意：缺失或类型错误会抛出配置异常。"""
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    # 类型必须是字符串，避免后续按字符串处理时出错。
    if not isinstance(v, str):
        raise ValueError(f"{where}: {key!r} must be str, got {type(v).__name__!r}")
    return v


def _req_num(d: dict, key: str, where: str) -> float:
    """读取必填数值字段。注意：布尔值不视为合法数值。"""
    if key not in d:
        raise ValueError(f"{where}: missing required field {key!r}")
    v = d[key]
    # 先排除 bool（int 子类），避免 True/False 被当作 1/0 通过。
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{where}: {key!r} must be a number, got {type(v).__name__!r}")
    return float(v)


def _check_finite_num(v: object, name: str) -> float:
    """校验数值是有限实数并转换为浮点数。注意：NaN 和无穷大不允许进入仿真。"""
    # bool 是 int 的子类，必须先排除，否则 True/False 会被误当作 1/0 通过。
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name} must be a number, got {type(v).__name__!r}")
    f = float(v)
    # NaN/Inf 一旦进入仿真会污染时间推进与状态，提前拦截。
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
            # 节点 ID 内禁止 '-'，否则会与 "src-dst" 形式的链路 ID 解析冲突。
            if "-" in nid:
                raise ValueError(f"node_id must not contain '-': {nid!r}")
            # "broadcast" 是 send() 里的保留目标关键字，不能作为真实节点。
            if nid == "broadcast":
                raise ValueError("'broadcast' is a reserved node_id")
            # 节点 ID 必须唯一，重复会导致收件箱/链路键覆盖。
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
            # 禁止自环：节点到自身的链路无物理意义。
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
            # 双工展开为正反两条有向记录；单工只建一条。两者共享同一原始 link_id。
            keys = [(a, b), (b, a)] if raw_direction == "duplex" else [(a, b)]
            if any(key in links for key in keys):
                raise ValueError(f"duplicate link: {link_id!r}")
            for key in keys:
                links[key] = _LinkConfig(link_id, raw_direction, latency_ms, loss_rate, raw_status)
            # link_index 记录原始 ID 到其有向键的映射，供双工对称操作快速反查。
            link_index[link_id] = keys

        self._nodes: list[str] = node_ids
        self._links: dict[tuple[str, str], _LinkConfig] = links
        self._link_index: dict[str, list[tuple[str, str]]] = link_index
        self._inbox: dict[str, list[MessageEnvelope]] = {nid: [] for nid in node_ids}
        self._in_flight: dict[tuple[str, str], list[_InFlightMessage]] = {}
        self._rng: np.random.Generator = np.random.default_rng(seed)
        self._seed: int = seed
        self._time_s: float = 0.0
        # 保存链路初始配置的深拷贝作为基线，reset() 据此撤销运行期注入的改动。
        self._base_links: dict[tuple[str, str], _LinkConfig] = copy.deepcopy(links)

    def reset(self) -> None:
        """复位 CommunicationChannel 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        # 清空在途消息与各节点收件箱，避免上一轮残留报文泄漏到新一轮。
        self._in_flight.clear()
        for lst in self._inbox.values():
            lst.clear()
        # 从初始化时保存的基线深拷贝恢复链路（撤销运行期注入的故障/QoS 改动）。
        self._links = copy.deepcopy(self._base_links)
        # 用同一 seed 重建随机数发生器，保证复位后丢包序列可复现。
        self._rng = np.random.default_rng(self._seed)
        self._time_s = 0.0

    def close(self) -> None:
        """释放 CommunicationChannel 持有的资源。注意：关闭后不应继续调用运行接口。"""
        # 清空链路、索引、在途消息与收件箱，释放运行期持有的全部数据。
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
            # 同一次更新内不允许对同一方向重复赋值，避免后者静默覆盖前者。
            if any(key in seen_keys for key in keys):
                raise ValueError(f"duplicate link in update_topology: {link_id!r}")
            seen_keys.update(keys)
            pending.append((keys, latency_ms, loss_rate))
        # 阶段 2：所有检查通过后再原子应用配置——保证校验失败时不留下半更新状态。
        for keys, latency_ms, loss_rate in pending:
            for key in keys:
                self._links[key].latency_ms = latency_ms
                self._links[key].loss_rate = loss_rate

    def send(self, messages: list[MessageEnvelope]) -> None:
        """发送一批通信消息。注意：链路丢失或丢包时消息可能被丢弃。"""
        for msg in messages:
            # 源节点必须是已知节点，否则该消息无从发出，直接忽略。
            if msg.source not in self._inbox:
                continue

            # 解析目标集合：广播展开为除自身外的全部节点；列表去重；单目标包成单元素列表。
            if msg.target == "broadcast":
                targets = [nid for nid in self._nodes if nid != msg.source]
            elif isinstance(msg.target, list):
                targets = list(dict.fromkeys(msg.target))
            else:
                targets = [msg.target]

            # 过滤非法目标：必须是已知节点且不能是源节点自身（不自发自收）。
            targets = [
                dst for dst in targets
                if dst in self._inbox and dst != msg.source
            ]

            for dst in targets:
                # 按 (源, 目标) 有向键查链路；查不到说明拓扑上不存在该方向链路，丢弃。
                key = (msg.source, dst)
                cfg = self._links.get(key)
                if cfg is None:
                    continue
                # 链路中断状态下整体不投递（区别于概率丢包）。
                if cfg.status == "lost":
                    continue
                # 伯努利丢包：以 loss_rate 概率丢弃本报文，体现链路误码/拥塞。
                if self._rng.random() < cfg.loss_rate:
                    continue
                # 把目标改写为具体单个节点（广播被拆成多条点对点报文）。
                delivered = dataclasses.replace(msg, target=dst)
                # 时延以毫秒配置，转换为秒后作为在途倒计时初值。
                delay_s = cfg.latency_ms / 1000.0
                self._in_flight.setdefault(key, []).append(
                    _InFlightMessage(delivered, delay_s)
                )

    def tick(self, dt_s: float) -> None:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
        dt_s = _check_finite_num(dt_s, "dt_s")
        if dt_s <= 0:
            raise ValueError(f"dt_s must be > 0, got {dt_s}")

        # 推进本模块内部时钟，故障到期判定依赖该绝对时间。
        self._time_s += dt_s

        # 自动恢复：定时故障到达截止时刻后把链路状态复位为正常。
        for cfg in self._links.values():
            if cfg.fault_until_s is not None and self._time_s >= cfg.fault_until_s:
                cfg.status = "normal"
                cfg.fault_until_s = None

        # 推进每条链路上的在途报文：剩余时延扣减一个步长。
        for key, queue in list(self._in_flight.items()):
            remaining: list[_InFlightMessage] = []
            for item in queue:
                item.remaining_s -= dt_s
                # 时延耗尽即投递到目标节点（key[1]）收件箱；否则保留继续等待。
                if item.remaining_s <= 0:
                    self._inbox[key[1]].append(item.envelope)
                else:
                    remaining.append(item)
            # 队列清空时删除该键，避免 _in_flight 无限累积空列表。
            if remaining:
                self._in_flight[key] = remaining
            else:
                del self._in_flight[key]

    def read_inbox(self, node_id: str) -> list[MessageEnvelope]:
        """读取指定节点的收件箱消息。注意：读取后会清空该节点当前缓存。"""
        if node_id not in self._inbox:
            raise KeyError(node_id)
        messages = self._inbox[node_id]
        # 读取即取走：返回后重置收件箱，确保同一消息不会被重复消费。
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
        # 按链路 ID 排序，保证快照顺序稳定、便于上层折叠和显示比对。
        states.sort(key=lambda s: s.link_id)
        return states

    def inject_link_fault(
        self, link_id: str, status: str, duration_s: float | None = None
    ) -> None:
        """注入链路故障或恢复链路。注意：目标链路会按单向链路 ID 解析。"""
        if status not in ("normal", "lost"):
            raise ValueError(f"invalid status {status!r}; must be 'normal' or 'lost'")
        # 恢复正常不应附带时长（时长只对 "lost" 的定时故障有意义）。
        if status == "normal" and duration_s is not None:
            raise ValueError("duration_s must be None when status='normal'")
        # 给定时长须为正——非正时长无法形成有效的定时恢复窗口。
        if duration_s is not None:
            duration_s = _check_finite_num(duration_s, "duration_s")
            if duration_s <= 0:
                raise ValueError(f"duration_s must be > 0, got {duration_s}")

        # 有时长则换算为绝对到期时刻（tick 中据此自动恢复）；无时长表示一直保持。
        fault_until = (self._time_s + duration_s) if duration_s is not None else None
        # 对双工链路同时设置正反两个方向，保证链路对称中断/恢复。
        for key in self._resolve_pair(link_id, key_error=True):
            cfg = self._links[key]
            cfg.status = status
            # 仅在置为 "lost" 时记录到期时刻；恢复正常时清空截止时间。
            cfg.fault_until_s = fault_until if status == "lost" else None

    def inject_link_qos(
        self,
        link_id: str,
        latency_ms: float | None,
        loss_rate: float | None,
    ) -> None:
        """注入链路 QoS 变化。注意：延迟和丢包率会覆盖该方向链路的运行参数。"""
        # 两者都为 None 表示无改动，直接返回避免无谓遍历。
        if latency_ms is None and loss_rate is None:
            return
        if latency_ms is not None:
            latency_ms = _check_finite_num(latency_ms, "latency_ms")
        if loss_rate is not None:
            loss_rate = _check_finite_num(loss_rate, "loss_rate")
        _validate_qos(latency_ms, loss_rate)
        # 双工链路两个方向一并覆盖；只更新显式给定的字段，None 字段保持原值。
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
        # 命名约定：链路 ID 恰好含一个 '-'，左右分别为源、目标节点。
        if link_id.count("-") != 1:
            raise ValueError(f"link_id must contain exactly one '-': {link_id!r}")
        a, b = link_id.split("-")
        key = (a, b)
        if key not in self._links:
            # key_error 区分语义：注入接口抛 KeyError（运行期未知目标），校验期抛 ValueError（配置错误）。
            if key_error:
                raise KeyError(link_id)
            raise ValueError(f"unknown link_id: {link_id!r}")
        cfg = self._links[key]
        # 双工链路返回正反两个方向键，使故障/QoS 操作对称作用；单工只返回给定方向。
        if cfg.direction == "duplex":
            return list(self._link_index[cfg.canonical_link_id])
        return [key]
