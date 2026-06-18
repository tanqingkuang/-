# 通信功能 LLT 设计

测试文件：`tests/llt/test_comm.py`
被测模块：`src/environment/comm.py`
设计依据：`docs/5-1-通信功能LLD.md`

---

## 测试辅助

```python
def _minimal_config(
    nodes=("A01", "A02", "A03"),
    latency_ms=20.0,
    loss_rate=0.0,
) -> dict:
    """三节点全连接配置，默认无丢包。"""
    node_list = [{"node_id": n} for n in nodes]
    links = [
        {"link_id": f"{a}-{b}", "latency_ms": latency_ms, "loss_rate": loss_rate}
        for i, a in enumerate(nodes)
        for b in nodes[i + 1:]
    ]
    return {"nodes": node_list, "links": links}


def _make_msg(source, target, topic="t", ts=0.0, payload=None):
    return MessageEnvelope(topic=topic, source=source, target=target,
                           timestamp=ts, payload=payload)


def _tick_n(ch, dt_s, n):
    for _ in range(n):
        ch.tick(dt_s)
```

---

## 1. TestInitValidation — init 输入校验

| 测试名 | 断言 |
|--------|------|
| `test_duplicate_node_id_raises` | 两个同名节点 → `ValueError` |
| `test_empty_node_id_raises` | `node_id=""` → `ValueError` |
| `test_node_id_broadcast_reserved_raises` | `node_id="broadcast"` → `ValueError` |
| `test_node_id_contains_hyphen_raises` | `node_id="A-01"` → `ValueError` |
| `test_link_id_no_hyphen_raises` | `link_id="A01A02"` → `ValueError` |
| `test_link_id_multi_hyphen_raises` | `link_id="A01-A02-A03"` → `ValueError` |
| `test_link_endpoint_unknown_node_raises` | link 两端含未知节点 → `ValueError` |
| `test_same_direction_duplicate_link_raises` | 两条 `"A01-A02"` → `ValueError` |
| `test_reverse_duplicate_link_raises` | 同时配置 `"A01-A02"` 和 `"A02-A01"` → `ValueError` |
| `test_reverse_simplex_links_can_coexist` | 两条互为反向的 `direction="simplex"` 链路允许共存，并各自保留 QoS |
| `test_invalid_direction_raises` | `direction` 非 `"duplex"` / `"simplex"` → `ValueError` |
| `test_negative_latency_raises` | `latency_ms=-1` → `ValueError` |
| `test_loss_rate_above_1_raises` | `loss_rate=1.1` → `ValueError` |
| `test_loss_rate_below_0_raises` | `loss_rate=-0.1` → `ValueError` |
| `test_missing_status_defaults_normal` | link 不带 `status` 字段，`read_link_states()` 返回 `status="normal"` |
| `test_invalid_initial_status_raises` | `status="degraded"` → `ValueError` |
| `test_self_loop_link_raises` | `link_id="A01-A01"`（两端节点相同）→ `ValueError` |
| `test_nodes_field_missing_raises` | config 无 `nodes` 字段 → `ValueError` |
| `test_node_id_missing_raises` | node dict 缺 `node_id` 字段 → `ValueError` |
| `test_node_id_not_str_raises` | `node_id: 42`（非字符串）→ `ValueError` |
| `test_latency_ms_missing_raises` | link dict 缺 `latency_ms` 字段 → `ValueError` |
| `test_latency_ms_not_number_raises` | `latency_ms: "fast"`（非数字）→ `ValueError` |
| `test_loss_rate_missing_raises` | link dict 缺 `loss_rate` 字段 → `ValueError` |
| `test_nan_latency_ms_raises` | `latency_ms=float("nan")` → `ValueError`（NaN 绕过 `< 0` 检查，需 `math.isfinite`） |
| `test_inf_latency_ms_raises` | `latency_ms=float("inf")` → `ValueError`（Inf 同理） |
| `test_valid_config_succeeds` | 合法配置不抛异常 |

---

## 2. TestBasicRouting — 基本路由与帧因果

| 测试名 | 断言 |
|--------|------|
| `test_unicast_delivered_after_tick` | 发送后当帧 inbox 为空；tick 后消息到达 |
| `test_zero_latency_not_same_tick` | `latency_ms=0` 的链路，send 后 inbox 仍为空，tick 后才到达 |
| `test_broadcast_reaches_all_except_source` | broadcast → 除 source 外所有节点收到，source 自身不收 |
| `test_multicast_reaches_listed_targets` | `target=["A02","A03"]` → 仅这两个节点收到 |
| `test_unicast_target_receives_single_node_id` | 收件箱中消息的 `target` 字段为接收节点自身 ID，不是 `"broadcast"` |
| `test_unconfigured_link_drops_silently` | A01→A02 无链路配置，消息静默丢弃，A02 inbox 为空 |
| `test_initial_lost_link_drops_messages` | 链路 init 时 `status="lost"`，发送后 tick，A02 inbox 为空 |
| `test_simplex_delivers_only_configured_direction` | `simplex A01-A02` 只投递 A01→A02，A02→A01 静默丢弃 |

---

## 3. TestQoS — 延迟与丢包

| 测试名 | 断言 |
|--------|------|
| `test_latency_delays_delivery` | `latency_ms=100`，`dt_s=0.011`：tick 8 次（88 ms）后 inbox 仍空，tick 第 9 次（99 ms 累计）后 inbox 仍空，tick 第 10 次（110 ms）到达 |
| `test_loss_rate_1_drops_all` | `loss_rate=1.0`，多次 tick 后 inbox 始终为空 |
| `test_loss_rate_0_delivers_all` | `loss_rate=0.0`，每条消息都到达 |
| `test_same_seed_same_outcome` | 两个相同 seed 的实例，发送相同消息序列，inbox 结果完全一致 |
| `test_per_link_queue_independent` | A01→A02 延迟 100 ms，A01→A03 延迟 20 ms；先到 A03 inbox，后到 A02 inbox |

---

## 4. TestSendValidation — send 非法输入

| 测试名 | 断言 |
|--------|------|
| `test_unknown_source_drops_silently` | source 不在节点列表，所有目标 inbox 为空 |
| `test_unknown_target_drops_silently` | target 为未知 node_id，消息静默丢弃 |
| `test_self_send_drops_silently` | source == target，消息静默丢弃 |
| `test_duplicate_targets_deduped` | `target=["A02","A02"]`，A02 只收到一条 |
| `test_empty_message_list_is_noop` | `send([])` 不抛异常，inbox 为空 |

---

## 5. TestFaultInjection — 故障注入

| 测试名 | 断言 |
|--------|------|
| `test_lost_drops_subsequent_messages` | inject lost 后，新发消息不到达 |
| `test_inflight_survives_fault` | 消息入队后注入 lost，tick 到期后仍送达 |
| `test_fault_auto_recovers_after_duration` | `duration_s=0.1`，`dt_s=0.011`：tick 9 次（99 ms）后消息仍不达，tick 第 10 次（110 ms）后链路恢复，新消息可达 |
| `test_permanent_fault_does_not_recover` | `duration_s=None`，经过足够长时间 tick 后仍为 lost |
| `test_manual_recover_clears_fault_until` | 手动 `inject_link_fault("A01-A02","normal")`，消息重新可达 |
| `test_duration_with_normal_status_raises` | `inject_link_fault("A01-A02","normal", duration_s=1.0)` → `ValueError` |
| `test_zero_duration_raises` | `duration_s=0.0` → `ValueError` |
| `test_negative_duration_raises` | `duration_s=-1.0` → `ValueError` |
| `test_reverse_link_id_targets_same_pair` | `inject_link_fault("A02-A01","lost")` 等价于 `"A01-A02"` |
| `test_invalid_status_raises` | `inject_link_fault("A01-A02","degraded")` → `ValueError` |
| `test_unknown_link_id_raises_key_error` | `inject_link_fault("A01-A99","lost")` → `KeyError` |
| `test_malformed_link_id_raises_value_error` | `inject_link_fault("A01A02","lost")` → `ValueError` |
| `test_nan_duration_raises` | `duration_s=float("nan")` → `ValueError` |
| `test_inf_duration_raises` | `duration_s=float("inf")` → `ValueError` |
| `test_fault_affects_both_directions` | inject lost 后，A01→A02 和 A02→A01 均不通 |
| `test_simplex_fault_affects_only_exact_direction` | 对 `simplex A01-A02` 注入 lost，只阻断 A01→A02，不影响反向 simplex |

---

## 6. TestInjectQoS — 运行期 QoS 注入

| 测试名 | 断言 |
|--------|------|
| `test_inject_latency_affects_new_messages` | 注入更大延迟后，新发消息按新延迟到达 |
| `test_inject_latency_does_not_affect_inflight` | 注入延迟前已在途消息按原延迟到达 |
| `test_inject_loss_rate_1_drops_all_new` | 注入 `loss_rate=1.0` 后新消息全丢 |
| `test_invalid_loss_rate_raises` | `loss_rate=1.5` → `ValueError` |
| `test_invalid_latency_raises` | `latency_ms=-1` → `ValueError` |
| `test_non_number_latency_raises` | `latency_ms="fast"`（字符串）→ `ValueError` |
| `test_bool_latency_raises` | `latency_ms=True`（bool）→ `ValueError` |
| `test_bool_loss_rate_raises` | `loss_rate=True`（bool）→ `ValueError` |
| `test_non_number_loss_rate_raises` | `loss_rate="high"`（字符串）→ `ValueError` |
| `test_nan_loss_rate_raises` | `loss_rate=float("nan")` → `ValueError` |
| `test_malformed_link_id_no_hyphen_raises` | `link_id="A01A02"` → `ValueError` |
| `test_malformed_link_id_multi_hyphen_raises` | `link_id="A01-A02-A03"` → `ValueError` |
| `test_unknown_link_id_raises_key_error` | 未知 `link_id` → `KeyError` |
| `test_reverse_link_id_updates_both_directions` | 更新 `"A02-A01"` 的 latency，A01→A02 和 A02→A01 两个方向均生效 |
| `test_both_params_none_is_noop` | `latency_ms=None, loss_rate=None` 时不报错，链路参数不变，消息正常送达 |
| `test_simplex_qos_affects_only_exact_direction` | 对 `simplex A01-A02` 注入 QoS，只影响 A01→A02，不影响反向 simplex |

---

## 7. TestUpdateTopology — 拓扑参数更新

| 测试名 | 断言 |
|--------|------|
| `test_update_qos_affects_new_messages` | 调用 `update_topology` 修改 latency_ms 后，新消息按新参数投递 |
| `test_update_reverse_link_id_equivalent` | `"A02-A01"` 与 `"A01-A02"` 等价，双向同时更新 |
| `test_update_ignores_status_field` | config 带 `status="lost"` 时，链路 status 不变 |
| `test_update_does_not_touch_fault_status` | 链路处于 lost 状态时调用 `update_topology`，status 仍为 lost |
| `test_update_does_not_clear_fault_until` | 有时限故障时调用 `update_topology`，tick 超过原 duration 后链路自动恢复（以此证明 `fault_until_s` 未被清零，黑盒可验证） |
| `test_inflight_messages_use_original_latency` | 消息入队后更新延迟，在途消息按原延迟到达 |
| `test_unknown_link_id_raises_value_error` | 传入基线外的 `link_id` → `ValueError` |
| `test_invalid_qos_raises_value_error` | `latency_ms=-1` → `ValueError` |
| `test_update_duplicate_reverse_link_raises` | 同一次调用中同时传 `"A01-A02"` 和 `"A02-A01"` → `ValueError` |
| `test_update_is_atomic_when_later_link_invalid` | 第一条链路合法、第二条 QoS 非法时，第一条链路的参数不被修改（原子性） |
| `test_update_links_not_list_raises` | `links` 字段为非列表（如字符串）→ `ValueError` |
| `test_update_link_latency_ms_missing_raises` | link dict 缺 `latency_ms` 字段 → `ValueError` |
| `test_update_link_loss_rate_not_number_raises` | `loss_rate: "high"`（非数字）→ `ValueError` |
| `test_update_reverse_simplex_links_independently` | 同一次 update 可分别更新互为反向的两条 simplex 链路 |

---

## 8. TestReset — reset 语义

| 测试名 | 断言 |
|--------|------|
| `test_reset_clears_inbox` | tick 后 inbox 有消息，reset 后 inbox 为空 |
| `test_reset_clears_inflight` | 消息入队未到期，reset 后 tick 多次 inbox 仍为空 |
| `test_reset_restores_base_link_config` | 运行期注入 `inject_link_qos` 后 reset，链路恢复 init 参数 |
| `test_reset_clears_runtime_fault` | 运行期注入 lost 故障后 reset，链路恢复 normal |
| `test_reset_restores_initial_lost_status` | init 时 `status="lost"`，运行期手动恢复 normal，reset 后再次丢包（基线恢复） |
| `test_reset_reproduces_same_rng` | reset 后发送相同消息，丢包结果与首次 init 后完全一致 |
| `test_reset_resets_time_s` | 经过若干 tick 后 reset，内部 `_time_s` 归零（验证方式：注入有时限故障后 reset，重新发送消息可达） |

---

## 9. TestReadInbox — read_inbox 接口

| 测试名 | 断言 |
|--------|------|
| `test_read_inbox_clears_after_read` | 连续两次 `read_inbox` 第二次返回空列表 |
| `test_read_inbox_unknown_node_raises` | `read_inbox("X99")` → `KeyError` |

---

## 10. TestReadLinkStates — read_link_states 接口

| 测试名 | 断言 |
|--------|------|
| `test_returns_sorted_by_link_id` | 返回列表按 `link_id` 字典序排列 |
| `test_duplex_link_returns_two_states` | 一条 duplex 链路展开后返回 `"A01-A02"` 和 `"A02-A01"` 两条 |
| `test_simplex_link_returns_one_state` | 一条 simplex 链路只返回配置方向的一条状态 |
| `test_reflects_current_status` | 注入 lost 后，对应 `LinkState.status == "lost"` |
| `test_reflects_injected_qos` | 注入新 latency 后，对应 `LinkState.latency_ms` 更新 |

---

## 11. TestTick — tick 约束

| 测试名 | 断言 |
|--------|------|
| `test_zero_dt_raises` | `tick(0.0)` → `ValueError` |
| `test_negative_dt_raises` | `tick(-0.005)` → `ValueError` |
| `test_nan_dt_raises` | `tick(float("nan"))` → `ValueError` |
| `test_inf_dt_raises` | `tick(float("inf"))` → `ValueError` |
| `test_non_number_dt_raises` | `tick("0.01")`（字符串）→ `ValueError` |
| `test_bool_dt_raises` | `tick(True)`（bool）→ `ValueError` |
| `test_fifo_within_same_link` | 同一链路连续发两条消息，到达顺序与发送顺序一致 |

> **说明（不写测试）**：跨链路同 tick 到达的顺序不保证，LLD 明确此为未定义行为。算法层如需排序，使用 `MessageEnvelope.timestamp`。

---

## 覆盖矩阵

| LLD 需求 | 覆盖测试类 |
|----------|-----------|
| init 配置校验 | TestInitValidation |
| 帧因果（零延迟也进 in_flight） | TestBasicRouting |
| 广播/多播展开 | TestBasicRouting |
| duplex / simplex 方向语义 | TestBasicRouting, TestInitValidation, TestReadLinkStates |
| 未配置链路丢弃 | TestBasicRouting |
| 延迟与丢包 QoS | TestQoS |
| seed 可复现 | TestQoS |
| send 非法输入静默丢弃 | TestSendValidation |
| 故障注入与自动恢复 | TestFaultInjection |
| inject_* link_id 归一化 | TestFaultInjection, TestInjectQoS |
| simplex 注入只影响精确方向 | TestFaultInjection, TestInjectQoS |
| update_topology 不改 status | TestUpdateTopology |
| update_topology 按方向记录判重 | TestUpdateTopology |
| reset 恢复基线 | TestReset |
| read_inbox 清空语义 | TestReadInbox |
| read_link_states 字典序 | TestReadLinkStates |
| tick dt_s 非 bool 有限数字且 > 0 | TestTick |
