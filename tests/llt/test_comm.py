"""Unit tests for the communication channel (src/environment/comm.py)."""

from __future__ import annotations

import unittest

from src.common.envelope import MessageEnvelope
from src.environment.comm import CommunicationChannel


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _minimal_config(
    nodes: tuple[str, ...] = ("A01", "A02", "A03"),
    latency_ms: float = 20.0,
    loss_rate: float = 0.0,
) -> dict:
    """Three-node fully-connected config, no packet loss by default."""
    node_list = [{"node_id": n} for n in nodes]
    links = [
        {"link_id": f"{a}-{b}", "latency_ms": latency_ms, "loss_rate": loss_rate}
        for i, a in enumerate(nodes)
        for b in nodes[i + 1:]
    ]
    return {"nodes": node_list, "links": links}


def _make_msg(
    source: str,
    target: str | list[str],
    topic: str = "t",
    ts: float = 0.0,
    payload: object = None,
) -> MessageEnvelope:
    return MessageEnvelope(topic=topic, source=source, target=target, timestamp=ts, payload=payload)


def _tick_n(ch: CommunicationChannel, dt_s: float, n: int) -> None:
    for _ in range(n):
        ch.tick(dt_s)


def _ch(config: dict | None = None, seed: int = 0) -> CommunicationChannel:
    ch = CommunicationChannel()
    ch.init(config if config is not None else _minimal_config(), seed=seed)
    return ch


# ---------------------------------------------------------------------------
# 1. TestInitValidation
# ---------------------------------------------------------------------------

class TestInitValidation(unittest.TestCase):
    def test_duplicate_node_id_raises(self):
        cfg = {"nodes": [{"node_id": "A01"}, {"node_id": "A01"}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_empty_node_id_raises(self):
        cfg = {"nodes": [{"node_id": ""}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_node_id_broadcast_reserved_raises(self):
        cfg = {"nodes": [{"node_id": "broadcast"}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_node_id_contains_hyphen_raises(self):
        cfg = {"nodes": [{"node_id": "A-01"}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_link_id_no_hyphen_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01A02", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_link_id_multi_hyphen_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}, {"node_id": "A03"}],
            "links": [{"link_id": "A01-A02-A03", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_link_endpoint_unknown_node_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}],
            "links": [{"link_id": "A01-A99", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_same_direction_duplicate_link_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 0.0},
                {"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0},
            ],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_reverse_duplicate_link_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 0.0},
                {"link_id": "A02-A01", "latency_ms": 10.0, "loss_rate": 0.0},
            ],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_reverse_simplex_links_can_coexist(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {
                    "link_id": "A01-A02",
                    "direction": "simplex",
                    "latency_ms": 10.0,
                    "loss_rate": 0.0,
                },
                {
                    "link_id": "A02-A01",
                    "direction": "simplex",
                    "latency_ms": 20.0,
                    "loss_rate": 0.0,
                },
            ],
        }
        ch = _ch(cfg)
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertEqual(set(states), {"A01-A02", "A02-A01"})
        self.assertAlmostEqual(states["A01-A02"].latency_ms, 10.0)
        self.assertAlmostEqual(states["A02-A01"].latency_ms, 20.0)

    def test_invalid_direction_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{
                "link_id": "A01-A02",
                "direction": "half-duplex",
                "latency_ms": 10.0,
                "loss_rate": 0.0,
            }],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_negative_latency_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": -1.0, "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_loss_rate_above_1_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 1.1}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_loss_rate_below_0_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": -0.1}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_missing_status_defaults_normal(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        ch = _ch(cfg)
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertEqual(states["A01-A02"].status, "normal")

    def test_invalid_initial_status_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 0.0, "status": "degraded"}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_self_loop_link_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A01", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_nodes_field_missing_raises(self):
        with self.assertRaises(ValueError):
            _ch({})

    def test_node_id_missing_raises(self):
        cfg = {"nodes": [{}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_node_id_not_str_raises(self):
        cfg = {"nodes": [{"node_id": 42}], "links": []}
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_latency_ms_missing_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_latency_ms_not_number_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": "fast", "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_loss_rate_missing_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_nan_latency_ms_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": float("nan"), "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_inf_latency_ms_raises(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": float("inf"), "loss_rate": 0.0}],
        }
        with self.assertRaises(ValueError):
            _ch(cfg)

    def test_valid_config_succeeds(self):
        _ch()  # must not raise


# ---------------------------------------------------------------------------
# 2. TestBasicRouting
# ---------------------------------------------------------------------------

class TestBasicRouting(unittest.TestCase):
    def test_unicast_delivered_after_tick(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])
        self.assertEqual(ch.read_inbox("A02"), [], "must be empty before tick")
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_zero_latency_not_same_tick(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 0.0, "loss_rate": 0.0}],
        }
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02")])
        self.assertEqual(ch.read_inbox("A02"), [], "zero-latency msg must not appear before tick")
        ch.tick(0.001)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_broadcast_reaches_all_except_source(self):
        ch = _ch()
        ch.send([_make_msg("A01", "broadcast")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)
        self.assertEqual(len(ch.read_inbox("A03")), 1)
        self.assertEqual(len(ch.read_inbox("A01")), 0)

    def test_multicast_reaches_listed_targets(self):
        ch = _ch()
        ch.send([_make_msg("A01", ["A02", "A03"])])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)
        self.assertEqual(len(ch.read_inbox("A03")), 1)

    def test_unicast_target_receives_single_node_id(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        msg = ch.read_inbox("A02")[0]
        self.assertEqual(msg.target, "A02")

    def test_unconfigured_link_drops_silently(self):
        cfg = {"nodes": [{"node_id": "A01"}, {"node_id": "A02"}], "links": []}
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_initial_lost_link_drops_messages(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 0.0, "loss_rate": 0.0, "status": "lost"}],
        }
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_simplex_delivers_only_configured_direction(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{
                "link_id": "A01-A02",
                "direction": "simplex",
                "latency_ms": 0.0,
                "loss_rate": 0.0,
            }],
        }
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.001)
        self.assertEqual(len(ch.read_inbox("A02")), 1)
        self.assertEqual(ch.read_inbox("A01"), [])


# ---------------------------------------------------------------------------
# 3. TestQoS
# ---------------------------------------------------------------------------

class TestQoS(unittest.TestCase):
    def test_latency_delays_delivery(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 100.0, "loss_rate": 0.0}],
        }
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02")])
        _tick_n(ch, 0.011, 8)                                   # 88 ms
        self.assertEqual(ch.read_inbox("A02"), [], "88 ms < 100 ms: must not arrive")
        ch.tick(0.011)                                          # 99 ms
        self.assertEqual(ch.read_inbox("A02"), [], "99 ms < 100 ms: must not arrive")
        ch.tick(0.011)                                          # 110 ms
        self.assertEqual(len(ch.read_inbox("A02")), 1, "110 ms >= 100 ms: must arrive")

    def test_loss_rate_1_drops_all(self):
        ch = _ch(_minimal_config(loss_rate=1.0))
        for _ in range(10):
            ch.send([_make_msg("A01", "A02")])
            ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_loss_rate_0_delivers_all(self):
        ch = _ch(_minimal_config(loss_rate=0.0))
        for _ in range(5):
            ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 5)

    def test_same_seed_same_outcome(self):
        cfg = _minimal_config(loss_rate=0.5)
        msgs = [_make_msg("A01", "A02", ts=float(i)) for i in range(20)]
        ch1 = _ch(cfg, seed=99)
        ch2 = _ch(cfg, seed=99)
        ch1.send(msgs)
        ch2.send(msgs)
        ch1.tick(0.025)
        ch2.tick(0.025)
        ts1 = [m.timestamp for m in ch1.read_inbox("A02")]
        ts2 = [m.timestamp for m in ch2.read_inbox("A02")]
        self.assertEqual(ts1, ts2)

    def test_per_link_queue_independent(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}, {"node_id": "A03"}],
            "links": [
                {"link_id": "A01-A02", "latency_ms": 100.0, "loss_rate": 0.0},
                {"link_id": "A01-A03", "latency_ms": 20.0,  "loss_rate": 0.0},
            ],
        }
        ch = _ch(cfg)
        ch.send([_make_msg("A01", "A02"), _make_msg("A01", "A03")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A03")), 1, "20 ms link must deliver at 25 ms")
        self.assertEqual(ch.read_inbox("A02"), [],      "100 ms link must not deliver at 25 ms")


# ---------------------------------------------------------------------------
# 4. TestSendValidation
# ---------------------------------------------------------------------------

class TestSendValidation(unittest.TestCase):
    def test_unknown_source_drops_silently(self):
        ch = _ch()
        ch.send([_make_msg("X99", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_unknown_target_drops_silently(self):
        ch = _ch()
        ch.send([_make_msg("A01", "X99")])
        ch.tick(0.025)
        for nid in ("A01", "A02", "A03"):
            self.assertEqual(ch.read_inbox(nid), [])

    def test_self_send_drops_silently(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A01")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A01"), [])

    def test_duplicate_targets_deduped(self):
        ch = _ch()
        ch.send([_make_msg("A01", ["A02", "A02", "A02"])])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_empty_message_list_is_noop(self):
        ch = _ch()
        ch.send([])
        ch.tick(0.025)
        for nid in ("A01", "A02", "A03"):
            self.assertEqual(ch.read_inbox(nid), [])


# ---------------------------------------------------------------------------
# 5. TestFaultInjection
# ---------------------------------------------------------------------------

class TestFaultInjection(unittest.TestCase):
    def test_lost_drops_subsequent_messages(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_inflight_survives_fault(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])
        ch.inject_link_fault("A01-A02", "lost")
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1, "in-flight before fault must still arrive")

    def test_fault_auto_recovers_after_duration(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost", duration_s=0.1)
        _tick_n(ch, 0.011, 9)                       # 99 ms — fault still active
        ch.send([_make_msg("A01", "A02", ts=1.0)])  # dropped (fault active)
        ch.tick(0.011)                              # 110 ms — fault expires in this tick
        self.assertEqual(ch.read_inbox("A02"), [], "msg sent during fault must be dropped")
        ch.send([_make_msg("A01", "A02", ts=2.0)])  # after recovery
        ch.tick(0.025)
        self.assertGreater(len(ch.read_inbox("A02")), 0)

    def test_permanent_fault_does_not_recover(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        _tick_n(ch, 0.1, 100)                       # 10 s
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.1)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_manual_recover_clears_fault_until(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        ch.inject_link_fault("A01-A02", "normal")
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_duration_with_normal_status_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "normal", duration_s=1.0)

    def test_zero_duration_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "lost", duration_s=0.0)

    def test_negative_duration_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "lost", duration_s=-1.0)

    def test_reverse_link_id_targets_same_pair(self):
        ch = _ch()
        ch.inject_link_fault("A02-A01", "lost")
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [], "reverse inject must block A01->A02")

    def test_invalid_status_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "degraded")

    def test_unknown_link_id_raises_key_error(self):
        ch = _ch()
        with self.assertRaises(KeyError):
            ch.inject_link_fault("A01-A99", "lost")

    def test_malformed_link_id_raises_value_error(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01A02", "lost")

    def test_nan_duration_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "lost", float("nan"))

    def test_inf_duration_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_fault("A01-A02", "lost", float("inf"))

    def test_fault_affects_both_directions(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])
        self.assertEqual(ch.read_inbox("A01"), [])

    def test_simplex_fault_affects_only_exact_direction(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {
                    "link_id": "A01-A02",
                    "direction": "simplex",
                    "latency_ms": 0.0,
                    "loss_rate": 0.0,
                },
                {
                    "link_id": "A02-A01",
                    "direction": "simplex",
                    "latency_ms": 0.0,
                    "loss_rate": 0.0,
                },
            ],
        }
        ch = _ch(cfg)
        ch.inject_link_fault("A01-A02", "lost")
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.001)
        self.assertEqual(ch.read_inbox("A02"), [])
        self.assertEqual(len(ch.read_inbox("A01")), 1)


# ---------------------------------------------------------------------------
# 6. TestInjectQoS
# ---------------------------------------------------------------------------

class TestInjectQoS(unittest.TestCase):
    _TWO_NODE_20MS = {
        "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
        "links": [{"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0}],
    }

    def test_inject_latency_affects_new_messages(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.inject_link_qos("A01-A02", latency_ms=200.0, loss_rate=None)
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [], "200 ms link must not arrive at 25 ms")
        ch.tick(0.2)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_inject_latency_does_not_affect_inflight(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.send([_make_msg("A01", "A02")])           # enters at 20 ms
        ch.inject_link_qos("A01-A02", latency_ms=500.0, loss_rate=None)
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1, "in-flight must use original latency")

    def test_inject_loss_rate_1_drops_all_new(self):
        ch = _ch()
        ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate=1.0)
        for _ in range(5):
            ch.send([_make_msg("A01", "A02")])
            ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_invalid_loss_rate_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate=1.5)

    def test_invalid_latency_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=-1.0, loss_rate=None)

    def test_non_number_latency_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms="fast", loss_rate=None)

    def test_bool_latency_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=True, loss_rate=None)

    def test_bool_loss_rate_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate=True)

    def test_non_number_loss_rate_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate="high")

    def test_nan_loss_rate_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate=float("nan"))

    def test_malformed_link_id_no_hyphen_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01A02", latency_ms=10.0, loss_rate=None)

    def test_malformed_link_id_multi_hyphen_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.inject_link_qos("A01-A02-A03", latency_ms=10.0, loss_rate=None)

    def test_unknown_link_id_raises_key_error(self):
        ch = _ch()
        with self.assertRaises(KeyError):
            ch.inject_link_qos("A01-A99", latency_ms=10.0, loss_rate=None)

    def test_reverse_link_id_updates_both_directions(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.inject_link_qos("A02-A01", latency_ms=200.0, loss_rate=None)
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.025)  # 25 ms < 200 ms — neither direction should arrive
        self.assertEqual(ch.read_inbox("A02"), [], "reverse inject must update A01->A02 direction")
        self.assertEqual(ch.read_inbox("A01"), [], "reverse inject must update A02->A01 direction")

    def test_both_params_none_is_noop(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.inject_link_qos("A01-A02", latency_ms=None, loss_rate=None)  # must not raise
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1, "no-op must leave link unchanged")
        self.assertEqual(ch.read_inbox("A01"), [], "no-op must not create reverse message")

    def test_simplex_qos_affects_only_exact_direction(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {
                    "link_id": "A01-A02",
                    "direction": "simplex",
                    "latency_ms": 20.0,
                    "loss_rate": 0.0,
                },
                {
                    "link_id": "A02-A01",
                    "direction": "simplex",
                    "latency_ms": 20.0,
                    "loss_rate": 0.0,
                },
            ],
        }
        ch = _ch(cfg)
        ch.inject_link_qos("A01-A02", latency_ms=200.0, loss_rate=None)
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])
        self.assertEqual(len(ch.read_inbox("A01")), 1)


# ---------------------------------------------------------------------------
# 7. TestUpdateTopology
# ---------------------------------------------------------------------------

class TestUpdateTopology(unittest.TestCase):
    _TWO_NODE_20MS = {
        "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
        "links": [{"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0}],
    }

    def test_update_qos_affects_new_messages(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": 200.0, "loss_rate": 0.0}]})
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [], "200 ms link must not arrive at 25 ms")

    def test_update_reverse_link_id_equivalent(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.update_topology({"links": [{"link_id": "A02-A01", "latency_ms": 200.0, "loss_rate": 0.0}]})
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [], "reverse update must affect A01->A02 direction")
        self.assertEqual(ch.read_inbox("A01"), [], "reverse update must affect A02->A01 direction")

    def test_update_ignores_status_field(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.update_topology({"links": [
            {"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0, "status": "lost"}
        ]})
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertEqual(states["A01-A02"].status, "normal")

    def test_update_does_not_touch_fault_status(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.inject_link_fault("A01-A02", "lost")
        ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": 50.0, "loss_rate": 0.0}]})
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertEqual(states["A01-A02"].status, "lost")

    def test_update_does_not_clear_fault_until(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.inject_link_fault("A01-A02", "lost", duration_s=0.1)
        ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": 50.0, "loss_rate": 0.0}]})
        _tick_n(ch, 0.011, 10)                      # 110 ms — fault_until_s=0.1 should expire
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.06)
        self.assertGreater(len(ch.read_inbox("A02")), 0, "fault_until_s must survive update_topology")

    def test_inflight_messages_use_original_latency(self):
        ch = _ch(self._TWO_NODE_20MS)
        ch.send([_make_msg("A01", "A02")])           # enters at 20 ms
        ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": 500.0, "loss_rate": 0.0}]})
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1, "in-flight must use original latency")

    def test_unknown_link_id_raises_value_error(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [{"link_id": "A01-A03", "latency_ms": 10.0, "loss_rate": 0.0}]})

    def test_invalid_qos_raises_value_error(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": -1.0, "loss_rate": 0.0}]})

    def test_update_duplicate_reverse_link_raises(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [
                {"link_id": "A01-A02", "latency_ms": 30.0, "loss_rate": 0.0},
                {"link_id": "A02-A01", "latency_ms": 30.0, "loss_rate": 0.0},
            ]})

    def test_update_is_atomic_when_later_link_invalid(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}, {"node_id": "A03"}],
            "links": [
                {"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0},
                {"link_id": "A01-A03", "latency_ms": 20.0, "loss_rate": 0.0},
            ],
        }
        ch = _ch(cfg)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [
                {"link_id": "A01-A02", "latency_ms": 200.0, "loss_rate": 0.0},  # valid
                {"link_id": "A01-A03", "latency_ms": -1.0,  "loss_rate": 0.0},  # invalid QoS
            ]})
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertAlmostEqual(states["A01-A02"].latency_ms, 20.0, msg="partial update must not occur")

    def test_update_links_not_list_raises(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": "A01-A02"})

    def test_update_link_latency_ms_missing_raises(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [{"link_id": "A01-A02", "loss_rate": 0.0}]})

    def test_update_link_loss_rate_not_number_raises(self):
        ch = _ch(self._TWO_NODE_20MS)
        with self.assertRaises(ValueError):
            ch.update_topology({"links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": "high"}]})

    def test_update_reverse_simplex_links_independently(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [
                {
                    "link_id": "A01-A02",
                    "direction": "simplex",
                    "latency_ms": 20.0,
                    "loss_rate": 0.0,
                },
                {
                    "link_id": "A02-A01",
                    "direction": "simplex",
                    "latency_ms": 20.0,
                    "loss_rate": 0.0,
                },
            ],
        }
        ch = _ch(cfg)
        ch.update_topology({"links": [
            {"link_id": "A01-A02", "latency_ms": 200.0, "loss_rate": 0.0},
            {"link_id": "A02-A01", "latency_ms": 20.0, "loss_rate": 0.0},
        ]})
        ch.send([_make_msg("A01", "A02"), _make_msg("A02", "A01")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [])
        self.assertEqual(len(ch.read_inbox("A01")), 1)


# ---------------------------------------------------------------------------
# 8. TestReset
# ---------------------------------------------------------------------------

class TestReset(unittest.TestCase):
    def test_reset_clears_inbox(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        ch.reset()
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_reset_clears_inflight(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])   # 20 ms, not delivered yet
        ch.reset()
        _tick_n(ch, 0.025, 10)
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_reset_restores_base_link_config(self):
        ch = _ch()
        ch.inject_link_qos("A01-A02", latency_ms=999.0, loss_rate=None)
        ch.reset()
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertAlmostEqual(states["A01-A02"].latency_ms, 20.0)

    def test_reset_clears_runtime_fault(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        ch.reset()
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(len(ch.read_inbox("A02")), 1)

    def test_reset_restores_initial_lost_status(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 0.0, "loss_rate": 0.0, "status": "lost"}],
        }
        ch = _ch(cfg)
        ch.inject_link_fault("A01-A02", "normal")   # manually restore to normal
        ch.reset()                                   # reset must reinstate baseline lost
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        self.assertEqual(ch.read_inbox("A02"), [], "reset must restore initial lost status")

    def test_reset_reproduces_same_rng(self):
        cfg = _minimal_config(loss_rate=0.5)
        msgs = [_make_msg("A01", "A02", ts=float(i)) for i in range(30)]
        ch = _ch(cfg, seed=7)
        ch.send(msgs)
        ch.tick(0.025)
        first_run = [m.timestamp for m in ch.read_inbox("A02")]
        ch.reset()
        ch.send(msgs)
        ch.tick(0.025)
        second_run = [m.timestamp for m in ch.read_inbox("A02")]
        self.assertEqual(first_run, second_run)

    def test_reset_resets_time_s(self):
        ch = _ch()
        _tick_n(ch, 0.1, 20)                                    # advance 2 s
        ch.reset()
        ch.inject_link_fault("A01-A02", "lost", duration_s=0.1)
        _tick_n(ch, 0.011, 9)                                   # 99 ms from reset
        ch.send([_make_msg("A01", "A02", ts=1.0)])              # dropped (fault active)
        ch.tick(0.011)                                          # 110 ms — fault expires
        ch.send([_make_msg("A01", "A02", ts=2.0)])
        ch.tick(0.025)
        self.assertGreater(len(ch.read_inbox("A02")), 0, "timed fault must expire on schedule after reset")


# ---------------------------------------------------------------------------
# 9. TestReadInbox
# ---------------------------------------------------------------------------

class TestReadInbox(unittest.TestCase):
    def test_read_inbox_clears_after_read(self):
        ch = _ch()
        ch.send([_make_msg("A01", "A02")])
        ch.tick(0.025)
        ch.read_inbox("A02")
        self.assertEqual(ch.read_inbox("A02"), [])

    def test_read_inbox_unknown_node_raises(self):
        ch = _ch()
        with self.assertRaises(KeyError):
            ch.read_inbox("X99")


# ---------------------------------------------------------------------------
# 10. TestReadLinkStates
# ---------------------------------------------------------------------------

class TestReadLinkStates(unittest.TestCase):
    def test_returns_sorted_by_link_id(self):
        ch = _ch()
        states = ch.read_link_states()
        link_ids = [s.link_id for s in states]
        self.assertEqual(link_ids, sorted(link_ids))

    def test_duplex_link_returns_two_states(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{"link_id": "A01-A02", "latency_ms": 10.0, "loss_rate": 0.0}],
        }
        ch = _ch(cfg)
        states = ch.read_link_states()
        link_ids = {s.link_id for s in states}
        self.assertIn("A01-A02", link_ids)
        self.assertIn("A02-A01", link_ids)
        self.assertEqual(len(states), 2)

    def test_simplex_link_returns_one_state(self):
        cfg = {
            "nodes": [{"node_id": "A01"}, {"node_id": "A02"}],
            "links": [{
                "link_id": "A01-A02",
                "direction": "simplex",
                "latency_ms": 10.0,
                "loss_rate": 0.0,
            }],
        }
        ch = _ch(cfg)
        states = ch.read_link_states()
        self.assertEqual([s.link_id for s in states], ["A01-A02"])

    def test_reflects_current_status(self):
        ch = _ch()
        ch.inject_link_fault("A01-A02", "lost")
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertEqual(states["A01-A02"].status, "lost")
        self.assertEqual(states["A02-A01"].status, "lost")

    def test_reflects_injected_qos(self):
        ch = _ch()
        ch.inject_link_qos("A01-A02", latency_ms=77.0, loss_rate=None)
        states = {s.link_id: s for s in ch.read_link_states()}
        self.assertAlmostEqual(states["A01-A02"].latency_ms, 77.0)


# ---------------------------------------------------------------------------
# 11. TestTick
# ---------------------------------------------------------------------------

class TestTick(unittest.TestCase):
    def test_zero_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick(0.0)

    def test_negative_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick(-0.005)

    def test_nan_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick(float("nan"))

    def test_inf_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick(float("inf"))

    def test_non_number_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick("0.01")

    def test_bool_dt_raises(self):
        ch = _ch()
        with self.assertRaises(ValueError):
            ch.tick(True)

    def test_fifo_within_same_link(self):
        ch = _ch()
        msgs = [_make_msg("A01", "A02", ts=float(i)) for i in range(5)]
        ch.send(msgs)
        ch.tick(0.025)
        inbox = ch.read_inbox("A02")
        self.assertEqual(len(inbox), 5)
        self.assertEqual([m.timestamp for m in inbox], sorted(m.timestamp for m in inbox))


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main()
