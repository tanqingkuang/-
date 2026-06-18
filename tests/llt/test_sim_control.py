"""Low-level tests for the simulation controller facade."""

from __future__ import annotations

import json
import math
import tempfile
import time
import unittest
from pathlib import Path

from src.runner.sim_control import DisturbanceCommand, SimulationController


def _write_config(directory: Path, *, duration_s: float = 0.03, step_s: float = 0.005) -> Path:
    config = {
        "duration_s": duration_s,
        "step_s": step_s,
        "playback_rate": 10.0,
        "nodes": [
            {"node_id": "A01", "role": "leader", "x_m": 0, "y_m": 0, "altitude_m": 1200},
            {"node_id": "A02", "role": "wingman", "x_m": -45, "y_m": 50, "altitude_m": 1215},
        ],
        "links": [
            {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04},
        ],
    }
    path = directory / "case.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return path


class SimulationControllerTests(unittest.TestCase):
    """Exercise the HLD-level application contract."""

    def test_initial_snapshot_is_unloaded(self) -> None:
        controller = SimulationController()
        snapshot = controller.get_snapshot()

        self.assertEqual(snapshot.run_state, "UNLOADED")
        self.assertEqual(snapshot.nodes, [])
        controller.close()

    def test_load_config_enters_ready_and_exposes_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            result = controller.load_config(str(_write_config(Path(tmp))))

            snapshot = controller.get_snapshot()
            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "READY")
            self.assertEqual(snapshot.duration_s, 0.03)
            self.assertEqual([node.node_id for node in snapshot.nodes], ["A01", "A02"])
            self.assertEqual(len(snapshot.links), 1)
            self.assertEqual(snapshot.links[0].link_id, "A01-A02")
            self.assertEqual(snapshot.links[0].latency_ms, 31.0)
            self.assertEqual(snapshot.links[0].loss_rate, 0.04)
            self.assertTrue(controller.get_recent_events())
            controller.close()

    def test_manual_step_advances_and_stays_paused(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            result = controller.step(2)
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "PAUSED")
            self.assertAlmostEqual(snapshot.time_s, 0.01)
            self.assertGreater(snapshot.nodes[0].x_m, 0.0)
            controller.close()

    def test_stub_algorithm_keeps_aircraft_heading_stable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = {
                "duration_s": 0.03,
                "step_s": 0.005,
                "nodes": [
                    {"node_id": "A01", "role": "leader", "x_m": 0, "y_m": 0, "altitude_m": 1200},
                    {
                        "node_id": "A02",
                        "role": "wingman",
                        "x_m": -45,
                        "y_m": 50,
                        "altitude_m": 1215,
                        "psi_v_deg": 30.0,
                        "speed_mps": 7.0,
                    },
                ],
                "links": [],
            }
            config_path = Path(tmp) / "heading.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))
            before = {node.node_id: node for node in controller.get_snapshot().nodes}

            controller.step()
            after = {node.node_id: node for node in controller.get_snapshot().nodes}

            self.assertAlmostEqual(after["A02"].psi_v_deg, 30.0)
            self.assertAlmostEqual(after["A02"].speed_mps, 7.0)
            self.assertAlmostEqual(after["A02"].x_m, before["A02"].x_m + 7.0 * 0.005 * math.cos(math.radians(30.0)))
            self.assertAlmostEqual(after["A02"].y_m, before["A02"].y_m + 7.0 * 0.005 * math.sin(math.radians(30.0)))
            controller.close()

    def test_run_until_complete_finishes_synchronously(self) -> None:
        controller = SimulationController()

        result = controller.run_until_complete(
            {
                "duration_s": 0.015,
                "step_s": 0.005,
                "nodes": [{"node_id": "A01"}],
                "links": [],
            }
        )
        snapshot = controller.get_snapshot()

        self.assertEqual(result.code, "OK")
        self.assertEqual(snapshot.run_state, "FINISHED")
        self.assertAlmostEqual(snapshot.time_s, 0.015)
        controller.close()

    def test_empty_config_does_not_create_default_aircraft_or_links(self) -> None:
        controller = SimulationController()

        result = controller.run_until_complete({"duration_s": 0.005, "step_s": 0.005})
        snapshot = controller.get_snapshot()

        self.assertEqual(result.code, "OK")
        self.assertEqual(snapshot.nodes, [])
        self.assertEqual(snapshot.links, [])
        controller.close()

    def test_start_without_config_is_rejected(self) -> None:
        controller = SimulationController()

        result = controller.start()

        self.assertEqual(result.code, "ERR_NO_CONFIG")
        controller.close()

    def test_finished_requires_reset_before_start(self) -> None:
        controller = SimulationController()
        controller.run_until_complete({"duration_s": 0.005, "step_s": 0.005})

        result = controller.start()

        self.assertEqual(result.code, "ERR_INVALID_STATE")
        controller.close()

    def test_pause_then_start_continues_background_worker(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=1.0, step_s=0.01)))

            self.assertEqual(controller.start().code, "OK")
            time.sleep(0.04)
            self.assertEqual(controller.pause().code, "OK")
            paused_time = controller.get_snapshot().time_s
            self.assertEqual(controller.start().code, "OK")
            time.sleep(0.04)
            resumed_time = controller.get_snapshot().time_s

            self.assertGreater(resumed_time, paused_time)
            controller.close()

    def test_reset_after_finish_returns_ready_at_zero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=0.005)))
            controller.step()

            result = controller.reset()
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "READY")
            self.assertEqual(snapshot.time_s, 0.0)
            controller.close()

    def test_disturbance_records_recent_event(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            result = controller.inject_disturbance(
                DisturbanceCommand("node_fault", target="A02", duration_s=1.0, params={"mode": "fault"})
            )
            events = controller.get_recent_events(min_level="INFO")

            self.assertEqual(result.code, "OK")
            self.assertTrue(any("node_fault" in event.message for event in events))
            controller.close()

    def test_link_fault_sets_link_status_lost(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            result = controller.inject_disturbance(
                {"type": "link_fault", "target": "A01-A02", "duration_s": 1.0, "params": {}}
            )
            controller.step(2)
            links = {s.link_id: s for s in controller.get_snapshot().links}
            link = links["A01-A02"]

            self.assertEqual(result.code, "OK")
            self.assertEqual(link.status, "lost")
            self.assertAlmostEqual(link.latency_ms, 31.0)   # QoS unchanged
            self.assertAlmostEqual(link.loss_rate, 0.04)    # QoS unchanged
            controller.close()

    def test_link_loss_degrades_link_loss_rate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            result = controller.inject_disturbance(
                {"type": "link_loss", "target": "A01-A02", "duration_s": 1.0, "params": {"loss_rate": 0.9}}
            )
            controller.step(2)
            links = {s.link_id: s for s in controller.get_snapshot().links}
            link = links["A01-A02"]

            self.assertEqual(result.code, "OK")
            self.assertEqual(link.status, "normal")          # fault status unchanged
            self.assertAlmostEqual(link.latency_ms, 31.0)   # latency unchanged
            self.assertAlmostEqual(link.loss_rate, 0.9)     # degraded to injected rate
            controller.close()

    def test_snapshot_exposes_configured_links_not_internal_directions(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = {
                "duration_s": 0.03,
                "step_s": 0.005,
                "nodes": [
                    {"node_id": "A01", "x_m": 0, "y_m": 0, "altitude_m": 1000},
                    {"node_id": "A02", "x_m": 10, "y_m": 0, "altitude_m": 1000},
                    {"node_id": "A03", "x_m": 20, "y_m": 0, "altitude_m": 1000},
                ],
                "links": [
                    {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 18.0, "loss_rate": 0.01},
                    {"link_id": "A02-A03", "direction": "simplex", "latency_ms": 30.0, "loss_rate": 0.02},
                ],
            }
            path = Path(tmp) / "case.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            result = controller.load_config(str(path))
            links = controller.get_snapshot().links

            self.assertEqual(result.code, "OK")
            self.assertEqual([link.link_id for link in links], ["A01-A02", "A02-A03"])
            self.assertEqual([link.direction for link in links], ["duplex", "simplex"])
            controller.close()

    def test_subscribe_snapshot_invokes_callback_and_unsubscribes(self) -> None:
        controller = SimulationController()
        seen = []

        subscription = controller.subscribe_snapshot(seen.append)
        subscription.unsubscribe()
        controller.step()

        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0].run_state, "UNLOADED")
        controller.close()

    def test_duplicate_subscribe_does_not_duplicate_refreshes(self) -> None:
        controller = SimulationController()
        seen = []

        first = controller.subscribe_snapshot(seen.append)
        second = controller.subscribe_snapshot(seen.append)
        first.unsubscribe()
        second.unsubscribe()

        self.assertEqual(len(seen), 2)
        self.assertEqual(len(controller._subscribers), 0)
        controller.close()

    def test_callback_error_records_warn_event(self) -> None:
        controller = SimulationController()

        def broken_callback(_snapshot: object) -> None:
            raise RuntimeError("boom")

        controller.subscribe_snapshot(broken_callback)
        events = controller.get_recent_events(min_level="WARN")

        self.assertTrue(any("callback failed" in event.message for event in events))
        controller.close()

    def test_wind_expires_independently_of_concurrent_fault(self) -> None:
        """Short wind overlapping a long fault must cancel when wind expires."""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            # Wind lasts < 1 step (step_s=0.005); fault lasts 1 s
            controller.inject_disturbance(
                DisturbanceCommand("wind", duration_s=0.003,
                                   params={"speed_mps": 50.0, "direction_deg": 0.0})
            )
            controller.inject_disturbance(
                DisturbanceCommand("node_fault", target="A02", duration_s=1.0,
                                   params={"mode": "fault"})
            )
            # 2 ticks: first tick at t=0.0 (wind active), second at t=0.005 (wind expires)
            controller.step(2)
            wind = controller._disturbance._model._wind_velocity_mps
            self.assertEqual(wind, (0.0, 0.0, 0.0))
            controller.close()

    def test_wind_expiry_does_not_cancel_link_fault(self) -> None:
        """Wind expiring must not cancel a still-active link_fault."""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            # link_fault on A01-A02 lasts 1 s
            controller.inject_disturbance(
                DisturbanceCommand("link_fault", target="A01-A02", duration_s=1.0, params={})
            )
            # wind expires after first tick (0.003 s < step 0.005 s)
            controller.inject_disturbance(
                DisturbanceCommand("wind", duration_s=0.003,
                                   params={"speed_mps": 10.0, "direction_deg": 0.0})
            )

            # step twice: wind expires at tick 1, link_fault must survive
            controller.step(2)

            links = {s.link_id: s for s in controller.get_snapshot().links}
            self.assertEqual(
                links["A01-A02"].status, "lost",
                "link_fault must remain active after wind expiry",
            )
            controller.close()

    def test_error_code_paths_and_recent_event_filters(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            missing = controller.load_config(str(Path(tmp) / "missing.json"))
            invalid_path = Path(tmp) / "invalid.json"
            invalid_path.write_text("{", encoding="utf-8")
            invalid = controller.load_config(str(invalid_path))
            controller._append_event_unlocked("DEBUG", "Test", "debug")
            controller._append_event_unlocked("WARN", "Test", "warn")

            self.assertEqual(missing.code, "ERR_CONFIG_NOT_FOUND")
            self.assertEqual(invalid.code, "ERR_CONFIG_INVALID")
            self.assertEqual(controller.set_playback_rate(20.0).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual(controller.step(0).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual(controller.inject_disturbance({"type": "bad"}).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual([event.level for event in controller.get_recent_events(limit=1, min_level="WARN")], ["WARN"])
            controller.close()

    def test_broadcast_reaches_other_nodes_after_comm_latency(self) -> None:
        """Algorithm outbox broadcasts must pass through CommunicationChannel and arrive in inbox."""
        config = {
            "duration_s": 0.1,
            "step_s": 0.005,
            "nodes": [
                {"node_id": "A01", "x_m": 0, "y_m": 0, "altitude_m": 1000},
                {"node_id": "A02", "x_m": 10, "y_m": 0, "altitude_m": 1000},
            ],
            "links": [
                {"link_id": "A01-A02", "latency_ms": 20.0, "loss_rate": 0.0},
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "case.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(path))
            controller.step(16)
            a01_inbox = controller._comm.read_inbox("A01")
            a02_inbox = controller._comm.read_inbox("A02")

            self.assertTrue(any(msg.topic == "node.status" for msg in a01_inbox))
            self.assertTrue(any(msg.topic == "node.status" for msg in a02_inbox))
            controller.close()


if __name__ == "__main__":
    unittest.main()
