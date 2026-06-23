"""Low-level tests for the simulation controller facade."""

from __future__ import annotations

import json
import math
import tempfile
import time
import unittest
from pathlib import Path

from src.algorithm.context.leaf_types import FormPatE
from src.environment.model import AircraftState
from src.runner.sim_control import DisturbanceCommand, SimulationController, _build_formation_comm_init, _build_leader_route


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


def _aircraft_state(node_id: str, x_m: float, y_m: float, altitude_m: float = 1200.0) -> AircraftState:
    return AircraftState(
        node_id=node_id,
        x_m=x_m,
        y_m=y_m,
        altitude_m=altitude_m,
        speed_mps=5.0,
        theta_rad=0.0,
        psi_rad=0.0,
        ax_mps2=0.0,
        ay_mps2=0.0,
        az_mps2=0.0,
        ax_rate_mps3=0.0,
        ay_rate_mps3=0.0,
        az_rate_mps3=0.0,
        nx=0.0,
        nz=1.0,
        phi_rad=0.0,
    )


def _straight_route(speed_mps: float = 35.0) -> dict[str, object]:
    return {
        "speed_mps": speed_mps,
        "waypoints": [
            {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
            {"x_m": 1000.0, "y_m": 0.0, "altitude_m": 1000.0},
        ],
    }


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

    def test_formation_hold_stage_reports_hold(self) -> None:
        """The UI-facing report should reflect the Hold formation task, not the old rally placeholder."""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp))))

            start = controller.start()
            started_snapshot = controller.get_snapshot()
            controller.pause()
            controller.step()
            stepped_snapshot = controller.get_snapshot()

            self.assertEqual(start.code, "OK")
            self.assertEqual(started_snapshot.control_report, "保持")
            self.assertEqual(stepped_snapshot.control_report, "保持")
            controller.close()

    def test_reference_route_uses_algorithm_route_not_leader_position(self) -> None:
        """UI reference route should keep the designed segment instead of moving it onto the leader."""
        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp))
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["nodes"][0].update(
                {
                    "x_m": 140.0,
                    "y_m": 260.0,
                    "altitude_m": 1200.0,
                    "psi_v_deg": 0.0,
                    "cross_track_error_m": 0.0,
                    "distance_to_go_m": 5840.0,
                }
            )
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))

            snapshot = controller.get_snapshot()
            route = snapshot.route
            leader = snapshot.nodes[0]

            self.assertIsNotNone(route)
            assert route is not None
            self.assertAlmostEqual(route.start_x_m, 0.0)
            self.assertAlmostEqual(route.start_y_m, 0.0)
            self.assertAlmostEqual(route.start_altitude_m, 1000.0)
            self.assertAlmostEqual(route.end_x_m, 1000.0)
            self.assertAlmostEqual(route.end_y_m, 0.0)
            self.assertAlmostEqual(route.end_altitude_m, 1000.0)
            self.assertAlmostEqual(leader.cross_track_error_m or 0.0, 260.0)
            self.assertAlmostEqual(leader.distance_to_go_m or 0.0, 860.0)
            controller.close()

    def test_configured_route_is_injected_into_leader_algorithm(self) -> None:
        """Configured route should drive the leader algorithm and route-derived snapshot metrics."""
        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp))
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["route"] = {
                "start": {"x_m": 10.0, "y_m": 20.0, "altitude_m": 1100.0},
                "end": {"x_m": 210.0, "y_m": 20.0, "altitude_m": 1100.0},
                "speed_mps": 6.0,
                "radius_m": 0.0,
            }
            config["nodes"][0].update({"x_m": 60.0, "y_m": 70.0, "altitude_m": 1150.0})
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))

            snapshot = controller.get_snapshot()
            route = snapshot.route
            leader = snapshot.nodes[0]

            self.assertIsNotNone(route)
            assert route is not None
            self.assertAlmostEqual(route.start_x_m, 10.0)
            self.assertAlmostEqual(route.start_y_m, 20.0)
            self.assertAlmostEqual(route.start_altitude_m, 1100.0)
            self.assertAlmostEqual(route.end_x_m, 210.0)
            self.assertAlmostEqual(route.end_y_m, 20.0)
            self.assertAlmostEqual(route.end_altitude_m, 1100.0)
            self.assertAlmostEqual(leader.cross_track_error_m or 0.0, 50.0)
            self.assertAlmostEqual(leader.distance_to_go_m or 0.0, 150.0)
            controller.close()

    def test_route_waypoints_build_continuous_segments(self) -> None:
        """route.waypoints should define a multi-segment route without repeated start/end objects."""

        route = _build_leader_route(
            {
                "route": {
                    "speed_mps": 12.0,
                    "waypoints": [
                        {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                        {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1000.0},
                        {"x_m": 100.0, "y_m": 80.0, "altitude_m": 1000.0},
                    ],
                }
            }
        )

        self.assertEqual(len(route.lines), 2)
        self.assertAlmostEqual(route.lines[0].start.pos.east, 0.0)
        self.assertAlmostEqual(route.lines[0].end.pos.east, 100.0)
        self.assertAlmostEqual(route.lines[1].start.pos.east, 100.0)
        self.assertAlmostEqual(route.lines[1].end.pos.north, 80.0)
        self.assertTrue(all(line.vdCmd == 12.0 for line in route.lines))

    def test_snapshot_exposes_full_reference_route_segments(self) -> None:
        """Snapshot should expose the complete configured route for UI drawing, not only the active segment."""

        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp))
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["route"] = {
                "speed_mps": 12.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                    {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1000.0},
                    {"x_m": 100.0, "y_m": 80.0, "altitude_m": 1000.0},
                ],
            }
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))

            snapshot = controller.get_snapshot()

            self.assertIsNotNone(snapshot.route)
            self.assertEqual(len(snapshot.route_segments), 2)
            self.assertAlmostEqual(snapshot.route_segments[0].start_x_m, 0.0)
            self.assertAlmostEqual(snapshot.route_segments[0].end_x_m, 100.0)
            self.assertAlmostEqual(snapshot.route_segments[1].start_x_m, 100.0)
            self.assertAlmostEqual(snapshot.route_segments[1].end_y_m, 80.0)
            controller.close()

    def test_default_triangle_slots_do_not_depend_on_initial_positions(self) -> None:
        """Default formation geometry should be a fixed wedge, not derived from start positions."""
        nodes = [
            {"node_id": "A01", "role": "leader"},
            {"node_id": "A02", "role": "wingman"},
            {"node_id": "A03", "role": "wingman"},
        ]
        comm_init = _build_formation_comm_init(nodes, [])
        slots = {slot.id: slot for slot in comm_init.formPos[0]}

        self.assertEqual(comm_init.formPat, [FormPatE.TRIANGLE])
        self.assertAlmostEqual(slots["A01"].x, 0.0)
        self.assertAlmostEqual(slots["A01"].y, 0.0)
        self.assertAlmostEqual(slots["A02"].x, -54.0)
        self.assertAlmostEqual(slots["A02"].y, 58.0)
        self.assertAlmostEqual(slots["A03"].x, -54.0)
        self.assertAlmostEqual(slots["A03"].y, -58.0)
        self.assertTrue(all(slot.z == 0.0 for slot in slots.values()))

    def test_configured_formation_slots_are_injected_into_comm_init(self) -> None:
        """Formation slots from config should replace the default wedge geometry."""
        nodes = [
            {"node_id": "A01", "role": "leader"},
            {"node_id": "A02", "role": "wingman"},
            {"node_id": "A03", "role": "wingman"},
        ]
        config = {
            "formation": {
                "pattern": "TRIANGLE",
                "slots": [
                    {"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                    {"node_id": "A02", "x_m": -70.0, "y_m": 40.0, "z_m": 5.0},
                    {"node_id": "A03", "x_m": -70.0, "y_m": -40.0, "z_m": -5.0},
                ],
            }
        }

        comm_init = _build_formation_comm_init(nodes, [], config)
        slots = {slot.id: slot for slot in comm_init.formPos[0]}

        self.assertEqual(comm_init.formPat, [FormPatE.TRIANGLE])
        self.assertAlmostEqual(slots["A02"].x, -70.0)
        self.assertAlmostEqual(slots["A02"].y, 40.0)
        self.assertAlmostEqual(slots["A02"].z, 5.0)
        self.assertAlmostEqual(slots["A03"].x, -70.0)
        self.assertAlmostEqual(slots["A03"].y, -40.0)
        self.assertAlmostEqual(slots["A03"].z, -5.0)

    def test_default_triangle_slots_reject_extra_wingmen(self) -> None:
        """Default wedge geometry should fail fast when more wingmen need explicit slots."""
        nodes = [
            {"node_id": "A01", "role": "leader"},
            {"node_id": "A02", "role": "wingman"},
            {"node_id": "A03", "role": "wingman"},
            {"node_id": "A04", "role": "wingman"},
        ]

        with self.assertRaisesRegex(ValueError, "explicit slots"):
            _build_formation_comm_init(nodes, [])

    def test_current_route_uses_written_vertical_segment_after_algorithm_step(self) -> None:
        """UI route fallback should not mistake a north-south current segment for an empty route."""
        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=0.02)
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["route"] = {
                "speed_mps": 8.0,
                "radius_m": 0.0,
                "segments": [
                    {
                        "start": {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                        "end": {"x_m": 0.0, "y_m": 10.0, "altitude_m": 1000.0},
                    },
                    {
                        "start": {"x_m": 0.0, "y_m": 10.0, "altitude_m": 1000.0},
                        "end": {"x_m": 0.0, "y_m": 20.0, "altitude_m": 1000.0},
                    },
                ],
            }
            config["nodes"][0].update({"x_m": 0.0, "y_m": 12.0, "altitude_m": 1000.0})
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))

            controller.step()
            route = controller.get_snapshot().route

            self.assertIsNotNone(route)
            assert route is not None
            self.assertAlmostEqual(route.start_y_m, 10.0)
            self.assertAlmostEqual(route.end_y_m, 20.0)
            controller.close()

    def test_off_route_initial_leader_runs_past_low_speed_guard(self) -> None:
        """Leader initially off the default route should not stop when descending toward the segment."""
        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=6.0)
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["nodes"] = [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0, "speed_mps": 5.2},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1245.0, "speed_mps": 5.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1281.0, "speed_mps": 5.0},
            ]
            config["links"].append({"link_id": "A01-A03", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04})
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()

            result = controller.run_until_complete(config)
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "FINISHED")
            self.assertAlmostEqual(snapshot.time_s, 6.0)
            controller.close()

    def test_default_base_scenario_runs_past_route_endpoint(self) -> None:
        """Default route tracking should continue past the single segment end without low-speed abort."""
        config = {
            "duration_s": 220.0,
            "step_s": 0.02,
            "route": _straight_route(),
            "nodes": [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0, "speed_mps": 5.2},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1245.0, "speed_mps": 5.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1281.0, "speed_mps": 5.0},
            ],
            "links": [
                {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04},
                {"link_id": "A01-A03", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04},
            ],
        }
        controller = SimulationController()

        result = controller.run_until_complete(config)
        snapshot = controller.get_snapshot()
        nodes = {node.node_id: node for node in snapshot.nodes}

        self.assertEqual(result.code, "OK")
        self.assertEqual(snapshot.run_state, "FINISHED")
        self.assertAlmostEqual(snapshot.time_s, 220.0)
        self.assertAlmostEqual(nodes["A01"].altitude_m, 1000.0, delta=0.5)
        self.assertGreater(nodes["A01"].speed_mps, 3.0)
        controller.close()

    def test_default_followers_converge_to_same_distance_to_go(self) -> None:
        """Symmetric fixed wedge slots should make A02/A03 report the same distance-to-go after forming."""
        config = {
            "duration_s": 170.0,
            "step_s": 0.02,
            "route": _straight_route(),
            "nodes": [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0, "speed_mps": 5.2},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1245.0, "speed_mps": 5.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1281.0, "speed_mps": 5.0},
            ],
            "links": [
                {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04},
                {"link_id": "A01-A03", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04},
            ],
        }
        controller = SimulationController()

        result = controller.run_until_complete(config)
        nodes = {node.node_id: node for node in controller.get_snapshot().nodes}
        leader = nodes["A01"]
        a02 = nodes["A02"]
        a03 = nodes["A03"]

        self.assertEqual(result.code, "OK")
        self.assertAlmostEqual(a02.distance_to_go_m or 0.0, a03.distance_to_go_m or 0.0, delta=2.0)
        self.assertAlmostEqual(a02.y_m - leader.y_m, 58.0, delta=2.0)
        self.assertAlmostEqual(a03.y_m - leader.y_m, -58.0, delta=2.0)
        controller.close()

    def test_default_formation_slots_converge_to_leader_altitude(self) -> None:
        """Default Hold slots should not preserve initial follower altitude offsets."""
        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=120.0)
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["route"] = _straight_route()
            config["nodes"] = [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0, "speed_mps": 5.2},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1245.0, "speed_mps": 5.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1281.0, "speed_mps": 5.0},
            ]
            config["links"].append({"link_id": "A01-A03", "direction": "duplex", "latency_ms": 31.0, "loss_rate": 0.04})
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()

            result = controller.run_until_complete(config)
            nodes = {node.node_id: node for node in controller.get_snapshot().nodes}

            self.assertEqual(result.code, "OK")
            self.assertAlmostEqual(nodes["A01"].altitude_m, 1000.0, delta=0.5)
            self.assertAlmostEqual(nodes["A02"].altitude_m, nodes["A01"].altitude_m, delta=0.5)
            self.assertAlmostEqual(nodes["A03"].altitude_m, nodes["A01"].altitude_m, delta=0.5)
            controller.close()

    def test_off_route_leader_turns_without_snaking_backward(self) -> None:
        """Leader should smoothly cut toward the route instead of weaving forward/backward."""
        controller = SimulationController()
        controller.load_config("configs/base.json")
        controller.pause()
        east_samples: list[float] = []

        for index in range(int(25.0 / 0.005)):
            result = controller.step()
            self.assertEqual(result.code, "OK")
            if index % 100 == 0:
                east_samples.append(controller.get_snapshot().nodes[0].x_m)

        for before, after in zip(east_samples, east_samples[1:]):
            self.assertGreaterEqual(after + 0.1, before)
        controller.close()

    def test_formation_algorithm_generates_finite_controlled_motion(self) -> None:
        """Controller should drive aircraft through the formation algorithm, not the old velocity stub."""

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

            self.assertTrue(math.isfinite(after["A02"].psi_v_deg))
            self.assertTrue(math.isfinite(after["A02"].speed_mps))
            self.assertGreater(after["A02"].speed_mps, 0.0)
            self.assertGreater(after["A02"].x_m, before["A02"].x_m)
            self.assertGreater(after["A02"].y_m, before["A02"].y_m)
            self.assertNotEqual(controller._current_controls["A02"].as_vector(), (0.0, 0.0, 0.0))
            controller.close()

    def test_algorithm_pid_period_follows_simulation_step_and_decimation(self) -> None:
        """编队 PID 控制周期应由仿真步长和算法分频统一注入，避免修改帧频后仍沿用固定 dt。"""

        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=0.02, step_s=0.02)
            controller = SimulationController()
            controller.load_config(str(config_path))

            leader = controller._node_algorithms["A01"]._entity
            follower = controller._node_algorithms["A02"]._entity

            self.assertAlmostEqual(leader._pos_track._lateral._cfg.dt, 0.2)
            self.assertAlmostEqual(follower._pos_track._lateral._cfg.dt, 0.2)
            controller.close()

    def test_default_algorithm_pid_period_matches_design_contract(self) -> None:
        """默认配置下编队 PID 控制周期应为 0.005s * 10 = 0.05s。"""

        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=0.02)
            controller = SimulationController()
            controller.load_config(str(config_path))

            leader = controller._node_algorithms["A01"]._entity
            follower = controller._node_algorithms["A02"]._entity

            self.assertAlmostEqual(leader._pos_track._lateral._cfg.dt, 0.05)
            self.assertAlmostEqual(follower._pos_track._lateral._cfg.dt, 0.05)
            controller.close()

    def test_realtime_logging_records_each_algorithm_frame_with_odd_decimation(self) -> None:
        """实时 tick 路径下算法分频为奇数时，日志也应记录每个算法更新帧。"""

        with tempfile.TemporaryDirectory() as tmp:
            config = {
                "duration_s": 0.05,
                "step_s": 0.005,
                "playback_rate": 10.0,
                "algorithm_decimation": 3,
                "nodes": [
                    {"node_id": "A01", "role": "leader"},
                    {"node_id": "A02", "role": "wingman"},
                ],
                "links": [],
            }
            config_path = Path(tmp) / "odd_decimation.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            controller = SimulationController()
            controller.load_config(str(config_path))

            with controller._lock:
                controller._run_state = "RUNNING"
                while controller._run_state == "RUNNING":
                    controller._tick_unlocked()

            logged_times = [round(snapshot.time_s, 3) for snapshot in controller._logger.snapshots]

            self.assertEqual(logged_times, [0.005, 0.02, 0.035, 0.05])
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

    def test_leader_formation_broadcast_reaches_follower_after_comm_latency(self) -> None:
        """LeaderEntity outbox broadcasts must pass through CommunicationChannel and arrive at followers."""
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

            self.assertEqual(a01_inbox, [])
            self.assertTrue(any(msg.topic == "formation.leader" for msg in a02_inbox))
            controller.close()


if __name__ == "__main__":
    unittest.main()
