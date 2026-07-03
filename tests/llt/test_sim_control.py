"""Low-level tests for the simulation controller facade."""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from src.algorithm.context.leaf_types import FormCommInitS, FormStageE, MotionProfS, PosInEarthS, WayPointInputS
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.algo.arc_path import arc_radius
from src.environment.model import AircraftState
from src.runner.sim_control import (
    DisturbanceCommand,
    LinkState,
    NodeState,
    RouteState,
    SimulationController,
    SimulationSnapshot,
    _DataLogger,
    _NodeAlgorithm,
    _build_formation_comm_init,
    _build_leader_route,
    _build_vel_cmd_limit,
    _route_state_from_wayline,
)


def _write_config(
    directory: Path,
    *,
    duration_s: float = 0.03,
    step_s: float = 0.005,
    playback_rate: float = 10.0,
) -> Path:
    config = {
        "duration_s": duration_s,
        "step_s": step_s,
        "playback_rate": playback_rate,
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
        psi_dot_deg_s=0.0,
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
            self.assertAlmostEqual(leader.cross_track_error_m or 0.0, -260.0)
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
            self.assertAlmostEqual(leader.cross_track_error_m or 0.0, -50.0)
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
        lines = waypoint_inputs_to_waylines(route)

        self.assertEqual(len(lines), 2)
        self.assertAlmostEqual(lines[0].start.pos.east, 0.0)
        self.assertAlmostEqual(lines[0].end.pos.east, 100.0)
        self.assertAlmostEqual(lines[1].start.pos.east, 100.0)
        self.assertAlmostEqual(lines[1].end.pos.north, 80.0)
        self.assertTrue(all(line.start.vdCmd == 12.0 for line in lines))

    def test_route_segments_preserve_per_segment_speed(self) -> None:
        """route.segments 相邻航段速度不同时，每段应使用自己的速度。"""

        route = _build_leader_route(
            {
                "route": {
                    "segments": [
                        {
                            "speed_mps": 10.0,
                            "start": {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                            "end": {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1000.0},
                        },
                        {
                            "speed_mps": 20.0,
                            "start": {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1000.0},
                            "end": {"x_m": 100.0, "y_m": 80.0, "altitude_m": 1000.0},
                        },
                    ]
                }
            }
        )
        lines = waypoint_inputs_to_waylines(route)

        self.assertEqual([line.start.vdCmd for line in lines], [10.0, 20.0])

    def test_route_waypoints_radius_inserts_tangent_arc(self) -> None:
        """内部拐点 R>0 时应在直线段间插入与两腿相切的圆弧段(东->北左转)。"""

        route = _build_leader_route(
            {
                "route": {
                    "speed_mps": 20.0,
                    "waypoints": [
                        {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 0.0},
                        {"x_m": 2000.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 400.0},
                        {"x_m": 2000.0, "y_m": 2000.0, "altitude_m": 1000.0, "R": 0.0},
                    ],
                }
            }
        )
        lines = waypoint_inputs_to_waylines(route)

        # 直线(0,0)->(1600,0) + 圆弧(1600,0)->(2000,400) + 直线(2000,400)->(2000,2000)
        self.assertEqual(len(lines), 3)
        leg_in, arc, leg_out = lines
        self.assertEqual(leg_in.start.turnSign, 0.0)
        self.assertAlmostEqual(leg_in.end.pos.east, 1600.0)
        self.assertAlmostEqual(leg_in.end.pos.north, 0.0)
        self.assertAlmostEqual(arc_radius(arc), 400.0)
        self.assertAlmostEqual(arc.start.turnSign, 1.0)  # 左转/逆时针
        self.assertAlmostEqual(arc.start.pos.east, 1600.0)
        self.assertAlmostEqual(arc.end.pos.east, 2000.0)
        self.assertAlmostEqual(arc.end.pos.north, 400.0)
        self.assertAlmostEqual(arc.start.center.east, 1600.0)
        self.assertAlmostEqual(arc.start.center.north, 400.0)
        self.assertEqual(leg_out.start.turnSign, 0.0)
        self.assertAlmostEqual(leg_out.start.pos.north, 400.0)
        self.assertAlmostEqual(leg_out.end.pos.north, 2000.0)

    def test_arc_route_cross_track_error_uses_radial_distance_not_chord(self) -> None:
        """圆弧当前航段的侧偏应按到圆弧的径向偏差计算，而不是到弦线的距离。"""

        route = _build_leader_route(
            {
                "route": {
                    "speed_mps": 20.0,
                    "waypoints": [
                        {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 0.0},
                        {"x_m": 2000.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 400.0},
                        {"x_m": 2000.0, "y_m": 2000.0, "altitude_m": 1000.0, "R": 0.0},
                    ],
                }
            }
        )
        arc = waypoint_inputs_to_waylines(route)[1]
        current_route = _route_state_from_wayline(arc)
        on_arc_midpoint = _aircraft_state(
            "A01",
            arc.start.center.east + arc_radius(arc) / math.sqrt(2.0),
            arc.start.center.north - arc_radius(arc) / math.sqrt(2.0),
            1000.0,
        )

        self.assertAlmostEqual(
            SimulationController._cross_track_error(on_arc_midpoint, current_route) or 0.0,
            0.0,
            delta=1e-9,
        )

    def test_cross_track_error_positive_to_track_right_side(self) -> None:
        """整体侧偏沿用航迹坐标系右侧向为正的符号约定。"""

        eastbound_route = RouteState(
            start_x_m=0.0,
            start_y_m=0.0,
            start_altitude_m=1000.0,
            end_x_m=100.0,
            end_y_m=0.0,
            end_altitude_m=1000.0,
        )
        northbound_route = RouteState(
            start_x_m=0.0,
            start_y_m=0.0,
            start_altitude_m=1000.0,
            end_x_m=0.0,
            end_y_m=100.0,
            end_altitude_m=1000.0,
        )

        self.assertAlmostEqual(
            SimulationController._cross_track_error(_aircraft_state("A01", 0.0, -10.0, 1000.0), eastbound_route) or 0.0,
            10.0,
        )
        self.assertAlmostEqual(
            SimulationController._cross_track_error(_aircraft_state("A01", 10.0, 0.0, 1000.0), northbound_route) or 0.0,
            10.0,
        )

    def test_arc_route_cross_track_error_positive_to_track_right_side(self) -> None:
        """圆弧段侧偏也应保持右侧向为正，避免转弯段和直线段反号。"""

        left_arc = waypoint_inputs_to_waylines(_build_leader_route(
            {
                "route": {
                    "speed_mps": 20.0,
                    "waypoints": [
                        {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 0.0},
                        {"x_m": 2000.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 400.0},
                        {"x_m": 2000.0, "y_m": 2000.0, "altitude_m": 1000.0, "R": 0.0},
                    ],
                }
            }
        ))[1]
        right_arc = waypoint_inputs_to_waylines(_build_leader_route(
            {
                "route": {
                    "speed_mps": 20.0,
                    "waypoints": [
                        {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 0.0},
                        {"x_m": 2000.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 400.0},
                        {"x_m": 2000.0, "y_m": -2000.0, "altitude_m": 1000.0, "R": 0.0},
                    ],
                }
            }
        ))[1]
        left_route = _route_state_from_wayline(left_arc)
        right_route = _route_state_from_wayline(right_arc)
        left_outside = _aircraft_state(
            "A01",
            left_arc.start.center.east + (arc_radius(left_arc) + 10.0) / math.sqrt(2.0),
            left_arc.start.center.north - (arc_radius(left_arc) + 10.0) / math.sqrt(2.0),
            1000.0,
        )
        left_inside = _aircraft_state(
            "A01",
            left_arc.start.center.east + (arc_radius(left_arc) - 10.0) / math.sqrt(2.0),
            left_arc.start.center.north - (arc_radius(left_arc) - 10.0) / math.sqrt(2.0),
            1000.0,
        )
        right_inside = _aircraft_state(
            "A01",
            right_arc.start.center.east + (arc_radius(right_arc) - 10.0) / math.sqrt(2.0),
            right_arc.start.center.north + (arc_radius(right_arc) - 10.0) / math.sqrt(2.0),
            1000.0,
        )
        right_outside = _aircraft_state(
            "A01",
            right_arc.start.center.east + (arc_radius(right_arc) + 10.0) / math.sqrt(2.0),
            right_arc.start.center.north + (arc_radius(right_arc) + 10.0) / math.sqrt(2.0),
            1000.0,
        )

        self.assertAlmostEqual(SimulationController._cross_track_error(left_outside, left_route) or 0.0, 10.0)
        self.assertAlmostEqual(SimulationController._cross_track_error(left_inside, left_route) or 0.0, -10.0)
        self.assertAlmostEqual(SimulationController._cross_track_error(right_inside, right_route) or 0.0, 10.0)
        self.assertAlmostEqual(SimulationController._cross_track_error(right_outside, right_route) or 0.0, -10.0)

    def test_display_route_keeps_original_segments_without_arc(self) -> None:
        """显示用航线(insert_arcs=False)应保留 base 原始航点折线、不插圆弧(界面只画原始航段)。"""
        config = {
            "route": {
                "speed_mps": 20.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 0.0},
                    {"x_m": 2000.0, "y_m": 0.0, "altitude_m": 1000.0, "R": 400.0},
                    {"x_m": 2000.0, "y_m": 2000.0, "altitude_m": 1000.0, "R": 0.0},
                ],
            }
        }
        # 跟踪航线插圆弧(3 段)，显示航线不插(原始 2 直线段、含尖角)。
        self.assertEqual(len(waypoint_inputs_to_waylines(_build_leader_route(config))), 3)
        display = waypoint_inputs_to_waylines(_build_leader_route(config, insert_arcs=False))
        self.assertEqual(len(display), 2)
        self.assertTrue(all(line.start.turnSign == 0.0 for line in display))
        self.assertAlmostEqual(display[0].end.pos.east, 2000.0)
        self.assertAlmostEqual(display[0].end.pos.north, 0.0)
        self.assertAlmostEqual(display[1].end.pos.north, 2000.0)

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

        self.assertEqual(comm_init.formPat, ["default"])
        self.assertAlmostEqual(slots["A01"].x, 0.0)
        self.assertAlmostEqual(slots["A01"].y, 0.0)
        self.assertAlmostEqual(slots["A02"].x, -54.0)
        self.assertAlmostEqual(slots["A02"].y, 0.0)
        self.assertAlmostEqual(slots["A02"].z, -58.0)
        self.assertAlmostEqual(slots["A03"].x, -54.0)
        self.assertAlmostEqual(slots["A03"].y, 0.0)
        self.assertAlmostEqual(slots["A03"].z, 58.0)

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
                "coordinate_system": "x_forward_y_up_z_right",
                "slots": [
                    {"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                    {"node_id": "A02", "x_m": -70.0, "y_m": 5.0, "z_m": -40.0},
                    {"node_id": "A03", "x_m": -70.0, "y_m": -5.0, "z_m": 40.0},
                ],
            }
        }

        comm_init = _build_formation_comm_init(nodes, [], config)
        slots = {slot.id: slot for slot in comm_init.formPos[0]}

        self.assertEqual(comm_init.formPat, ["TRIANGLE"])
        self.assertAlmostEqual(slots["A02"].x, -70.0)
        self.assertAlmostEqual(slots["A02"].y, 5.0)
        self.assertAlmostEqual(slots["A02"].z, -40.0)
        self.assertAlmostEqual(slots["A03"].x, -70.0)
        self.assertAlmostEqual(slots["A03"].y, -5.0)
        self.assertAlmostEqual(slots["A03"].z, 40.0)

    def test_configured_formation_slots_require_axis_declaration(self) -> None:
        """显式配置槽位时必须声明轴序，避免旧 y 侧向配置被按新语义静默接收。"""
        nodes = [
            {"node_id": "A01", "role": "leader"},
            {"node_id": "A02", "role": "wingman"},
        ]
        config = {
            "formation": {
                "pattern": "TRIANGLE",
                "slots": [
                    {"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                    {"node_id": "A02", "x_m": -30.0, "y_m": 20.0, "z_m": 0.0},
                ],
            }
        }

        with self.assertRaisesRegex(ValueError, "coordinate_system"):
            _build_formation_comm_init(nodes, [], config)

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

    def test_vel_cmd_limit_absent_block_means_unbounded(self) -> None:
        """缺省 control 块时前向/垂向速度指令均不限(±inf)。"""
        limit = _build_vel_cmd_limit({})
        self.assertEqual(limit.forwardMin, float("-inf"))
        self.assertEqual(limit.forwardMax, float("inf"))
        self.assertEqual(limit.verticalMin, float("-inf"))
        self.assertEqual(limit.verticalMax, float("inf"))

    def test_vel_cmd_limit_parsed_from_config(self) -> None:
        """control.velocity_command_limits 四个速度上下限被正确解析。"""
        limit = _build_vel_cmd_limit(
            {
                "control": {
                    "velocity_command_limits": {
                        "forward_min_mps": 1.0,
                        "forward_max_mps": 40.0,
                        "vertical_min_mps": -3.0,
                        "vertical_max_mps": 3.0,
                    }
                }
            }
        )
        self.assertEqual(
            (limit.forwardMin, limit.forwardMax, limit.verticalMin, limit.verticalMax),
            (1.0, 40.0, -3.0, 3.0),
        )

    def test_vel_cmd_limit_rejects_inverted_bounds(self) -> None:
        """下限>上限的非法限幅在解析阶段报错。"""
        with self.assertRaisesRegex(ValueError, "forward_min_mps"):
            _build_vel_cmd_limit({"control": {"velocity_command_limits": {"forward_min_mps": 50.0, "forward_max_mps": 10.0}}})
        with self.assertRaisesRegex(ValueError, "vertical_min_mps"):
            _build_vel_cmd_limit({"control": {"velocity_command_limits": {"vertical_min_mps": 3.0, "vertical_max_mps": -3.0}}})

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
        """Leader should smoothly cut toward the route instead of weaving forward/backward.

        本 fixture 长机初始横偏达 260m。1.2 引入"按侧偏的航迹角变限幅"后，大侧偏下航迹角被限到
        最多 90°(垂直切入)、不越 90°，故东向不回退、平滑切入(不再有 1.1 中间态那种越过正南的向后蛇行)。
        """
        controller = SimulationController()
        controller.load_config(str(Path(__file__).resolve().parent / "fixtures" / "test.json"))
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

            self.assertAlmostEqual(leader._pos_track._lateral_cascade._cfg.dt, 0.2)
            self.assertAlmostEqual(follower._pos_track._lateral_cascade._cfg.dt, 0.2)
            controller.close()

    def test_default_algorithm_pid_period_matches_design_contract(self) -> None:
        """默认配置下编队 PID 控制周期应为 0.005s * 10 = 0.05s。"""

        with tempfile.TemporaryDirectory() as tmp:
            config_path = _write_config(Path(tmp), duration_s=0.02)
            controller = SimulationController()
            controller.load_config(str(config_path))

            leader = controller._node_algorithms["A01"]._entity
            follower = controller._node_algorithms["A02"]._entity

            self.assertAlmostEqual(leader._pos_track._lateral_cascade._cfg.dt, 0.05)
            self.assertAlmostEqual(follower._pos_track._lateral_cascade._cfg.dt, 0.05)
            self.assertAlmostEqual(leader._pos_track._lateral_cascade._cfg.rollMaxRad, math.radians(40.0))
            self.assertAlmostEqual(follower._pos_track._lateral_cascade._cfg.rollMaxRad, math.radians(40.0))
            controller.close()

    def test_realtime_logging_uses_sim_time_10_hz(self) -> None:
        """实时 tick 路径下日志应按仿真时间 10Hz 记录，避免不同倍频采样点不一致。"""

        with tempfile.TemporaryDirectory() as tmp:
            config = {
                "duration_s": 1.0,
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
            fake_wall_s = 1.0

            def fake_clock() -> float:
                return fake_wall_s

            with patch("src.runner.sim_control.time.monotonic", fake_clock):
                with controller._lock:
                    controller._run_state = "RUNNING"
                    controller._control_report = controller._derive_control_report_unlocked()
                    for index in range(100):
                        # 模拟 9x 左右播放：100 个 5ms 仿真步只消耗约 55ms 墙钟。
                        fake_wall_s = 1.0 + index * (0.005 / 9.0)
                        controller._tick_unlocked()

            logged_times = [round(snapshot.time_s, 3) for snapshot in controller._logger.snapshots]

            self.assertEqual(logged_times, [0.1, 0.2, 0.3, 0.4, 0.5])
            controller.close()

    def test_realtime_snapshot_generation_is_limited_to_display_rate(self) -> None:
        """非日志快照应按墙钟显示频率限流，避免高倍频时随 tick 高频构造。"""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=1.0, step_s=0.005)))
            controller._logger._file_logging_disabled = True
            fake_wall_s = 1.0
            snapshot_count = 0
            original_make_snapshot = controller._make_snapshot_unlocked

            def fake_clock() -> float:
                return fake_wall_s

            def counted_make_snapshot() -> SimulationSnapshot:
                nonlocal snapshot_count
                snapshot_count += 1
                return original_make_snapshot()

            controller._make_snapshot_unlocked = counted_make_snapshot  # type: ignore[method-assign]
            with patch("src.runner.sim_control.time.monotonic", fake_clock):
                with controller._lock:
                    controller._run_state = "RUNNING"
                    controller._control_report = controller._derive_control_report_unlocked()
                    controller._next_log_sample_time_s = 999.0
                    for index in range(50):
                        fake_wall_s = 1.0 + index * 0.01
                        controller._tick_unlocked()

            self.assertGreaterEqual(snapshot_count, 4)
            self.assertLessEqual(snapshot_count, 6)
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

    def test_set_duration_updates_snapshot_and_finish_boundary(self) -> None:
        """界面修改仿真时长后，控制器应使用新时长作为停止边界。"""
        controller = SimulationController()
        controller.run_until_complete({"duration_s": 0.005, "step_s": 0.005})
        controller.reset()

        result = controller.set_duration(0.01)
        controller.step(2)
        snapshot = controller.get_snapshot()

        self.assertEqual(result.code, "OK")
        self.assertAlmostEqual(snapshot.duration_s, 0.01)
        self.assertEqual(snapshot.run_state, "FINISHED")
        self.assertAlmostEqual(snapshot.time_s, 0.01)
        controller.close()

    def test_set_duration_rejects_time_before_current_snapshot(self) -> None:
        """暂停态修改总时长不能回退仿真时间，否则快照时间会和模型状态不一致。"""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=1.0, step_s=0.005)))
            controller.step(100)
            before = controller.get_snapshot()

            result = controller.set_duration(0.2)
            after = controller.get_snapshot()

            self.assertEqual(result.code, "ERR_INVALID_ARGUMENT")
            self.assertAlmostEqual(before.time_s, 0.5)
            self.assertAlmostEqual(after.time_s, before.time_s)
            self.assertAlmostEqual(after.duration_s, before.duration_s)
            self.assertEqual(after.run_state, "PAUSED")
            self.assertAlmostEqual(after.nodes[0].x_m, before.nodes[0].x_m)
            controller.close()

    def test_timed_data_logger_records_snapshots_at_10_hz(self) -> None:
        """关键数据记录应固定为仿真时间 10Hz，而不是随播放倍率改变。"""
        controller = SimulationController()

        result = controller.run_until_complete(
            {
                "duration_s": 0.1,
                "step_s": 0.01,
                "nodes": [{"node_id": "A01"}],
                "links": [],
            }
        )
        logged_times = [round(snapshot.time_s, 6) for snapshot in controller._logger.snapshots]

        self.assertEqual(result.code, "OK")
        self.assertEqual(logged_times, [0.1])
        controller.close()

    def test_timed_data_logger_persists_snapshot_files(self) -> None:
        """关键数据日志应落盘到 logs/run-*/snapshots.jsonl，便于仿真后查找。"""
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path.cwd()
            try:
                os.chdir(tmp)
                controller = SimulationController()
                result = controller.run_until_complete(
                    {
                        "duration_s": 0.1,
                        "step_s": 0.01,
                        "nodes": [{"node_id": "A01"}],
                        "links": [],
                    }
                )
                controller.close()
            finally:
                os.chdir(cwd)

            run_dirs = list((Path(tmp) / "logs").glob("run-*"))
            self.assertEqual(result.code, "OK")
            self.assertEqual(len(run_dirs), 1)
            self.assertTrue((run_dirs[0] / "config.json").is_file())
            self.assertTrue((run_dirs[0] / "events.jsonl").is_file())
            snapshot_lines = (run_dirs[0] / "snapshots.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual([round(json.loads(line)["time_s"], 6) for line in snapshot_lines], [0.1])

    def test_snapshot_log_contains_control_command_and_errors(self) -> None:
        """关键数据日志应包含控制目标指令和位置/速度误差，供后处理与 UI 复用。"""
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path.cwd()
            try:
                os.chdir(tmp)
                controller = SimulationController()
                result = controller.run_until_complete(
                    {
                        "duration_s": 0.1,
                        "step_s": 0.01,
                        "nodes": [{"node_id": "A01"}],
                        "links": [],
                    }
                )
                controller.close()
            finally:
                os.chdir(cwd)

            run_dirs = list((Path(tmp) / "logs").glob("run-*"))
            payload = json.loads((run_dirs[0] / "snapshots.jsonl").read_text(encoding="utf-8").splitlines()[0])
            node = payload["nodes"][0]

            self.assertEqual(result.code, "OK")
            for key in (
                "cmd_pos_east_m",
                "cmd_pos_north_m",
                "cmd_pos_h_m",
                "cmd_vel_east_mps",
                "cmd_vel_north_mps",
                "cmd_vel_up_mps",
                "pos_err_east_m",
                "pos_err_north_m",
                "pos_err_h_m",
                "vel_err_east_mps",
                "vel_err_north_mps",
                "vel_err_up_mps",
                "track_pos_err_x_m",
                "track_pos_err_y_m",
                "track_pos_err_z_m",
                "track_vel_err_x_mps",
                "track_vel_err_y_mps",
                "track_vel_err_z_mps",
            ):
                self.assertIn(key, node)
                self.assertIsInstance(node[key], (int, float))

    def test_data_logger_opens_files_only_after_first_tick(self) -> None:
        """加载配置和 reset 不应创建空 run 目录，首次推进仿真时才创建日志目录。"""
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path.cwd()
            try:
                os.chdir(tmp)
                config_path = _write_config(Path(tmp), duration_s=0.1, step_s=0.01)
                controller = SimulationController()
                self.assertEqual(controller.load_config(str(config_path)).code, "OK")
                self.assertFalse((Path(tmp) / "logs").exists())

                self.assertEqual(controller.reset().code, "OK")
                self.assertFalse((Path(tmp) / "logs").exists())

                self.assertEqual(controller.step().code, "OK")
                self.assertEqual(len(list((Path(tmp) / "logs").glob("run-*"))), 1)
                controller.close()
            finally:
                os.chdir(cwd)

    def test_snapshot_log_write_failure_warns_and_keeps_stepping(self) -> None:
        """快照文件写失败只能降级为 WARN，不能让仿真 tick 返回 ERR_TICK_FAILED。"""

        class BrokenSnapshotFile:
            def write(self, _text: str) -> int:
                raise OSError("disk unavailable")

            def close(self) -> None:
                return None

            def flush(self) -> None:
                return None

        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path.cwd()
            try:
                os.chdir(tmp)
                controller = SimulationController()
                config_path = _write_config(Path(tmp), duration_s=0.1, step_s=0.01)
                self.assertEqual(controller.load_config(str(config_path)).code, "OK")
                self.assertEqual(controller.step(9).code, "OK")
                assert controller._logger._snapshot_file is not None
                controller._logger._snapshot_file.close()
                controller._logger._snapshot_file = BrokenSnapshotFile()

                result = controller.step()
                snapshot = controller.get_snapshot()
                warn_events = controller.get_recent_events(min_level="WARN")

                self.assertEqual(result.code, "OK")
                self.assertAlmostEqual(snapshot.time_s, 0.1)
                self.assertTrue(any("snapshot log failed: disk unavailable" in event.message for event in warn_events))
                self.assertFalse(controller._logger.opened)
                self.assertTrue(controller._logger._file_logging_disabled)
                controller.close()
            finally:
                os.chdir(cwd)

    def test_data_logger_rounds_persisted_snapshot_precision(self) -> None:
        """关键数据日志落盘时按字段语义四舍五入，内存快照保留原始精度。"""
        with tempfile.TemporaryDirectory() as tmp:
            snapshot = SimulationSnapshot(
                time_s=1.23456,
                duration_s=2.34567,
                step_s=0.00555,
                run_state="RUNNING",
                control_report="保持",
                nodes=[
                    NodeState(
                        node_id="A01",
                        role="leader",
                        health="normal",
                        x_m=1.235,
                        y_m=2.345,
                        altitude_m=1234.565,
                        psi_v_deg=12.345,
                        theta_deg=1.235,
                        speed_mps=40.555,
                        vx_mps=1.235,
                        vy_mps=2.345,
                        vz_mps=-3.455,
                        nx=0.12345,
                        nz=1.23456,
                        phi_deg=6.785,
                        psi_dot_deg_s=3.456,
                        cmd_pos_east_m=10.005,
                        cmd_pos_north_m=20.005,
                        cmd_pos_h_m=1235.005,
                        cmd_vel_east_mps=4.445,
                        cmd_vel_north_mps=5.555,
                        cmd_vel_up_mps=6.665,
                        pos_err_east_m=7.775,
                        pos_err_north_m=8.885,
                        pos_err_h_m=9.995,
                        vel_err_east_mps=1.115,
                        vel_err_north_mps=2.225,
                        vel_err_up_mps=3.335,
                        track_pos_err_x_m=4.445,
                        track_pos_err_y_m=5.555,
                        track_pos_err_z_m=6.665,
                        track_vel_err_x_mps=7.775,
                        track_vel_err_y_mps=8.885,
                        track_vel_err_z_mps=9.995,
                        cross_track_error_m=7.895,
                        distance_to_go_m=99.995,
                    )
                ],
                links=[LinkState("A01-A02", "duplex", 12.3456, 0.012345, "normal")],
                route=RouteState(0.005, 1.235, 1000.555, 2.345, 3.455, 1001.565),
            )
            cwd = Path.cwd()
            try:
                os.chdir(tmp)
                logger = _DataLogger()
                logger.open("run-precision", {})
                logger.write_snapshot(snapshot)
                logger.close()
            finally:
                os.chdir(cwd)

            payload = json.loads((Path(tmp) / "logs" / "run-precision" / "snapshots.jsonl").read_text(encoding="utf-8"))
            node = payload["nodes"][0]

            self.assertEqual(payload["time_s"], 1.235)
            self.assertEqual(payload["duration_s"], 2.346)
            self.assertNotIn("step_s", payload)
            self.assertNotIn("route", payload)
            self.assertNotIn("route_segments", payload)
            self.assertEqual(node["x_m"], 1.24)
            self.assertEqual(node["altitude_m"], 1234.57)
            self.assertEqual(node["speed_mps"], 40.56)
            self.assertEqual(node["vx_mps"], 1.24)
            self.assertEqual(node["nx"], 0.1235)
            self.assertEqual(node["nz"], 1.2346)
            self.assertEqual(node["psi_v_deg"], 12.35)
            self.assertEqual(node["phi_deg"], 6.79)
            self.assertEqual(node["psi_dot_deg_s"], 3.46)
            self.assertEqual(node["cmd_pos_east_m"], 10.01)
            self.assertEqual(node["cmd_vel_up_mps"], 6.67)
            self.assertEqual(node["track_vel_err_z_mps"], 10.0)
            self.assertEqual(node["cross_track_error_m"], 7.9)
            self.assertEqual(logger.snapshots[0].time_s, 1.23456)
            self.assertEqual(_DataLogger._round_log_value("ax_mps2", 1.2345), 1.235)

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
            self.assertEqual(controller.set_playback_rate(50.0).code, "OK")
            self.assertEqual(controller.set_playback_rate(50.1).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual(controller.step(0).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual(controller.inject_disturbance({"type": "bad"}).code, "ERR_INVALID_ARGUMENT")
            self.assertEqual([event.level for event in controller.get_recent_events(limit=1, min_level="WARN")], ["WARN"])
            controller.close()

    def test_playback_rate_property_reflects_config_and_updates(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_config(Path(tmp))
            controller = SimulationController()

            self.assertAlmostEqual(controller.playback_rate, 1.0)
            self.assertEqual(controller.load_config(str(path)).code, "OK")
            self.assertAlmostEqual(controller.playback_rate, 10.0)
            self.assertEqual(controller.set_playback_rate(50.0).code, "OK")
            self.assertAlmostEqual(controller.playback_rate, 50.0)
            controller.close()

    def test_reset_preserves_runtime_playback_rate_update(self) -> None:
        """运行期调整播放倍率后 reset 不应退回配置文件默认值。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_config(Path(tmp), playback_rate=1.0)
            controller = SimulationController()

            self.assertEqual(controller.load_config(str(path)).code, "OK")
            self.assertEqual(controller.set_playback_rate(9.0).code, "OK")
            self.assertAlmostEqual(controller.playback_rate, 9.0)

            self.assertEqual(controller.reset().code, "OK")

            self.assertAlmostEqual(controller.playback_rate, 9.0)
            controller.close()

    def test_run_loop_batches_ticks_after_wall_clock_delay(self) -> None:
        """后台调度晚醒后应按累计墙钟时间批量补拍，而不是逐拍亚毫秒睡眠。"""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=0.1, step_s=0.005)))
            controller.set_playback_rate(20.0)
            # 本测试只验证调度节奏，关闭文件落盘避免临时日志影响断言。
            controller._logger._file_logging_disabled = True
            fake_now_s = 0.0
            sleeps: list[float] = []

            def fake_clock() -> float:
                return fake_now_s

            def fake_sleep(seconds: float) -> None:
                nonlocal fake_now_s
                sleeps.append(seconds)
                fake_now_s += seconds

            with patch("src.runner.sim_control.time.perf_counter", fake_clock), patch(
                "src.runner.sim_control.time.monotonic",
                fake_clock,
            ), patch("src.runner.sim_control.time.sleep", fake_sleep):
                with controller._lock:
                    controller._run_state = "RUNNING"
                    controller._control_report = controller._derive_control_report_unlocked()
                controller._run_loop()

            snapshot = controller.get_snapshot()
            self.assertEqual(snapshot.run_state, "FINISHED")
            self.assertAlmostEqual(snapshot.time_s, 0.1)
            self.assertLess(len(sleeps), 10)
            self.assertGreater(max(sleeps), 0.001)
            controller.close()

    def test_run_loop_reports_busy_ratio_as_cpu_utilization(self) -> None:
        """CPU 利用率应按统计周期内非 sleep 时间 / 墙钟时间计算。"""
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            controller.load_config(str(_write_config(Path(tmp), duration_s=0.05, step_s=0.005)))
            controller.set_playback_rate(1.0)
            controller._logger._file_logging_disabled = True
            fake_now_s = 0.0

            def fake_clock() -> float:
                return fake_now_s

            def fake_sleep(_seconds: float) -> None:
                nonlocal fake_now_s
                fake_now_s += 0.001

            def fake_tick(*, force_snapshot: bool = False) -> SimulationSnapshot:  # noqa: ARG001
                nonlocal fake_now_s
                fake_now_s += 0.004
                controller._time_s = min(controller._duration_s, controller._time_s + controller._step_s)
                if controller._time_s >= controller._duration_s:
                    controller._run_state = "FINISHED"
                controller._latest_snapshot = controller._make_snapshot_unlocked()
                return controller._latest_snapshot

            controller._tick_unlocked = fake_tick  # type: ignore[method-assign]
            with patch("src.runner.sim_control._CPU_UTILIZATION_SAMPLE_PERIOD_S", 0.01), patch(
                "src.runner.sim_control.time.perf_counter",
                fake_clock,
            ), patch("src.runner.sim_control.time.monotonic", fake_clock), patch(
                "src.runner.sim_control.time.sleep",
                fake_sleep,
            ):
                with controller._lock:
                    controller._run_state = "RUNNING"
                    controller._control_report = controller._derive_control_report_unlocked()
                controller._run_loop()

            snapshot = controller.get_snapshot()
            self.assertEqual(snapshot.run_state, "FINISHED")
            self.assertGreaterEqual(snapshot.cpu_utilization, 0.75)
            self.assertLessEqual(snapshot.cpu_utilization, 0.9)
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


class NodeAlgorithmResetTests(unittest.TestCase):
    """验证 _NodeAlgorithm.reset() 保留构造期僚机冷启动预置。"""

    def _make_comm_init(self) -> FormCommInitS:
        return FormCommInitS()

    def _make_leader_state(self) -> MotionProfS:
        s = MotionProfS()
        s.pos.east = 100.0
        s.pos.north = 200.0
        s.pos.h = 500.0
        return s

    def _make_leader_route(self) -> list[WayPointInputS]:
        return [
            WayPointInputS(idx=0, pos=PosInEarthS(east=0.0, north=0.0, h=500.0), vdCmd=20.0),
            WayPointInputS(idx=1, pos=PosInEarthS(east=1000.0, north=0.0, h=500.0), vdCmd=20.0),
        ]

    def test_reset_preserves_cold_start_preset_for_wingman(self) -> None:
        """reset() 后僚机 cmd.stage/pattern 和 leaderState 应与构造后一致。"""
        initial_leader = self._make_leader_state()
        node = _NodeAlgorithm(
            node_id="W01",
            role="wingman",
            comm_init=self._make_comm_init(),
            initial_leader_state=initial_leader,
            leader_route=self._make_leader_route(),
            control_period_s=0.02,
        )

        # 构造后预置正确
        self.assertEqual(node._entity.cxt.cmd.stage, FormStageE.HOLD)
        self.assertEqual(node._entity.cxt.cmd.pattern, 0)
        self.assertAlmostEqual(node._entity.cxt.leaderState.pos.east, 100.0)

        # reset 后预置应被恢复
        node.reset()

        self.assertEqual(node._entity.cxt.cmd.stage, FormStageE.HOLD)
        self.assertEqual(node._entity.cxt.cmd.pattern, 0)
        self.assertAlmostEqual(node._entity.cxt.leaderState.pos.east, 100.0)

    def test_reset_clears_rally_completed_flag(self) -> None:
        """reset() 后 _rally_completed 应归 False，允许重新触发集结完成流程。"""
        from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
        from src.algorithm.context.leaf_types import PosInEarthS as P

        rally_cfg = RallyTaskInitS(
            expectedFollowerIds=[],
            dt_s=0.02,
            targetPattern=0,
        )
        rally_route = [
            WayPointInputS(idx=0, pos=P(east=0.0, north=0.0, h=500.0), vdCmd=20.0),
            WayPointInputS(idx=1, pos=P(east=100.0, north=0.0, h=500.0), vdCmd=20.0),
        ]
        mission_route = [
            WayPointInputS(idx=0, pos=P(east=100.0, north=0.0, h=500.0), vdCmd=20.0),
            WayPointInputS(idx=1, pos=P(east=200.0, north=0.0, h=500.0), vdCmd=20.0),
        ]
        node = _NodeAlgorithm(
            node_id="R01",
            role="rally_leader",
            comm_init=self._make_comm_init(),
            initial_leader_state=None,
            leader_route=mission_route,
            control_period_s=0.02,
            rally_route=rally_route,
            rally_cfg=rally_cfg,
        )

        # 模拟集结完成后锁存
        node._rally_completed = True
        node._remote_stage = FormStageE.HOLD

        node.reset()

        self.assertFalse(node._rally_completed)
        self.assertEqual(node._remote_stage, FormStageE.RALLY)


if __name__ == "__main__":
    unittest.main()
