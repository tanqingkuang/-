"""集结场景仿真控制器低层测试。"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from src.algorithm.context.leaf_types import FormStageE, FormationAnalysisS
from tests.llt._geo_route import geodetic_config
from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity
from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity
from src.runner.sim_control import (
    SimulationController,
    _build_formation_comm_init,
    _build_rally_route,
    _build_rally_task_init,
)


def _rally_config() -> dict[str, object]:
    """构造最小可运行的三机集结配置（航线为 ENU，供直接调用 _build_* 的单元测试）。

    注意：经 load_config 加载的用例由 _write_json 转成经纬航线（产品约定）。
    """

    return {
        "duration_s": 0.05,
        "step_s": 0.005,
        "algorithm_decimation": 1,
        "playback_rate": 1.0,
        "route": {
            # route（mission_route）起点必须等于 rally_route 起点 A=(0,0)（见 sim_control_modules.py
            # 的连续性校验），不是旧版语义里的 rally_route 终点。
            "speed_mps": 20.0,
            "waypoints": [
                {"x_m": 0.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
                {"x_m": 200.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
            ],
        },
        "rally_route": {
            "speed_mps": 20.0,
            "waypoints": [
                {"x_m": 0.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
                {"x_m": 100.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
            ],
        },
        "rally_cfg": {
            "loose_scale": 3.0,
            "convergence_radius_m": 5.0,
            "stable_hold_s": 0.1,
            "compress_time_s": 0.2,
            "tight_radius_m": 2.0,
            "stale_timeout_s": 1.0,
        },
        "formation": {
            "coordinate_system": "x_forward_y_up_z_right",
            "formations": [
                {
                    "name": "TRIANGLE",
                    "slots": [
                        {"node_id": "R01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                        {"node_id": "R02", "x_m": -10.0, "y_m": 0.0, "z_m": -5.0},
                        {"node_id": "R03", "x_m": -10.0, "y_m": 0.0, "z_m": 5.0},
                    ],
                }
            ],
        },
        "nodes": [
            {"node_id": "R01", "role": "rally_leader", "x_m": 0.0, "y_m": 0.0, "altitude_m": 500.0, "speed_mps": 20.0},
            {
                "node_id": "R02",
                "role": "rally_follower",
                "x_m": -50.0,
                "y_m": 20.0,
                "altitude_m": 500.0,
                "speed_mps": 20.0,
            },
            {
                "node_id": "R03",
                "role": "rally_follower",
                "x_m": -50.0,
                "y_m": -20.0,
                "altitude_m": 500.0,
                "speed_mps": 20.0,
            },
        ],
        "links": [
            {"link_id": "R01-R02", "direction": "duplex", "latency_ms": 1.0, "loss_rate": 0.0},
            {"link_id": "R01-R03", "direction": "duplex", "latency_ms": 1.0, "loss_rate": 0.0},
        ],
    }


def _write_json(directory: Path, config: dict[str, object]) -> Path:
    """把配置写入临时 JSON 文件。注意：航线转经纬后写盘，符合"JSON 只支持经纬航线"约定。"""

    config = json.loads(json.dumps(config))
    formation = config.get("formation")
    if isinstance(formation, dict):
        formations = formation.pop("formations", None)
        if isinstance(formations, list):
            formation_dir = directory / "element" / "formations"
            formation_dir.mkdir(parents=True, exist_ok=True)
            formation_files = []
            for index, item in enumerate(formations):
                formation_path = formation_dir / f"rally_formation_{index}.json"
                formation_path.write_text(json.dumps(item, ensure_ascii=False), encoding="utf-8")
                formation_files.append(str(formation_path.relative_to(directory)).replace("\\", "/"))
            formation["formation_files"] = formation_files
    path = directory / "rally_case.json"
    path.write_text(json.dumps(geodetic_config(config)), encoding="utf-8")
    return path


class SimControlRallyTests(unittest.TestCase):
    """验证控制器对集结场景的配置解析、实体装配和快照透传。"""

    def test_build_rally_route_from_waypoints(self) -> None:
        """验证 rally_route.waypoints 被解析为连续航段。"""

        route = _build_rally_route(_rally_config())

        self.assertIsNotNone(route)
        assert route is not None
        self.assertEqual(len(route), 2)
        self.assertAlmostEqual(route[0].pos.east, 0.0)
        self.assertAlmostEqual(route[1].pos.east, 100.0)
        self.assertAlmostEqual(route[0].vdCmd, 20.0)

    def test_build_rally_task_init_collects_expected_followers_and_period(self) -> None:
        """验证 rally_cfg 生成 RallyTaskInitS，并按角色收集期望僚机。"""

        config = _rally_config()
        nodes = list(config["nodes"])  # type: ignore[arg-type]
        task_init = _build_rally_task_init(config, 0.025, nodes)

        self.assertIsNotNone(task_init)
        assert task_init is not None
        self.assertEqual(task_init.expectedFollowerIds, ["R02", "R03"])
        self.assertEqual(task_init.targetPattern, 0)
        self.assertAlmostEqual(task_init.dt_s, 0.025)
        self.assertAlmostEqual(task_init.tightRadius_m, 2.0)

    def test_formation_comm_init_accepts_rally_roles_in_slots(self) -> None:
        """验证集结角色同样使用展开后的队形槽位注入通信初始化结构。"""

        config = _rally_config()
        nodes = list(config["nodes"])  # type: ignore[arg-type]
        links = list(config["links"])  # type: ignore[arg-type]

        comm = _build_formation_comm_init(nodes, links, config)
        slots = {slot.id: slot for slot in comm.formPos[0]}

        self.assertEqual(comm.formPat, ["TRIANGLE"])
        self.assertEqual([link.startId for link in comm.netWork], ["R01", "R01"])
        self.assertAlmostEqual(slots["R02"].x, -10.0)
        self.assertAlmostEqual(slots["R03"].z, 5.0)

    def test_load_config_instantiates_rally_entities_and_initial_route(self) -> None:
        """验证控制器按 rally_leader/rally_follower 角色装配集结实体。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            result = controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertIsInstance(controller._node_algorithms["R01"]._entity, RallyLeaderEntity)
            self.assertIsInstance(controller._node_algorithms["R02"]._entity, RallyFollowerEntity)
            self.assertIsInstance(controller._node_algorithms["R03"]._entity, RallyFollowerEntity)
            self.assertIsNotNone(snapshot.route)
            assert snapshot.route is not None
            self.assertAlmostEqual(snapshot.route.start_x_m, 0.0, places=3)
            self.assertAlmostEqual(snapshot.route.end_x_m, 200.0, places=3)

    def test_rally_scenario_supports_runtime_formation_switch_entry(self) -> None:
        """验证集结场景复用现有队形重构入口，运行期切换 rally_leader 的目标队形索引。"""

        config = _rally_config()
        config["formation"]["formations"].append(  # type: ignore[index]
            {
                "name": "LINE",
                "slots": [
                    {"node_id": "R01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                    {"node_id": "R02", "x_m": -20.0, "y_m": 0.0, "z_m": 0.0},
                    {"node_id": "R03", "x_m": -40.0, "y_m": 0.0, "z_m": 0.0},
                ],
            }
        )
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            load_result = controller.load_config(str(_write_json(Path(tmp), config)))

            switch_result = controller.switch_formation(1)
            task = controller._node_algorithms["R01"]._entity._task

            self.assertEqual(load_result.code, "OK")
            self.assertEqual(switch_result.code, "OK")
            self.assertEqual(controller.get_formation_index(), 1)
            self.assertEqual(task._target_pattern, 1)

    def test_rally_cfg_approach_speed_is_injected_into_followers(self) -> None:
        """验证 rally_cfg.approach_speed_mps 会注入僚机 RallyJoinPos。"""

        config = _rally_config()
        config["rally_cfg"]["approach_speed_mps"] = 16.0  # type: ignore[index]
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            result = controller.load_config(str(_write_json(Path(tmp), config)))
            follower = controller._node_algorithms["R02"]._entity

            self.assertEqual(result.code, "OK")
            self.assertAlmostEqual(follower._rally_join._approach_speed, 16.0)

    def test_vel_cmd_limit_vertical_injected_into_follower_rally_join(self) -> None:
        """velCmdLimit.vertical 应注入僚机 RallyJoinPos 替换硬编码 ±3.0。"""
        config = _rally_config()
        config.setdefault("control", {})["velocity_command_limits"] = {  # type: ignore[index]
            "forward_min_mps": 14.0,
            "forward_max_mps": 25.0,
            "vertical_min_mps": -2.0,
            "vertical_max_mps": 2.5,
        }
        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            result = controller.load_config(str(_write_json(Path(tmp), config)))
            follower = controller._node_algorithms["R02"]._entity

            self.assertEqual(result.code, "OK")
            self.assertAlmostEqual(follower._rally_join._v_up_min, -2.0)
            self.assertAlmostEqual(follower._rally_join._v_up_max, 2.5)

    def test_manual_step_runs_rally_stage_and_sends_leader_broadcast(self) -> None:
        """验证手动步进后集结长机进入 RALLY 并通过通信信道投递长机广播。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))

            result = controller.step(1)
            stage = controller._node_algorithms["R01"].current_stage()
            inbox = controller._comm.read_inbox("R02")

            self.assertEqual(result.code, "OK")
            self.assertEqual(stage, FormStageE.RALLY)
            self.assertTrue(any(msg.topic == "formation.leader" for msg in inbox))

    def test_snapshot_exposes_latched_rally_analysis(self) -> None:
        """验证控制器快照透传集结完成后锁存的编队分析。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            analysis = FormationAnalysisS(posErrMax_m=1.2, posErrRms_m=0.8, inPositionCount=2, totalCount=2)

            with controller._lock:
                controller._formation_completed_analysis = analysis
                snapshot = controller._make_snapshot_unlocked()

            self.assertIs(snapshot.rally_analysis, analysis)

    def test_build_rally_task_init_ignores_removed_last_arrival_threshold_s(self) -> None:
        """last_arrival_threshold_s 已移除，配置中出现该键应被忽略，不影响加载，且结果中不携带该字段。"""
        config = _rally_config()
        config.setdefault("rally_cfg", {})["last_arrival_threshold_s"] = 99.0  # type: ignore[index]
        nodes = list(config["nodes"])  # type: ignore[arg-type]

        task_init = _build_rally_task_init(config, 0.02, nodes)

        self.assertIsNotNone(task_init)
        self.assertFalse(hasattr(task_init, "last_arrival_threshold_s"))

    def test_validate_rejects_rally_leader_without_rally_route(self) -> None:
        """rally_leader 角色缺少 rally_route 时 validate() 应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        del config["rally_route"]  # type: ignore[misc]
        with self.assertRaises(ValueError, msg="rally_route is required"):
            _ConfigLoader().validate(config)

    def test_validate_rejects_rally_leader_without_route(self) -> None:
        """rally_leader 角色缺少任务 route 时 validate() 应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        del config["route"]  # type: ignore[misc]
        with self.assertRaises(ValueError, msg="route is required"):
            _ConfigLoader().validate(config)

    def test_validate_rejects_mission_route_start_far_from_rally_route_start(self) -> None:
        """route（mission_route）起点必须等于 rally_route 起点 A；相距过远时 validate() 应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        config["route"]["waypoints"][0]["x_m"] = 9999.0  # type: ignore[index]
        with self.assertRaises(ValueError, msg="route start must equal rally_route start A"):
            _ConfigLoader().validate(config)

    def test_validate_rejects_mission_route_first_segment_direction_mismatch(self) -> None:
        """route（mission_route）首段方向必须与 rally_route 首段方向（任务航向）大致一致；
        起点位置对得上但方向差得多（如垂直）时 validate() 也应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        # 起点仍是 (0,0)（跟 rally_route 起点 A 对得上），但第二个航点改成正北，
        # 首段方向从正东变成正北，跟 rally_route 的正东方向差 90°。
        config["route"]["waypoints"][1] = {"x_m": 0.0, "y_m": 200.0, "altitude_m": 500.0, "R": 0.0}  # type: ignore[index]
        with self.assertRaises(ValueError, msg="route first-segment direction must match rally_route's"):
            _ConfigLoader().validate(config)

    def test_validate_rejects_loiter_radius_too_small_for_capture_window(self) -> None:
        """回归用例：过小的 loiter_radius_m 应在 validate() 阶段就被拒绝（ERR_CONFIG_INVALID），
        而不是要等到 load_config() 实际构造 RallyJoinPos 时才失败（之前是 ERR_MODULE_INIT_FAILED，
        语义上属于"配置错误"而不是"模块初始化失败"，且报错时机应尽量提前）。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        config.setdefault("rally_cfg", {})["loiter_radius_m"] = 1.0  # type: ignore[index]
        with self.assertRaises(ValueError, msg="loiter_radius_m too small for capture window"):
            _ConfigLoader().validate(config)

    def test_validate_rejects_rally_roles_without_rally_cfg(self) -> None:
        """任何集结角色都缺少 rally_cfg 时 validate() 应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        del config["rally_cfg"]  # type: ignore[misc]
        with self.assertRaises(ValueError, msg="rally_cfg is required"):
            _ConfigLoader().validate(config)

    def test_repository_rally_demo_config_loads(self) -> None:
        """验证仓库内 rally_demo.json 与控制器当前配置契约一致。"""

        controller = SimulationController()
        self.addCleanup(controller.close)
        result = controller.load_config("configs/rally_demo.json")

        self.assertEqual(result.code, "OK")
        snapshot = controller.get_snapshot()
        self.assertEqual([node.role for node in snapshot.nodes], ["rally_leader", "rally_follower", "rally_follower"])
        self.assertEqual(controller.get_formation_names(), ["三机三角"])
        self.assertEqual(len(snapshot.route_segments), 1)

    def test_repository_rally_demo_5_aircraft_config_loads(self) -> None:
        """验证仓库内 5 机集结配置复用五机多队形，并以 A03 作为集结长机。"""

        controller = SimulationController()
        self.addCleanup(controller.close)
        result = controller.load_config("configs/rally_demo_5_aircraft.json")

        self.assertEqual(result.code, "OK")
        snapshot = controller.get_snapshot()
        self.assertEqual(
            [node.role for node in snapshot.nodes],
            ["rally_follower", "rally_follower", "rally_leader", "rally_follower", "rally_follower"],
        )
        self.assertEqual(controller.get_formation_names(), ["五机楔形", "五机横队", "五机双纵队"])
        self.assertEqual(len(snapshot.route_segments), 4)
        self.assertEqual(controller.switch_formation(1).code, "OK")
        self.assertEqual(controller.switch_formation(2).code, "OK")

    def test_repository_rally_demo_initial_heading_points_toward_rally_origin(self) -> None:
        """验证仓库内 rally_demo.json 三机初始航向大致对齐集结航线起点，避免起始瞬间大转向。

        回归背景：三机 psi_v_deg 曾固定为 0，与各自实际需要飞向的方向偏差很大，
        导致运行开始的几秒内速度矢量方向剧烈摆动，观感上像是一开始就在打转。
        """
        with open("configs/rally_demo.json", encoding="utf-8") as f:
            config = json.load(f)

        expected_psi_deg = {"A01": 153.43, "A02": 33.69, "A03": -158.20}
        for node in config["nodes"]:
            node_id = node["node_id"]
            bearing_to_origin_deg = math.degrees(math.atan2(-node["y_m"], -node["x_m"]))
            # 航向应对齐指向集结航线起点(0,0)的方位角，容差覆盖编队槽位偏移带来的小角度误差。
            self.assertAlmostEqual(node["psi_v_deg"], bearing_to_origin_deg, delta=1.0)
            self.assertAlmostEqual(node["psi_v_deg"], expected_psi_deg[node_id], places=2)


if __name__ == "__main__":
    unittest.main()
