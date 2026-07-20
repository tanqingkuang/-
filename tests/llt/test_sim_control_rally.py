"""集结场景仿真控制器低层测试。"""

from __future__ import annotations

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

from src.algorithm.context.leaf_types import FormStageE, PosInEarthS, WayPointInputS
from src.algorithm.units.algo.pos_calc.rally_join_pos import RALLY_STATE_STANDBY
from src.algorithm.units.process.outbound.follower_broadcast import FOLLOWER_STATUS_TOPIC
from src.common.envelope import MessageEnvelope
from tests.llt._geo_route import geodetic_config
from src.algorithm.entity.leader_follower.follower import FollowerEntity
from src.algorithm.entity.leader_follower.leader import LeaderEntity
from src.algorithm.units.algo.pos_calc import PosCalcStrategyE
from src.algorithm.units.algo.pos_calc.rally_join_pos import RallyJoinPos
from src.runner.sim_control import (
    RallyPlanGeometryState,
    SimulationController,
    _build_formation_comm_init,
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
            # 统一航线首点是集结中心，首段确定集结方向，后续航段用于编队任务飞行。
            "speed_mps": 20.0,
            "waypoints": [
                {"x_m": 0.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
                {"x_m": 200.0, "y_m": 0.0, "altitude_m": 500.0, "R": 0.0},
            ],
        },
        "rally_cfg": {
            "loose_scale": 3.0,
            "convergence_radius_m": 5.0,
            "stable_hold_s": 0.1,
            "tight_radius_m": 2.0,
            "stale_timeout_s": 1.0,
            "altitude_separation_m": 60.0,
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


def _snapshot_node_phases(controller: SimulationController) -> dict[str, str]:
    """读取快照中的节点集结阶段。注意：测试只关心 node_id 到 phase 的稳定映射。"""

    return {node.node_id: node.rally_phase for node in controller.get_snapshot().nodes}


def _rally_join(entity: LeaderEntity | FollowerEntity) -> RallyJoinPos:
    """取得 Manager 缓存的集结位置解算产品，供白盒几何断言使用。"""

    strategy = entity._pos_calc._registry[PosCalcStrategyE.RALLY_JOIN]
    assert isinstance(strategy, RallyJoinPos)
    return strategy


class SimControlRallyTests(unittest.TestCase):
    """验证控制器对集结场景的配置解析、实体装配和快照透传。"""

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
        self.assertFalse(hasattr(task_init, "compressTime_s"))

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
            self.assertIsInstance(controller._node_algorithms["R01"]._entity, LeaderEntity)
            self.assertIsInstance(controller._node_algorithms["R02"]._entity, FollowerEntity)
            self.assertIsInstance(controller._node_algorithms["R03"]._entity, FollowerEntity)
            self.assertIsNotNone(snapshot.route)
            assert snapshot.route is not None
            self.assertAlmostEqual(snapshot.route.start_x_m, 0.0, places=3)
            self.assertAlmostEqual(snapshot.route.end_x_m, 200.0, places=3)

    def test_apply_avoidance_route_rebuilds_all_rally_geometry_from_override(self) -> None:
        """验证采用避障航线后，全部集结实体与界面几何同步使用覆盖航线。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            override = [
                WayPointInputS(pos=PosInEarthS(east=100.0, north=200.0, h=500.0), vdCmd=20.0),
                WayPointInputS(pos=PosInEarthS(east=100.0, north=400.0, h=500.0), vdCmd=20.0),
            ]

            result = controller.apply_avoidance_route(override)
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertIsNotNone(snapshot.route)
            assert snapshot.route is not None
            self.assertAlmostEqual(snapshot.route.start_x_m, 100.0)
            self.assertAlmostEqual(snapshot.route.start_y_m, 200.0)
            self.assertAlmostEqual(snapshot.route.end_x_m, 100.0)
            self.assertAlmostEqual(snapshot.route.end_y_m, 400.0)
            for node_id, algorithm in controller._node_algorithms.items():
                rally_join = _rally_join(algorithm._entity)
                geometry = snapshot.rally_geometry[node_id]
                self.assertAlmostEqual(rally_join._mission_heading, math.pi / 2.0)
                self.assertAlmostEqual(geometry.rally_center_east_m, rally_join._loiter_center_e)
                self.assertAlmostEqual(geometry.rally_center_north_m, rally_join._loiter_center_n)

    def test_rally_snapshot_ready_hides_phase_and_exposes_plan_geometry(self) -> None:
        """验证 READY 阶段只暴露几何预览，不把算法待命盘旋阶段写入节点状态。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            result = controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "READY")
            self.assertEqual(snapshot.control_report, "待命")
            self.assertEqual(_snapshot_node_phases(controller), {"R01": "", "R02": "", "R03": ""})
            self.assertEqual(set(snapshot.rally_geometry), {"R01", "R02", "R03"})
            for geometry in snapshot.rally_geometry.values():
                self.assertIsInstance(geometry, RallyPlanGeometryState)
                self.assertGreater(geometry.local_radius_m, 0.0)
                self.assertGreater(geometry.rally_radius_m, 0.0)
                for removed_field in (
                    "local_tangent_east_m",
                    "local_tangent_north_m",
                    "rally_tangent_east_m",
                    "rally_tangent_north_m",
                    "slot_east_m",
                    "slot_north_m",
                    "fallback_used",
                ):
                    self.assertFalse(hasattr(geometry, removed_field))

    def test_start_rally_command_requires_loaded_running_or_paused_standby(self) -> None:
        """验证开始集结命令只允许在已开始运行后的本地待命盘旋阶段触发。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)

            self.assertEqual(controller.start_rally().code, "ERR_NO_CONFIG")
            load_result = controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            ready_result = controller.start_rally()
            step_result = controller.step(1)
            standby_phases = _snapshot_node_phases(controller)
            first_start = controller.start_rally()
            duplicate_start = controller.start_rally()

            self.assertEqual(load_result.code, "OK")
            self.assertEqual(ready_result.code, "ERR_INVALID_STATE")
            self.assertIn("请先开始运行", ready_result.message)
            self.assertEqual(step_result.code, "OK")
            self.assertEqual(standby_phases, {"R01": "LOCAL_LOITER", "R02": "LOCAL_LOITER", "R03": "LOCAL_LOITER"})
            self.assertEqual(first_start.code, "OK")
            self.assertEqual(duplicate_start.code, "ERR_INVALID_STATE")
            self.assertIn("已在集结中", duplicate_start.message)

    def test_pause_in_local_loiter_keeps_control_report_standby(self) -> None:
        """验证本地待命盘旋暂停时，全局回报仍派生为待命而不是写死保持。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            controller.step(1)
            controller._run_state = "RUNNING"

            result = controller.pause()
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "PAUSED")
            self.assertEqual(snapshot.control_report, "待命")
            self.assertEqual(_snapshot_node_phases(controller), {"R01": "LOCAL_LOITER", "R02": "LOCAL_LOITER", "R03": "LOCAL_LOITER"})

    def test_set_duration_to_current_time_keeps_local_loiter_report_standby(self) -> None:
        """验证待命阶段被时长边界结束时，全局回报仍按当前阶段派生。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            controller.step(1)
            current_time_s = controller.get_snapshot().time_s

            result = controller.set_duration(current_time_s)
            snapshot = controller.get_snapshot()

            self.assertEqual(result.code, "OK")
            self.assertEqual(snapshot.run_state, "FINISHED")
            self.assertEqual(snapshot.control_report, "待命")
            self.assertEqual(_snapshot_node_phases(controller), {"R01": "LOCAL_LOITER", "R02": "LOCAL_LOITER", "R03": "LOCAL_LOITER"})

    def test_rally_standby_tick_is_executed_by_entity_layer(self) -> None:
        """验证待命盘旋由实体层执行，runner 只把 STANDBY 遥控阶段转发进去。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            algorithm = controller._node_algorithms["R01"]
            calls: list[FormStageE] = []
            original_step = algorithm._entity.step

            def wrapped_step(entity_input: object, entity_output: object) -> None:
                remote = getattr(entity_input, "remote", None)
                calls.append(remote.stage if remote is not None else FormStageE.NONE)
                original_step(entity_input, entity_output)  # type: ignore[arg-type]

            algorithm._entity.step = wrapped_step  # type: ignore[method-assign]

            result = controller.step(1)

            self.assertEqual(result.code, "OK")
            self.assertEqual(calls, [FormStageE.STANDBY])

    def test_run_until_complete_auto_starts_rally_config(self) -> None:
        """验证批处理运行集结配置时会自动离开本地待命，而不是盘旋到结束。"""

        config = _rally_config()
        config["duration_s"] = 0.05
        controller = SimulationController()
        self.addCleanup(controller.close)

        result = controller.run_until_complete(config)
        snapshot = controller.get_snapshot()
        phases = {node.node_id: node.rally_phase for node in snapshot.nodes}

        self.assertEqual(result.code, "OK")
        self.assertEqual(snapshot.run_state, "FINISHED")
        self.assertEqual(snapshot.control_report, "集结")
        self.assertNotEqual(set(phases.values()), {"LOCAL_LOITER"})
        for node_id, algorithm in controller._node_algorithms.items():
            join = _rally_join(algorithm._entity)
            self.assertIsNotNone(join._standby_center_e, msg=node_id)
            self.assertIsNotNone(join._standby_center_n, msg=node_id)
        self.assertEqual(
            _rally_join(controller._node_algorithms["R02"]._entity)._transit_phase,
            "ARC_TO_TANGENT",
        )

    def test_start_rally_before_first_tick_primes_standby_geometry(self) -> None:
        """验证启动后立即集结时先建立待命圆，不退回旧的当前点到集结圆切线。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            # 模拟已经启动/暂停但后台首拍尚未执行的确定性边界。
            controller._run_state = "PAUSED"

            result = controller.start_rally()

            self.assertEqual(result.code, "OK")
            self.assertEqual(controller._tick_index, 0)
            for node_id, algorithm in controller._node_algorithms.items():
                join = _rally_join(algorithm._entity)
                self.assertIsNotNone(join._standby_center_e, msg=node_id)
                self.assertIsNotNone(join._standby_center_n, msg=node_id)

    def test_start_rally_first_tick_prime_does_not_advance_ordinary_nodes_or_communication(self) -> None:
        """验证首 tick 预热只初始化集结节点，不推进普通节点或通信状态。"""

        config = _rally_config()
        config["nodes"].append({
            "node_id": "W01",
            "role": "wingman",
            "x_m": -80.0,
            "y_m": 40.0,
            "altitude_m": 500.0,
            "speed_mps": 20.0,
        })
        config["formation"]["formations"][0]["slots"].append({
            "node_id": "W01",
            "x_m": -20.0,
            "y_m": 0.0,
            "z_m": 10.0,
        })
        config["links"].append({
            "link_id": "R01-W01",
            "direction": "duplex",
            "latency_ms": 1.0,
            "loss_rate": 0.25,
        })

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            self.assertEqual(controller.load_config(str(_write_json(Path(tmp), config))).code, "OK")
            controller._run_state = "PAUSED"

            ordinary_algorithm = controller._node_algorithms["W01"]
            ordinary_step_calls = 0
            original_step = ordinary_algorithm.step

            def wrapped_step(*args: object, **kwargs: object) -> object:
                nonlocal ordinary_step_calls
                ordinary_step_calls += 1
                return original_step(*args, **kwargs)

            ordinary_algorithm.step = wrapped_step  # type: ignore[method-assign]
            sentinel = MessageEnvelope("test.prime", "R01", "W01", 0.0, {"value": 1})
            controller._comm._inbox["W01"].append(sentinel)
            inbox_before = {node_id: list(messages) for node_id, messages in controller._comm._inbox.items()}
            in_flight_before = {link: list(messages) for link, messages in controller._comm._in_flight.items()}
            rng_before = copy.deepcopy(controller._comm._rng.bit_generator.state)

            result = controller.start_rally()

            self.assertEqual(result.code, "OK")
            self.assertEqual(controller._tick_index, 0)
            self.assertEqual(ordinary_step_calls, 0)
            self.assertEqual(controller._comm._inbox, inbox_before)
            self.assertEqual(controller._comm._in_flight, in_flight_before)
            self.assertEqual(controller._comm._rng.bit_generator.state, rng_before)

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
            self.assertAlmostEqual(_rally_join(follower)._approach_speed, 16.0)

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
            self.assertAlmostEqual(_rally_join(follower)._v_up_min, -2.0)
            self.assertAlmostEqual(_rally_join(follower)._v_up_max, 2.5)

    def test_manual_step_before_start_rally_stays_local_loiter(self) -> None:
        """验证点开始集结前，手动步进进入本地待命盘旋，并正常投递 STANDBY 广播。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))

            result = controller.step(1)
            stage = controller._node_algorithms["R01"].current_stage()
            inbox = controller._comm.read_inbox("R02")
            leader_inbox = controller._comm.read_inbox("R01")

            self.assertEqual(result.code, "OK")
            self.assertEqual(stage, FormStageE.STANDBY)
            self.assertEqual(_snapshot_node_phases(controller), {"R01": "LOCAL_LOITER", "R02": "LOCAL_LOITER", "R03": "LOCAL_LOITER"})
            self.assertTrue(any(
                msg.topic == "formation.leader" and msg.payload.get("cmd", {}).get("stage") == int(FormStageE.STANDBY)
                for msg in inbox
            ))
            self.assertTrue(any(
                msg.topic == FOLLOWER_STATUS_TOPIC and msg.payload.get("rally_state") == RALLY_STATE_STANDBY
                for msg in leader_inbox
            ))

    def test_local_loiter_standby_outputs_tangential_control(self) -> None:
        """验证集结待命阶段输出本地盘旋切向速度和非零控制，而不是原地冻结。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))

            result = controller.step(1)
            snapshot = controller.get_snapshot()
            node = next(item for item in snapshot.nodes if item.node_id == "R02")
            control = controller._current_controls["R02"]

            self.assertEqual(result.code, "OK")
            self.assertGreater(math.hypot(node.cmd_vel_east_mps, node.cmd_vel_north_mps), 1.0)
            self.assertGreater(abs(control.ax_cmd_mps2) + abs(control.ay_cmd_mps2), 0.01)

    def test_rally_standby_and_joining_use_layered_altitudes(self) -> None:
        """验证待命和 JOINING 阶段按集结高度分层，避免多机同高转场。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))

            controller.step(1)
            standby_altitudes = {
                node.node_id: node.cmd_pos_h_m
                for node in controller.get_snapshot().nodes
            }
            controller.start_rally()
            controller.step(4)
            joining_altitudes = {
                node.node_id: node.cmd_pos_h_m
                for node in controller.get_snapshot().nodes
            }

            self.assertEqual(standby_altitudes, {"R01": 500.0, "R02": 560.0, "R03": 440.0})
            self.assertEqual(joining_altitudes, {"R01": 500.0, "R02": 560.0, "R03": 440.0})

    def test_start_rally_then_step_enters_rally_transit_and_sends_leader_broadcast(self) -> None:
        """验证点击开始集结后，下一拍进入集结转场并恢复长机广播。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            controller.step(1)

            start_result = controller.start_rally()
            step_result = controller.step(2)
            stage = controller._node_algorithms["R01"].current_stage()
            inbox = controller._comm.read_inbox("R02")

            self.assertEqual(start_result.code, "OK")
            self.assertEqual(step_result.code, "OK")
            self.assertEqual(stage, FormStageE.RALLY)
            phases = _snapshot_node_phases(controller)
            self.assertIn(phases["R01"], {"RALLY_TRANSIT", "RALLY_LOITER"})
            self.assertEqual(phases["R02"], "RALLY_TRANSIT")
            self.assertEqual(phases["R03"], "RALLY_TRANSIT")
            self.assertTrue(any(msg.topic == "formation.leader" for msg in inbox))

    def test_start_rally_plans_each_aircraft_from_its_own_standby_circle(self) -> None:
        """验证各机独立规划公切线，并在转移期间保持原集结分层高度。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))
            controller.step(1)
            standby_altitudes = {
                node.node_id: node.cmd_pos_h_m
                for node in controller.get_snapshot().nodes
            }

            self.assertEqual(controller.start_rally().code, "OK")
            # 僚机需等待长机广播经现有通信链路送达，4 拍后再核对全队规划结果。
            self.assertEqual(controller.step(4).code, "OK")
            joining_snapshot = controller.get_snapshot()

            for node_id, algorithm in controller._node_algorithms.items():
                entity = algorithm._entity
                join = _rally_join(entity)
                center_distance = math.hypot(
                    join._loiter_center_e - join._standby_center_e,
                    join._loiter_center_n - join._standby_center_n,
                )
                if center_distance <= 0.5:
                    self.assertEqual(join.state, "LOITERING", msg=node_id)
                    continue
                self.assertEqual(join.state, "FLYING", msg=node_id)
                self.assertEqual(join._transit_phase, "ARC_TO_TANGENT", msg=node_id)
                self.assertAlmostEqual(
                    math.hypot(
                        entity.cxt.selfCmd.pos.east - join._standby_center_e,
                        entity.cxt.selfCmd.pos.north - join._standby_center_n,
                    ),
                    join._loiter_radius,
                )

            joining_altitudes = {
                node.node_id: node.cmd_pos_h_m
                for node in joining_snapshot.nodes
            }
            self.assertEqual(joining_altitudes, standby_altitudes)
            self.assertTrue(all(
                node.rally_phase in {"RALLY_TRANSIT", "RALLY_LOITER"}
                for node in joining_snapshot.nodes
            ))

    def test_snapshot_excludes_removed_rally_analysis(self) -> None:
        """编队分析功能删除后，控制器和快照不得继续暴露旧接口。"""

        with tempfile.TemporaryDirectory() as tmp:
            controller = SimulationController()
            self.addCleanup(controller.close)
            controller.load_config(str(_write_json(Path(tmp), _rally_config())))

            with controller._lock:
                snapshot = controller._make_snapshot_unlocked()

            self.assertFalse(hasattr(controller, "_formation_completed_analysis"))
            self.assertFalse(hasattr(snapshot, "rally_analysis"))

    def test_build_rally_task_init_ignores_removed_last_arrival_threshold_s(self) -> None:
        """last_arrival_threshold_s 已移除，配置中出现该键应被忽略，不影响加载，且结果中不携带该字段。"""
        config = _rally_config()
        config.setdefault("rally_cfg", {})["last_arrival_threshold_s"] = 99.0  # type: ignore[index]
        nodes = list(config["nodes"])  # type: ignore[arg-type]

        task_init = _build_rally_task_init(config, 0.02, nodes)

        self.assertIsNotNone(task_init)
        self.assertFalse(hasattr(task_init, "last_arrival_threshold_s"))

    def test_validate_accepts_rally_roles_with_route_only(self) -> None:
        """集结场景只提供统一 route 时应通过配置校验。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        _ConfigLoader().validate(config)

    def test_validate_rejects_rally_leader_without_route(self) -> None:
        """rally_leader 角色缺少任务 route 时 validate() 应拒绝配置。"""
        from src.runner.sim_control import _ConfigLoader

        config = _rally_config()
        del config["route"]  # type: ignore[misc]
        with self.assertRaises(ValueError, msg="route is required"):
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

    def test_repository_legacy_three_aircraft_rally_demo_removed(self) -> None:
        """验证旧三机 rally_demo.json 不再作为仓库演示配置保留。"""

        self.assertFalse(Path("configs/rally_demo.json").exists())

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
        """验证默认五机集结配置初始航向大致对齐集结航线起点，避免起始瞬间大转向。

        回归背景：集结演示的 psi_v_deg 曾固定为 0，与各自实际需要飞向的方向偏差很大，
        导致运行开始的几秒内速度矢量方向剧烈摆动，观感上像是一开始就在打转。
        """
        with open("configs/rally_demo_5_aircraft.json", encoding="utf-8") as f:
            config = json.load(f)

        expected_psi_deg = {"A01": 153.43, "A02": 33.69, "A03": -158.20, "A04": -25.64, "A05": 156.37}
        for node in config["nodes"]:
            node_id = node["node_id"]
            bearing_to_origin_deg = math.degrees(math.atan2(-node["y_m"], -node["x_m"]))
            # 航向应对齐指向集结航线起点(0,0)的方位角，容差覆盖编队槽位偏移带来的小角度误差。
            self.assertAlmostEqual(node["psi_v_deg"], bearing_to_origin_deg, delta=1.0)
            self.assertAlmostEqual(node["psi_v_deg"], expected_psi_deg[node_id], places=2)


if __name__ == "__main__":
    unittest.main()
