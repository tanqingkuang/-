"""3D 态势场景数据适配回归测试。"""

from __future__ import annotations

from pathlib import Path
import json
import struct
import time
import unittest
from unittest.mock import patch

from src.ui.gui.situation3d import scene_data
from src.ui.gui.situation3d.scene_data import (
    DEFAULT_TERRAIN_SPAN_M,
    MAX_ROUTE_DASHES_PER_SEGMENT,
    build_scene_payload,
    enu_to_quick3d,
)
from src.ui.gui.situation3d import terrain_field as terrain_field_module
from src.ui.gui.situation3d.terrain_field import (
    DEFAULT_TERRAIN_RESOLUTION,
    generate_terrain_field,
    generate_terrain_field_from_file,
)
from src.ui.gui.situation3d.terrain_geometry import TerrainGeometry
from src.ui.gui.situation3d.trail_ribbon_geometry import TrailRibbonGeometry
from src.ui.gui.simulation_adapter import ControllerSimulationAdapter
from src.ui.gui.view_models import (
    LinkState,
    NodeState,
    ObstacleView,
    ReferenceRoute,
    Snapshot,
    TrailPoint,
)

QML_VIEW_PATH = Path(__file__).resolve().parents[2] / "src" / "ui" / "gui" / "situation3d" / "qml" / "Situation3DView.qml"
TERRAIN_LAYOUT_PATH = Path(__file__).resolve().parents[2] / "configs" / "element" / "terrain_mountain_demo.json"
MOUNTAIN_CONFIG_PATH = Path(__file__).resolve().parents[2] / "configs" / "mountain_demo.json"


class Situation3DSceneDataTests(unittest.TestCase):
    """验证 3D 视图适配层不污染仿真坐标语义。"""

    def _snapshot(self) -> Snapshot:
        """构造含飞机、尾迹和航线的最小快照。"""
        return Snapshot(
            time=12.0,
            duration=120.0,
            step=0.1,
            run_state="RUNNING",
            control_report="保持",
            disturbance="无",
            nodes=[
                NodeState(
                    node_id="A01",
                    role="leader",
                    x=10.0,
                    y=20.0,
                    vx=3.0,
                    vy=4.0,
                    altitude=30.0,
                    trail=[TrailPoint(1.0, 2.0, 3.0, 10.0), TrailPoint(4.0, 5.0, 6.0, 11.0)],
                ),
                NodeState("A02", "wingman", 40.0, 50.0, 1.0, 0.0, altitude=60.0),
            ],
            links=[LinkState("A01", "A02", "duplex", 10, 0.0)],
            route_segments=[ReferenceRoute(0.0, 0.0, 100.0, 100.0, 50.0, 150.0)],
        )

    def test_enu_to_quick3d_maps_north_to_negative_z(self) -> None:
        self.assertEqual(enu_to_quick3d(10.0, 20.0, 30.0), {"x": 10.0, "y": 30.0, "z": -20.0})

    def test_payload_contains_aircraft_route_trails_obstacles_and_terrain(self) -> None:
        obstacle = ObstacleView("OBS1", "circle", center_x=80.0, center_y=70.0, radius=25.0)

        payload = build_scene_payload(self._snapshot(), [obstacle], clearance_m=5.0)

        aircraft = payload["aircraft"]
        self.assertEqual([item["nodeId"] for item in aircraft], ["A01", "A02"])
        self.assertEqual(aircraft[0]["x"], 10.0)
        self.assertEqual(aircraft[0]["y"], 30.0)
        self.assertEqual(aircraft[0]["z"], -20.0)
        # vx=3, vy=4(东偏北航向)，机头应偏向 Quick3D 的 +z 象限，yawDeg 应为正。
        self.assertGreater(aircraft[0]["yawDeg"], 0.0)

        self.assertEqual(payload["counts"]["aircraft"], 2)
        self.assertEqual(payload["counts"]["trailRibbons"], 1)
        self.assertGreaterEqual(payload["counts"]["routePoints"], 2)
        self.assertGreaterEqual(payload["counts"]["routeDashes"], 1)
        self.assertEqual(payload["counts"]["obstacles"], 1)
        self.assertNotIn("trailPoints", payload)
        self.assertNotIn("trailSegments", payload)
        trail_ribbon = payload["trailRibbons"][0]
        self.assertEqual(trail_ribbon["nodeId"], "A01")
        self.assertEqual(trail_ribbon["width"], 44.0)
        self.assertEqual(json.loads(trail_ribbon["pathValue"]), [[1.0, 3.0, -2.0], [4.0, 6.0, -5.0]])
        route_dash = payload["routeDashes"][0]
        self.assertEqual(route_dash["color"], "#22d3ee")
        self.assertEqual(route_dash["width"], 16.0)
        route_dash_path = json.loads(route_dash["pathValue"])
        self.assertGreaterEqual(len(route_dash_path), 2)
        self.assertEqual(route_dash_path[0], [0.0, 100.0, -0.0])
        self.assertEqual(payload["terrain"]["ground"]["width"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["ground"]["depth"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["surface"]["mode"], "procedural")
        self.assertEqual(payload["terrain"]["surface"]["width"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["surface"]["depth"], DEFAULT_TERRAIN_SPAN_M)
        self.assertGreater(payload["terrain"]["surface"]["height"], 0.0)
        self.assertEqual(payload["riskZones"], [])

        obstacle_payload = payload["obstacles"][0]
        self.assertEqual(obstacle_payload["kind"], "circle")
        self.assertEqual(obstacle_payload["radius"], 30.0)
        self.assertEqual(obstacle_payload["z"], -70.0)

    def test_terrain_field_generates_layout_height_grid_and_risk_zones(self) -> None:
        """验证布局地形高度场尺寸、有限值和风险区显式标记。"""

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=128)

        self.assertEqual(field.resolution, 128)
        self.assertEqual(field.heights_m.shape, (128, 128))
        self.assertEqual(field.normals.shape, (128, 128, 3))
        self.assertEqual(field.colors.shape, (128, 128, 3))
        self.assertFalse(bool((field.heights_m != field.heights_m).any()))
        self.assertGreater(float(field.heights_m.max()), 2000.0)
        self.assertEqual([zone.zone_id for zone in field.risk_zones], ["hazard_peak_west", "hazard_peak_east"])
        self.assertLess(field.generation_time_ms, 5000.0)

    def test_payload_uses_layout_terrain_and_risk_zone_models(self) -> None:
        """验证 terrain_display_file 会进入布局模式，并导出风险区渲染数据(覆盖正式 768 网格)。"""

        snapshot = self._snapshot()
        snapshot.terrain_display_file = str(TERRAIN_LAYOUT_PATH)
        # 阻塞预热正式分辨率高度场,使 payload 走确定性的就绪路径。
        layout_detail_pre = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))["detail"]
        terrain_field_module.get_terrain_field(TERRAIN_LAYOUT_PATH, resolution=layout_detail_pre["grid_resolution"])

        payload = build_scene_payload(snapshot)
        self.assertTrue(payload["terrain"]["surface"]["fieldReady"])

        surface = payload["terrain"]["surface"]
        self.assertEqual(surface["mode"], "layout")
        self.assertEqual(surface["layoutFile"], str(TERRAIN_LAYOUT_PATH.resolve()))
        # 分辨率应跟随布局文件 detail.grid_resolution,而不是模块默认值。
        layout_detail = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))["detail"]
        self.assertEqual(surface["resolution"], layout_detail["grid_resolution"])
        self.assertEqual(payload["counts"]["riskZones"], 2)
        self.assertEqual(len(payload["riskZones"]), 2)
        self.assertGreater(len(payload["riskZoneLines"]), 0)
        self.assertGreater(len(payload["riskZoneBuffers"]), 0)
        # 风险网格必须细于主航线(16m),保持任务航线的视觉层级优先。
        self.assertEqual(payload["riskZoneLines"][0]["width"], 7.0)
        self.assertLess(payload["riskZoneBuffers"][0]["width"], 7.0)
        risk_line_points = json.loads(payload["riskZoneLines"][0]["pathValue"])
        self.assertGreater(len(risk_line_points), 2)
        self.assertGreater(max(point[1] for point in risk_line_points) - min(point[1] for point in risk_line_points), 1.0)
        buffer_points = json.loads(payload["riskZoneBuffers"][0]["pathValue"])
        self.assertGreater(buffer_points[0][1], 0.0)

    def test_terrain_heights_keep_metric_semantics(self) -> None:
        """验证高度场米制语义:空布局平坦、低峰不被抬高、风险峰中心贴近声明高度。"""

        empty = {"mountain_chains": [], "map": {"render_extent_km": 52, "effective_extent_km": 32}}
        self.assertLess(float(generate_terrain_field(empty, resolution=128).heights_m.max()), 60.0)

        low = {
            "mountain_chains": [
                {
                    "id": "t",
                    "polyline_uv": [[0, 0], [2, 0]],
                    "saddle_height_factor": 0.3,
                    "peaks": [{"id": "p", "station_km": 1.0, "height_m": 100, "base_radius_km": 0.8}],
                }
            ],
            "map": {"render_extent_km": 20, "effective_extent_km": 10},
        }
        low_max = float(generate_terrain_field(low, resolution=257).heights_m.max())
        self.assertGreater(low_max, 80.0)
        self.assertLess(low_max, 140.0)

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=257)
        for zone in field.risk_zones:
            measured = scene_data._sample_field_height(field, zone.east_m, zone.north_m)
            self.assertLess(abs(measured - zone.height_m) / zone.height_m, 0.12)

    def test_route_corridor_keeps_cruise_clearance(self) -> None:
        """验证走廊段(进入风险链前)地形满足 巡航900m-净空200m;风险段封锁属 P2 演示语义。"""

        layout = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))
        cruise = float(layout["flight"]["cruise_altitude_m"])
        clearance = float(layout["flight"]["clearance_m"])
        route = layout["flight"]["original_route_uv"]
        corridor = [point for point in route if point[0] <= 14.2] + [[14.2, 0.0]]
        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=641)
        worst = 0.0
        for (u1, v1), (u2, v2) in zip(corridor, corridor[1:]):
            for step in range(300):
                ratio = step / 299.0
                east = (u1 + (u2 - u1) * ratio) * 1000.0
                north = (v1 + (v2 - v1) * ratio) * 1000.0
                worst = max(worst, scene_data._sample_field_height(field, east, north))
        self.assertLessEqual(worst, cruise - clearance)

    def test_terrain_field_cache_is_shared_and_generates_once(self) -> None:
        """验证正式 build_scene_payload 链路与 TerrainGeometry 共享缓存,768 场只生成一次。"""

        self._reset_terrain_caches()
        layout_detail = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))["detail"]
        resolution = int(layout_detail["grid_resolution"])
        snapshot = self._snapshot()
        snapshot.terrain_display_file = str(TERRAIN_LAYOUT_PATH)
        with patch.object(
            terrain_field_module,
            "generate_terrain_field_from_file",
            wraps=terrain_field_module.generate_terrain_field_from_file,
        ) as spy:
            payload = build_scene_payload(snapshot)
            deadline = time.monotonic() + 60.0
            # 正式链路是异步就绪:轮询到 fieldReady 后再走几何层,验证仍只生成一次。
            while not payload["terrain"]["surface"]["fieldReady"] and time.monotonic() < deadline:
                time.sleep(0.1)
                payload = build_scene_payload(snapshot)
            self.assertTrue(payload["terrain"]["surface"]["fieldReady"])
            geometry = TerrainGeometry()
            geometry.resolutionValue = resolution
            geometry.layoutFile = str(TERRAIN_LAYOUT_PATH)
            self.assertGreater(geometry.boundsMax().y(), 2000.0)
        self.assertEqual(spy.call_count, 1)

    def test_payload_stays_responsive_while_field_generates(self) -> None:
        """验证冷缓存下 payload 构建不阻塞主线程,就绪后 fieldReady/revision 翻转。"""

        import tempfile

        layout = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))
        layout["detail"]["grid_resolution"] = 128
        with tempfile.TemporaryDirectory() as temp_dir:
            layout_path = Path(temp_dir) / "async_layout.json"
            layout_path.write_text(json.dumps(layout, ensure_ascii=False), encoding="utf-8")
            self._reset_terrain_caches()
            snapshot = self._snapshot()
            snapshot.terrain_display_file = str(layout_path)
            started = time.monotonic()
            payload = build_scene_payload(snapshot)
            elapsed = time.monotonic() - started
            self.assertLess(elapsed, 1.0)
            self.assertFalse(payload["terrain"]["surface"]["fieldReady"])
            self.assertTrue(str(payload["terrain"]["surface"]["revision"]).endswith(":0"))
            deadline = time.monotonic() + 30.0
            while not payload["terrain"]["surface"]["fieldReady"] and time.monotonic() < deadline:
                time.sleep(0.05)
                payload = build_scene_payload(snapshot)
            self.assertTrue(payload["terrain"]["surface"]["fieldReady"])
            self.assertTrue(str(payload["terrain"]["surface"]["revision"]).endswith(":1"))
            # 就绪后风险线应贴地采样(y 有起伏),而不是占位的水平线。
            points = json.loads(payload["riskZoneLines"][0]["pathValue"])
            self.assertGreater(max(p[1] for p in points) - min(p[1] for p in points), 1.0)

    @staticmethod
    def _reset_terrain_caches() -> None:
        """清空高度场与布局缓存。注意：仅测试使用,模拟冷启动。"""

        terrain_field_module._cached_field.cache_clear()
        with terrain_field_module._PENDING_LOCK:
            terrain_field_module._READY_FIELDS.clear()
            terrain_field_module._PENDING_KEYS.clear()
        scene_data._cached_layout_versioned.cache_clear()

    def test_invalid_layout_values_fall_back_to_procedural_with_diagnostic(self) -> None:
        """验证坏配置族(错误类型/NaN/零范围)都回退 procedural 并输出诊断。"""

        import tempfile

        mutations = [
            ("grid_resolution 类型错误", lambda data: data["detail"].__setitem__("grid_resolution", "很多")),
            ("render_extent_km 为零", lambda data: data["map"].__setitem__("render_extent_km", 0)),
            ("风险峰 height_m NaN", lambda data: data["mountain_chains"][4]["peaks"][0].__setitem__("height_m", "NaN")),
            ("风险峰 risk_radius_km NaN", lambda data: data["mountain_chains"][4]["peaks"][0].__setitem__("risk_radius_km", "NaN")),
        ]
        for label, mutate in mutations:
            with self.subTest(label):
                broken = json.loads(TERRAIN_LAYOUT_PATH.read_text(encoding="utf-8"))
                mutate(broken)
                with tempfile.TemporaryDirectory() as temp_dir:
                    broken_path = Path(temp_dir) / "broken_layout.json"
                    broken_path.write_text(json.dumps(broken, ensure_ascii=False), encoding="utf-8")
                    snapshot = self._snapshot()
                    snapshot.terrain_display_file = str(broken_path)
                    with self.assertLogs("src.ui.gui.situation3d.scene_data", level="WARNING"):
                        payload = build_scene_payload(snapshot)
                self.assertEqual(payload["terrain"]["surface"]["mode"], "procedural", label)
                # 末端防线:payload 必须可被 QML JSON.parse 消费,不含 NaN/Inf。
                json.dumps(payload, ensure_ascii=False, allow_nan=False)

    def test_release_scripts_bundle_detail_normal_texture(self) -> None:
        """验证细节法线贴图存在且以 --add-data 形式进入双平台打包参数(注释不算数)。"""

        import re

        project_root = Path(__file__).resolve().parents[2]
        texture = project_root / "src" / "ui" / "gui" / "situation3d" / "qml" / "assets" / "terrain_detail_normal.png"
        self.assertTrue(texture.is_file())
        self.assertGreater(texture.stat().st_size, 0)
        # 必须是真实的 --add-data 参数:源为该 PNG、目标为 QML 同级 assets 目录。
        pattern = re.compile(
            r'--add-data\s+"src/ui/gui/situation3d/qml/assets/terrain_detail_normal\.png[;:]src/ui/gui/situation3d/qml/assets"'
        )
        for script_name in ("scripts/build_windows_full_release.ps1", "scripts/build_macos_full_release.sh"):
            script_text = (project_root / script_name).read_text(encoding="utf-8")
            self.assertRegex(script_text, pattern, script_name)

    def test_follow_view_tracks_moving_leader_with_terrain_clearance(self) -> None:
        """谷地运动双帧测试:跟随锁定长机角色、随快照更新焦点,相机与视线保持地形净空。"""

        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow

        app = QApplication.instance() or QApplication([])

        def canyon_snapshot(leader_east: float) -> Snapshot:
            # 僚机排在第 0 位,验证跟随按角色而不是下标选目标。
            return Snapshot(
                time=1.0,
                duration=120.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[
                    NodeState("A01", "wingman", leader_east - 400.0, -120.0, 20.0, 0.0, altitude=900.0),
                    NodeState("A03", "leader", leader_east, 0.0, 20.0, 0.0, altitude=900.0),
                ],
                links=[],
                route_segments=[],
            )

        window = Situation3DWindow()
        window.set_snapshot(canyon_snapshot(9000.0))
        app.processEvents()
        root = window.quick_view.rootObject()
        self.assertIsNotNone(root)
        root.setFollowView()
        app.processEvents()
        self.assertEqual(str(root.property("followNodeId")), "A03")
        self.assertEqual(str(root.property("cameraMode")), "跟随")

        # 第二帧:长机沿峡谷东移,焦点必须跟上(Behavior 平滑,轮询等待收敛)。
        window.set_snapshot(canyon_snapshot(9600.0))
        deadline = time.monotonic() + 3.0
        while abs(float(root.property("focusX")) - 9600.0) > 5.0 and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.02)
        self.assertLess(abs(float(root.property("focusX")) - 9600.0), 5.0)

        # 相机与视线净空:按跟随构图公式还原相机点,采样正式地形验证离地。
        import math as pymath

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=257)
        focus = (float(root.property("focusX")), float(root.property("focusY")), float(root.property("focusZ")))
        yaw = pymath.radians(float(root.property("yaw")))
        pitch = pymath.radians(float(root.property("pitch")))
        distance = float(root.property("distance"))
        horizontal = distance * pymath.cos(pitch)
        camera = (
            focus[0] + horizontal * pymath.sin(yaw),
            focus[1] - distance * pymath.sin(pitch),
            focus[2] + horizontal * pymath.cos(yaw),
        )
        for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = camera[0] + (focus[0] - camera[0]) * ratio
            y = camera[1] + (focus[1] - camera[1]) * ratio
            z = camera[2] + (focus[2] - camera[2]) * ratio
            ground = scene_data._sample_field_height(field, x, -z)
            self.assertGreater(y, ground + 50.0, f"视线采样点 {ratio} 距地不足")
        window.close()

    def test_controller_adapter_keeps_terrain_display_file_as_gui_metadata(self) -> None:
        """验证演示配置能加载，且地形文件只作为 Snapshot 显示元数据透传。"""

        adapter = ControllerSimulationAdapter()
        try:
            snapshot = adapter.load_config(str(MOUNTAIN_CONFIG_PATH))
            self.assertEqual(adapter.last_result_code, "OK")
            self.assertEqual(snapshot.terrain_display_file, str(TERRAIN_LAYOUT_PATH.resolve()))
            self.assertIsNotNone(snapshot.route)
            self.assertGreaterEqual(len(snapshot.route_segments), 1)
        finally:
            adapter.close()

    def test_trail_smoothing_is_default_on_and_only_affects_trails(self) -> None:
        """尾迹默认平滑，但航线虚线仍使用原始采样结果。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [
            TrailPoint(0.0, 0.0, 100.0, 0.0),
            TrailPoint(100.0, 0.0, 100.0, 1.0),
            TrailPoint(100.0, 100.0, 100.0, 2.0),
            TrailPoint(200.0, 100.0, 100.0, 3.0),
        ]

        with patch.object(scene_data, "ENABLE_TRAIL_SMOOTHING", False):
            raw_payload = build_scene_payload(snapshot)
        smooth_payload = build_scene_payload(snapshot)

        raw_path = json.loads(raw_payload["trailRibbons"][0]["pathValue"])
        smooth_path = json.loads(smooth_payload["trailRibbons"][0]["pathValue"])

        self.assertEqual(
            raw_path,
            [
                [0.0, 100.0, -0.0],
                [100.0, 100.0, -0.0],
                [100.0, 100.0, -100.0],
                [200.0, 100.0, -100.0],
            ],
        )
        self.assertGreater(len(smooth_path), len(raw_path))
        self.assertEqual(smooth_path[0], raw_path[0])
        self.assertEqual(smooth_path[-1], raw_path[-1])
        self.assertNotEqual(smooth_path, raw_path)
        self.assertEqual(smooth_payload["routeDashes"], raw_payload["routeDashes"])

    def test_trail_smoothing_can_be_disabled_by_code_flag(self) -> None:
        """代码级开关关闭后，尾迹 pathValue 回到原始折线点。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [
            TrailPoint(0.0, 0.0, 100.0, 0.0),
            TrailPoint(100.0, 0.0, 100.0, 1.0),
            TrailPoint(100.0, 100.0, 100.0, 2.0),
        ]

        with patch.object(scene_data, "ENABLE_TRAIL_SMOOTHING", False):
            payload = build_scene_payload(snapshot)

        path = json.loads(payload["trailRibbons"][0]["pathValue"])

        self.assertEqual(
            path,
            [
                [0.0, 100.0, -0.0],
                [100.0, 100.0, -0.0],
                [100.0, 100.0, -100.0],
            ],
        )

    def test_terrain_geometry_builds_connected_heightfield(self) -> None:
        """验证 3D 地形使用一张连续 mesh，而不是多个独立山体模型。"""

        geometry = TerrainGeometry()
        geometry.widthValue = DEFAULT_TERRAIN_SPAN_M
        geometry.depthValue = DEFAULT_TERRAIN_SPAN_M
        geometry.amplitudeValue = 760.0
        vertex_data = bytes(geometry.vertexData())
        y_values = [
            struct.unpack_from("<f", vertex_data, offset + 4)[0]
            for offset in range(0, len(vertex_data), geometry.stride())
        ]

        self.assertEqual(geometry.stride(), 48)
        self.assertGreater(geometry.vertexData().size(), 0)
        self.assertGreater(geometry.indexData().size(), 0)
        self.assertLessEqual(geometry.boundsMin().y(), 0.0)
        self.assertGreater(geometry.boundsMax().y(), 760.0)
        self.assertGreater(max(y_values) - min(y_values), 450.0)

    def test_terrain_geometry_consumes_layout_file_and_keeps_fallback(self) -> None:
        """验证 TerrainGeometry 有布局时使用新高度场，无布局时仍回退旧行为。"""

        # 几何层是非阻塞消费:先阻塞预热,再验证布局 mesh 构建。
        terrain_field_module.get_terrain_field(TERRAIN_LAYOUT_PATH, resolution=128)
        geometry = TerrainGeometry()
        geometry.resolutionValue = 128
        geometry.layoutFile = str(TERRAIN_LAYOUT_PATH)
        layout_vertex_size = geometry.vertexData().size()

        self.assertEqual(geometry.stride(), 48)
        self.assertEqual(layout_vertex_size, 128 * 128 * geometry.stride())
        self.assertGreater(geometry.indexData().size(), 0)
        self.assertGreater(geometry.generationTimeMs, 0.0)
        self.assertGreater(geometry.boundsMax().y(), 2000.0)

        geometry.layoutFile = ""
        geometry.widthValue = DEFAULT_TERRAIN_SPAN_M
        self.assertNotEqual(geometry.vertexData().size(), layout_vertex_size)

    def test_trail_ribbon_geometry_builds_single_continuous_mesh(self) -> None:
        """验证尾迹 ribbon 使用一张连续三角带，而不是离散点或分段圆柱。"""

        geometry = TrailRibbonGeometry()
        geometry.pathValue = json.dumps([[0.0, 100.0, 0.0], [60.0, 105.0, -20.0], [120.0, 108.0, -42.0]])
        geometry.widthValue = 32.0

        vertex_data = geometry.vertexData()
        index_data = geometry.indexData()

        self.assertEqual(geometry.stride(), 48)
        self.assertEqual(vertex_data.size(), 6 * geometry.stride())
        self.assertEqual(index_data.size(), 12 * 4)
        self.assertLessEqual(geometry.boundsMin().x(), -32.0)
        self.assertGreaterEqual(geometry.boundsMax().x(), 120.0)

    def test_route_ribbon_can_disable_trail_alpha_gradient(self) -> None:
        """验证航线虚线可关闭尾迹渐隐，避免单段 dash 内部颜色不一致。"""

        geometry = TrailRibbonGeometry()
        geometry.pathValue = json.dumps([[0.0, 100.0, 0.0], [120.0, 100.0, 0.0], [240.0, 100.0, 0.0]])

        trail_vertex_data = bytes(geometry.vertexData())
        first_trail_alpha = struct.unpack_from("<f", trail_vertex_data, 11 * 4)[0]
        last_trail_alpha = struct.unpack_from("<f", trail_vertex_data, 5 * geometry.stride() + 11 * 4)[0]
        self.assertLess(first_trail_alpha, last_trail_alpha)

        geometry.alphaMode = "solid"
        solid_vertex_data = bytes(geometry.vertexData())
        first_solid_alpha = struct.unpack_from("<f", solid_vertex_data, 11 * 4)[0]
        last_solid_alpha = struct.unpack_from("<f", solid_vertex_data, 5 * geometry.stride() + 11 * 4)[0]
        self.assertEqual(first_solid_alpha, 1.0)
        self.assertEqual(last_solid_alpha, 1.0)

    def test_diagonal_heading_yaw_matches_travel_direction_not_mirrored(self) -> None:
        """东北向斜航向的机头朝向应与航迹前方一致，不能被镜像到东南向。"""

        node = NodeState("A03", "leader", x=0.0, y=0.0, vx=60.0, vy=40.0, altitude=100.0)
        snapshot = self._snapshot()
        snapshot.nodes = [node]

        payload = build_scene_payload(snapshot)
        yaw_deg = payload["aircraft"][0]["yawDeg"]

        # Quick3D 里 z=-north，机头方向向量应为 (vx, -vy)，对应 yaw=atan2(vy, vx)。
        # 镜像 bug 会把符号反过来，产生 atan2(-vy, vx) 的相反象限结果。
        self.assertAlmostEqual(yaw_deg, 33.690067525979785, places=6)

    def test_disabled_obstacle_is_not_exported_to_scene(self) -> None:
        obstacle = ObstacleView("OFF", "circle", enabled=False, center_x=1.0, center_y=2.0, radius=3.0)

        payload = build_scene_payload(self._snapshot(), [obstacle], clearance_m=10.0)

        self.assertEqual(payload["counts"]["obstacles"], 0)
        self.assertEqual(payload["obstacles"], [])

    def test_current_route_falls_back_to_3d_route_dash_when_segments_are_absent(self) -> None:
        """验证 3D 航线与俯视图一致：无多航段时仍绘制当前目标航段虚线。"""

        snapshot = self._snapshot()
        snapshot.route = ReferenceRoute(0.0, 0.0, 100.0, 360.0, 0.0, 100.0)
        snapshot.route_segments = []

        payload = build_scene_payload(snapshot)

        self.assertEqual(payload["counts"]["routePoints"], 2)
        self.assertGreaterEqual(payload["counts"]["routeDashes"], 2)
        first_dash = json.loads(payload["routeDashes"][0]["pathValue"])
        self.assertEqual(first_dash[0], [0.0, 100.0, -0.0])

    def test_long_route_dash_count_is_capped(self) -> None:
        """验证超长航段不会生成无限增长的 QML dash delegate。"""

        snapshot = self._snapshot()
        snapshot.route_segments = [ReferenceRoute(0.0, 0.0, 100.0, 100000.0, 0.0, 100.0)]

        payload = build_scene_payload(snapshot)

        self.assertEqual(payload["counts"]["routeDashes"], MAX_ROUTE_DASHES_PER_SEGMENT)
        self.assertEqual(len(payload["routeDashes"]), MAX_ROUTE_DASHES_PER_SEGMENT)

    def test_qml_accepts_middle_button_for_focus_pan(self) -> None:
        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("Qt.MiddleButton", qml)
        self.assertIn("mouse.buttons & Qt.MiddleButton", qml)
        self.assertIn("function applyGroundPan(dx, dy)", qml)
        self.assertIn("function applyCameraDrag(dx, dy, pointerY)", qml)
        self.assertIn("const yawRadians = yaw * Math.PI / 180.0", qml)
        self.assertIn("focusX += (-dx * cosYaw - dy * sinYaw) * scale", qml)
        self.assertIn("focusZ += (dx * sinYaw - dy * cosYaw) * scale", qml)
        self.assertIn("const yawSign = pointerY < height / 2.0 ? 1.0 : -1.0", qml)
        self.assertIn("yaw += dx * 0.25 * yawSign", qml)
        self.assertIn("pitch = clampPitch(pitch - dy * 0.18)", qml)
        self.assertNotIn("root.focusX -= dx * root.distance / 1800.0", qml)
        self.assertNotIn("root.focusZ += dy * root.distance / 1800.0", qml)
        self.assertNotIn("root.yaw += dx * 0.25", qml)
        self.assertIn("TerrainGeometry", qml)
        # 顶点色只做按高度的平滑渐变，必须保留光照；历史碎斑来自噪声色 + NoLighting。
        self.assertIn("vertexColorsEnabled: true", qml)
        self.assertNotIn("PrincipledMaterial.NoLighting", qml)
        self.assertNotIn("hillModel", qml)
        self.assertIn("Math.min(50000", qml)
        self.assertIn("data.trailRibbons", qml)
        self.assertIn("property real nearViewWidthScale", qml)
        self.assertIn("property real aircraftVisualScale", qml)
        self.assertIn("property real routeDashWidthScale: nearViewWidthScale", qml)
        self.assertIn("property real trailWidthScale: Math.min(0.17, aircraftVisualScale * aircraftUnitWingspan / 5.0 / 44.0)", qml)
        self.assertIn("1800m 是默认自由视角量级", qml)
        self.assertIn("Math.max(0.25, Math.min(1.0, distance / 1800.0))", qml)
        self.assertIn("ListModel { id: routeDashModel }", qml)
        self.assertIn("ListModel { id: riskZoneModel }", qml)
        self.assertIn("data.riskZones", qml)
        self.assertIn("data.riskZoneLines", qml)
        self.assertIn("data.riskZoneBuffers", qml)
        self.assertIn('terrainGeometry.layoutFile = surface.layoutFile || ""', qml)
        self.assertIn("terrainGeometry.resolutionValue = surface.resolution || 641", qml)
        self.assertIn("data.routeDashes", qml)
        self.assertIn("model: routeDashModel", qml)
        self.assertIn("TrailRibbonGeometry", qml)
        self.assertIn("pathValue: model.pathValue", qml)
        self.assertIn("widthValue: model.widthValue * root.routeDashWidthScale", qml)
        self.assertIn("widthValue: model.widthValue * root.trailWidthScale", qml)
        self.assertIn('alphaMode: "solid"', qml)
        self.assertIn("function syncTrailModel", qml)
        self.assertNotIn("trailModel.clear()", qml)
        self.assertNotIn("data.trailPoints", qml)

    def test_aircraft_model_stays_recognizable_but_distance_visible(self) -> None:
        """飞机应按真实尺寸渲染无人机模型，并随相机距离自适应缩放以免缩远后消失。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("RuntimeLoader", qml)
        self.assertIn("property string aircraftModelSource: \"assets/BayraktarTB2.glb\"", qml)
        self.assertIn("source: Qt.resolvedUrl(root.aircraftModelSource)", qml)
        self.assertIn("property real aircraftVisualScale", qml)
        self.assertIn("aircraftBaseScale * Math.max(1.0, distance * 0.0207 / aircraftRealWingspanM)", qml)
        self.assertNotIn("assets/PredatorUAV.glb", qml)
        self.assertNotIn("Math.max(8.5, distance / 85.0)", qml)
        self.assertIn("scale: Qt.vector3d(root.aircraftVisualScale, root.aircraftVisualScale, root.aircraftVisualScale)", qml)

    def test_trail_width_is_one_fifth_of_near_view_aircraft_span(self) -> None:
        """近景尾迹宽度应约为飞机翼展 1/5，远景不应继续变粗。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("property real trailWidthScale: Math.min(0.17, aircraftVisualScale * aircraftUnitWingspan / 5.0 / 44.0)", qml)
        self.assertIn("飞机视觉缩放单独保持远景可辨识；尾迹近景按翼展 1/5 显示，远景不继续加粗。", qml)
        self.assertNotIn("property real trailWidthScale: nearViewWidthScale", qml)
        self.assertNotIn("property real trailWidthScale: aircraftVisualScale / (1800.0 / 85.0)", qml)


if __name__ == "__main__":
    unittest.main()
