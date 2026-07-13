"""3D 态势场景数据适配回归测试。"""

from __future__ import annotations

from collections import deque
from pathlib import Path
import json
import math
import struct
import time
import unittest
from unittest.mock import patch

import numpy as np
from PySide6.QtGui import QVector3D

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
        self.assertEqual(
            json.loads(trail_ribbon["pathValue"]),
            {
                "op": "reset",
                "generation": 0,
                "firstSequence": 0,
                "endSequence": 2,
                "points": [[1.0, 3.0, -2.0], [4.0, 6.0, -5.0]],
            },
        )
        self.assertEqual(
            (trail_ribbon["tipStartX"], trail_ribbon["tipStartY"], trail_ribbon["tipStartZ"]),
            (4.0, 6.0, -5.0),
        )
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

        surface = payload["terrain"]["surface"]
        risk_area = payload["terrainRiskAreas"][0]
        self.assertEqual(risk_area["id"], "OBS1")
        self.assertEqual(risk_area["kind"], "circle")
        self.assertEqual(risk_area["radius"], 30.0)
        self.assertEqual(risk_area["center"], [80.0 - surface["x"], -70.0 - surface["z"]])

    def test_payload_contains_blocked_route_with_red_color(self) -> None:
        """验证封锁航线拥有独立红色 payload，空数据不产生残留模型。"""

        snapshot = self._snapshot()
        snapshot.blocked_route_segments = [ReferenceRoute(-30.0, 0.0, 90.0, 0.0, 80.0, 110.0)]
        payload = build_scene_payload(snapshot)
        self.assertTrue(payload["blockedRoutePoints"])
        self.assertTrue(payload["blockedRouteDashes"])
        self.assertTrue(all(item["color"] == "#ff5a45" for item in payload["blockedRoutePoints"]))
        self.assertTrue(all(item["color"] == "#ff5a45" for item in payload["blockedRouteDashes"]))

        snapshot.blocked_route_segments = []
        cleared_payload = build_scene_payload(snapshot)
        self.assertEqual(cleared_payload["blockedRoutePoints"], [])
        self.assertEqual(cleared_payload["blockedRouteDashes"], [])

    def test_obstacle_risk_zones_only_include_enabled_and_empty_falls_back_to_layout(self) -> None:
        """验证风险区只跟随启用障碍；全禁用时清空，无避障数据时才回退布局。"""

        enabled = ObstacleView("启用圆", "circle", center_x=100.0, center_y=200.0, radius=80.0)
        disabled = ObstacleView("禁用圆", "circle", enabled=False, center_x=300.0, center_y=400.0, radius=50.0)
        zones = scene_data._risk_zones_from_obstacles([enabled, disabled], None)
        self.assertEqual([zone.zone_id for zone in zones], ["启用圆"])

        fallback = scene_data._layout_terrain_payload(str(TERRAIN_LAYOUT_PATH), [])
        assert fallback is not None
        self.assertEqual([zone["id"] for zone in fallback["riskZones"]], ["hazard_peak_west", "hazard_peak_east"])

        all_disabled = scene_data._layout_terrain_payload(str(TERRAIN_LAYOUT_PATH), [disabled])
        assert all_disabled is not None
        self.assertEqual(all_disabled["riskZones"], [])

        snapshot = self._snapshot()
        snapshot.terrain_display_file = str(TERRAIN_LAYOUT_PATH)
        obstacle_payload = build_scene_payload(snapshot, [enabled], clearance_m=20.0)
        self.assertEqual(len(obstacle_payload["riskZoneLines"]), 1)
        self.assertEqual(obstacle_payload["riskZoneBuffers"], [])
        boundary = obstacle_payload["riskZoneLines"][0]
        self.assertTrue(boundary["pulse"])
        boundary_points = json.loads(boundary["pathValue"])
        self.assertGreater(len(boundary_points), 24)
        self.assertEqual(boundary_points[0], boundary_points[-1])
        radii = [math.hypot(point[0] - 100.0, point[2] + 200.0) for point in boundary_points]
        self.assertTrue(all(abs(radius - 100.0) < 1e-6 for radius in radii))

    def test_obstacle_fill_payload_matches_boundary_outline_and_hugs_terrain(self) -> None:
        """验证危险区填充覆盖层：轮廓与边界同源、三角网合法、逐点贴合地形高度。"""

        enabled = ObstacleView("启用圆", "circle", center_x=100.0, center_y=200.0, radius=80.0)
        snapshot = self._snapshot()
        snapshot.terrain_display_file = str(TERRAIN_LAYOUT_PATH)
        payload = build_scene_payload(snapshot, [enabled], clearance_m=20.0)

        fills = payload["riskZoneFills"]
        self.assertEqual(len(fills), 1)
        self.assertEqual(fills[0]["color"], "#ff684f")
        mesh = json.loads(fills[0]["meshValue"])
        vertices = mesh["v"]
        triangles = mesh["t"]
        self.assertGreater(len(vertices), 8)
        self.assertGreater(len(triangles), 8)
        self.assertEqual(len(triangles) % 3, 0)
        self.assertTrue(all(0 <= index < len(vertices) for index in triangles))
        # 全部顶点必须落在安全区(半径 80+20)内，容差只放开厘米级坐标取整。
        radii = [math.hypot(point[0] - 100.0, point[2] + 200.0) for point in vertices]
        self.assertLessEqual(max(radii), 100.0 + 0.05)
        # 三角面总面积应接近圆面积，证明内部整格与边界裁剪块共同铺满了危险区。
        area = 0.0
        for start in range(0, len(triangles), 3):
            a, b, c = (vertices[triangles[start + offset]] for offset in range(3))
            area += abs((b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])) / 2.0
        self.assertGreater(area / (math.pi * 100.0 * 100.0), 0.9)
        # 每个顶点高度 = 地形显示高度 + 固定抬升，既不穿地也不悬空。
        surface = payload["terrain"]["surface"]
        field = scene_data._cached_terrain_field(surface["layoutFile"], surface["resolution"])
        for point in vertices[:32]:
            expected = scene_data._terrain_surface_height(
                surface, field, point[0], point[2], scene_data._FILL_HEIGHT_OFFSET_M
            )
            self.assertLess(abs(point[1] - expected), 8.0)

        # 无避障数据的旧场景不生成填充；全部禁用时同样应清空。
        self.assertEqual(build_scene_payload(snapshot)["riskZoneFills"], [])
        disabled = ObstacleView("禁用圆", "circle", enabled=False, center_x=0.0, center_y=0.0, radius=40.0)
        self.assertEqual(build_scene_payload(snapshot, [disabled])["riskZoneFills"], [])

    def test_obstacle_fill_mesh_is_cached_and_participates_in_static_key(self) -> None:
        """验证填充三角网走缓存复用，且进入 staticKey 参与静态重建判定。"""

        enabled = ObstacleView("启用圆", "circle", center_x=100.0, center_y=200.0, radius=80.0)
        snapshot = self._snapshot()
        first = build_scene_payload(snapshot, [enabled], clearance_m=20.0)
        second = build_scene_payload(snapshot, [enabled], clearance_m=20.0)
        # 同一障碍与地形版本必须复用同一份缓存字符串，避免 10Hz 快照重复三角化。
        self.assertIs(first["riskZoneFills"][0]["meshValue"], second["riskZoneFills"][0]["meshValue"])
        self.assertEqual(first["staticKey"], second["staticKey"])
        # 障碍尺寸变化 → 填充变化 → staticKey 必须翻转触发 QML 静态模型重建。
        larger = ObstacleView("启用圆", "circle", center_x=100.0, center_y=200.0, radius=120.0)
        changed = build_scene_payload(snapshot, [larger], clearance_m=20.0)
        self.assertNotEqual(first["staticKey"], changed["staticKey"])

    def test_risk_fill_geometry_uploads_valid_mesh_and_rejects_bad_input(self) -> None:
        """验证 RiskFillGeometry 上传合法三角网，坏 JSON 与越界索引降级为空几何。"""

        from src.ui.gui.situation3d.risk_fill_geometry import RiskFillGeometry

        geometry = RiskFillGeometry()
        mesh = {"v": [[0.0, 10.0, 0.0], [100.0, 12.0, 0.0], [0.0, 11.0, -100.0]], "t": [0, 1, 2]}
        geometry.meshValue = json.dumps(mesh)
        # 顶点布局 position(3)+normal(3)，共 24 字节一个顶点。
        self.assertEqual(len(geometry.vertexData()), 3 * 24)
        self.assertEqual(len(geometry.indexData()), 3 * 4)
        bounds_min = geometry.boundsMin()
        bounds_max = geometry.boundsMax()
        self.assertEqual(bounds_min.x(), 0.0)
        self.assertEqual(bounds_max.x(), 100.0)
        self.assertLess(bounds_min.y(), 10.0)
        self.assertGreater(bounds_max.y(), 12.0)

        geometry.meshValue = "not json"
        self.assertEqual(len(geometry.vertexData()), 0)
        geometry.meshValue = json.dumps({"v": [[0.0, 0.0, 0.0]], "t": [0, 0, 5]})
        self.assertEqual(len(geometry.vertexData()), 0)
        geometry.meshValue = json.dumps({"v": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], "t": [0, 1]})
        self.assertEqual(len(geometry.vertexData()), 0)

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

    def test_obstacle_boundary_samples_rendered_terrain_surface(self) -> None:
        """验证告警边界贴合显示沟脊，而不是埋在未增强的米制地形里。"""

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=128)
        assert field.display_heights_m is not None
        displacement = np.abs(field.display_heights_m - field.heights_m)
        row, column = np.unravel_index(int(np.argmax(displacement)), displacement.shape)
        east = field.center_east_m - field.width_m / 2.0 + field.width_m * column / (field.resolution - 1)
        north = field.center_north_m - field.depth_m / 2.0 + field.depth_m * row / (field.resolution - 1)

        measured = scene_data._terrain_surface_height({"mode": "layout"}, field, east, -north, 34.0)

        self.assertAlmostEqual(measured, float(field.display_heights_m[row, column]) + 34.0, places=5)
        self.assertGreater(abs(measured - (float(field.heights_m[row, column]) + 34.0)), 5.0)

    def test_display_relief_adds_bounded_rock_structure_without_changing_metric_height(self) -> None:
        """显示层应强化岩脊沟壑，同时保留独立且未改写的米制语义高度。"""

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=257)
        self.assertIsNotNone(field.display_heights_m)
        self.assertIsNotNone(field.display_normals)
        assert field.display_heights_m is not None
        assert field.display_normals is not None

        metric = field.heights_m
        display = field.display_heights_m
        displacement = display - metric
        mountain_mask = metric >= 420.0
        lowland_mask = metric <= 120.0
        metric_residual = np.abs(metric - terrain_field_module._box_blur(metric, 3, passes=2))
        display_residual = np.abs(display - terrain_field_module._box_blur(display, 3, passes=2))

        self.assertEqual(display.shape, metric.shape)
        self.assertEqual(field.display_normals.shape, (257, 257, 3))
        self.assertFalse(np.shares_memory(display, metric))
        self.assertLessEqual(float(np.max(np.abs(displacement))), 240.0)
        self.assertTrue(bool(np.any(lowland_mask)))
        np.testing.assert_array_equal(display[lowland_mask], metric[lowland_mask])
        self.assertGreater(float(np.percentile(np.abs(displacement[mountain_mask]), 75)), 12.0)
        self.assertGreater(
            float(np.percentile(display_residual[mountain_mask], 85)),
            float(np.percentile(metric_residual[mountain_mask], 85)) * 1.22,
        )

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
        self.assertFalse(payload["riskZoneLines"][0]["pulse"])
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

    def test_layout_terrain_palette_is_dark_layered_and_low_saturation(self) -> None:
        """验证正式布局地形呈深绿灰、岩石灰与冷暖坡面层次，而非浅绿塑料色。"""

        field = generate_terrain_field_from_file(TERRAIN_LAYOUT_PATH, resolution=257)
        linear = np.clip(field.colors.astype(np.float64), 0.0, 1.0)
        colors = np.where(
            linear <= 0.0031308,
            linear * 12.92,
            1.055 * np.power(linear, 1.0 / 2.4) - 0.055,
        )
        luminance_weights = np.array([0.2126, 0.7152, 0.0722], dtype=np.float64)
        heights = field.heights_m

        valley_colors = colors[heights <= 180.0]
        mid_colors = colors[(heights > 700.0) & (heights <= 1800.0)]
        high_colors = colors[heights > 1800.0]
        self.assertGreater(len(mid_colors), 200)
        self.assertGreater(len(high_colors), 40)
        self.assertLess(float((valley_colors @ luminance_weights).mean()), 0.29)
        self.assertLess(float((valley_colors[:, 1] - valley_colors[:, 0]).mean()), 0.07)
        self.assertLess(float((mid_colors.max(axis=1) - mid_colors.min(axis=1)).mean()), 0.09)
        self.assertLess(float((mid_colors @ luminance_weights).mean()), 0.33)

        high_luminance = high_colors @ luminance_weights
        lit_high = high_colors[high_luminance >= np.percentile(high_luminance, 75)]
        self.assertGreaterEqual(float(lit_high[:, 0].mean()), float(lit_high[:, 2].mean()))
        self.assertLess(float(lit_high[:, 1].mean() - lit_high[:, 0].mean()), 0.045)

        slope = np.sqrt(np.maximum(0.0, 1.0 / np.square(np.maximum(field.normals[:, :, 1], 1e-6)) - 1.0))
        steep_mid = colors[(heights > 700.0) & (slope > 0.35)]
        steep_luminance = steep_mid @ luminance_weights
        shadow = steep_mid[steep_luminance <= np.percentile(steep_luminance, 25)]
        light = steep_mid[steep_luminance >= np.percentile(steep_luminance, 75)]
        self.assertGreater(float(shadow[:, 2].mean() - shadow[:, 0].mean()), 0.03)
        self.assertGreater(float(np.percentile(steep_luminance, 75) - np.percentile(steep_luminance, 25)), 0.09)
        self.assertGreater(float((light @ luminance_weights).mean()), float((shadow @ luminance_weights).mean()) + 0.09)

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

    def test_release_scripts_bundle_terrain_detail_textures(self) -> None:
        """验证岩面法线与反照率贴图存在，并以 --add-data 形式进入双平台打包参数。"""

        import re

        project_root = Path(__file__).resolve().parents[2]
        asset_dir = project_root / "src" / "ui" / "gui" / "situation3d" / "qml" / "assets"
        for texture_name in ("terrain_detail_normal.png", "terrain_detail_albedo.png"):
            texture = asset_dir / texture_name
            self.assertTrue(texture.is_file(), texture_name)
            self.assertGreater(texture.stat().st_size, 0, texture_name)
            # 必须是真实的 --add-data 参数:源为该 PNG、目标为 QML 同级 assets 目录。
            pattern = re.compile(
                rf'--add-data\s+"src/ui/gui/situation3d/qml/assets/{re.escape(texture_name)}[;:]src/ui/gui/situation3d/qml/assets"'
            )
            for script_name in ("scripts/build_windows_full_release.ps1", "scripts/build_macos_full_release.sh"):
                script_text = (project_root / script_name).read_text(encoding="utf-8")
                self.assertRegex(script_text, pattern, f"{script_name}: {texture_name}")

    def test_ready_state_single_snapshot_still_swaps_in_layout_terrain(self) -> None:
        """复现负责人现场问题:READY 态只推一次快照(主窗口 tick 未启动),
        后台生成完成后地形必须自动替换为山地,而不是永远停在占位小图。"""

        from PySide6.QtCore import QObject
        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow

        app = QApplication.instance() or QApplication([])
        self._reset_terrain_caches()
        snapshot = self._snapshot()
        snapshot.terrain_display_file = str(TERRAIN_LAYOUT_PATH)
        window = Situation3DWindow()
        # 关键:只调用一次 set_snapshot,模拟 READY 态没有 100ms tick 的场景。
        window.set_snapshot(snapshot)
        app.processEvents()

        def layout_geometry_height() -> float:
            for child in window.quick_view.rootObject().findChildren(QObject):
                if child.metaObject().className().startswith("TerrainGeometry"):
                    return float(child.boundsMax().y())
            return -1.0

        deadline = time.monotonic() + 60.0
        height = layout_geometry_height()
        while height < 2000.0 and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.05)
            height = layout_geometry_height()
        self.assertGreater(height, 2000.0, "READY 态下山地未自动替换占位地形")
        payload = json.loads(window.bridge.sceneData())
        self.assertTrue(payload["terrain"]["surface"]["fieldReady"])
        window.close()

    def test_follow_view_only_tracks_moving_leader_focus(self) -> None:
        """谷地运动双帧测试:跟随只接管焦点,并与旋转、缩放状态相互独立。"""

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
        self.assertTrue(bool(root.property("followEnabled")))

        # 第二帧:长机沿峡谷东移,焦点必须跟上(Behavior 平滑,轮询等待收敛)。
        window.set_snapshot(canyon_snapshot(9600.0))
        deadline = time.monotonic() + 3.0
        while abs(float(root.property("focusX")) - 9600.0) > 5.0 and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.02)
        self.assertLess(abs(float(root.property("focusX")) - 9600.0), 5.0)

        # 跟随不再规定斜后方构图,旧视线净空公式已无固定前提;这里只锁定三轴解耦语义。
        distance = 2345.0
        root.setProperty("distance", distance)
        root.setTopView()
        app.processEvents()
        self.assertTrue(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("cameraMode")), "俯视")
        self.assertAlmostEqual(float(root.property("yaw")), 0.0)
        self.assertAlmostEqual(float(root.property("pitch")), -76.0)
        self.assertAlmostEqual(float(root.property("distance")), distance)

        # 缩放只改变 distance,逐帧刷新焦点时仍保持跟随。
        root.setProperty("distance", 1800.0)
        window.set_snapshot(canyon_snapshot(9800.0))
        app.processEvents()
        self.assertTrue(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("cameraMode")), "俯视")

        root.applyCameraDrag(20.0, 0.0, 100.0)
        self.assertTrue(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("cameraMode")), "自由")

        distance = float(root.property("distance"))
        root.setSideView()
        self.assertTrue(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("cameraMode")), "侧视")
        self.assertAlmostEqual(float(root.property("yaw")), -90.0)
        self.assertAlmostEqual(float(root.property("pitch")), -8.0)
        self.assertAlmostEqual(float(root.property("distance")), distance)

        root.setTopView()
        root.applyGroundPan(20.0, 0.0)
        self.assertFalse(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("cameraMode")), "俯视")

        root.setFollowView()
        self.assertTrue(bool(root.property("followEnabled")))
        root.setFollowView()
        self.assertFalse(bool(root.property("followEnabled")))
        self.assertEqual(str(root.property("followNodeId")), "")
        window.close()

    def test_follow_focus_matches_leader_during_qml_interpolation(self) -> None:
        """跟随焦点必须与长机共用展示时钟，不能在快照间相对前后抖动。"""

        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow

        app = QApplication.instance() or QApplication([])

        def moving_snapshot(time_s: float, east_m: float) -> Snapshot:
            """构造沿东向匀速运动的单长机快照。"""

            return Snapshot(
                time=time_s,
                duration=100.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[NodeState("A01", "leader", east_m, 0.0, 20.0, 0.0, altitude=900.0)],
                links=[],
                route_segments=[],
            )

        def process_until(predicate, timeout_s: float = 1.0) -> None:  # noqa: ANN001
            """轮询 Qt 事件直至动画进入指定状态。"""

            deadline = time.monotonic() + timeout_s
            while not predicate() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.002)
            app.processEvents()

        window = Situation3DWindow()
        try:
            window.show()
            window.set_snapshot(moving_snapshot(1.0, 100.0))
            root = window.quick_view.rootObject()
            process_until(lambda: float(root.property("presentationProgress")) >= 0.999)
            root.setFollowView()
            process_until(lambda: abs(float(root.property("focusX")) - 100.0) < 0.5)

            window.set_snapshot(moving_snapshot(2.0, 300.0))
            errors = []
            deadline = time.monotonic() + 1.0
            while float(root.property("presentationProgress")) < 0.999 and time.monotonic() < deadline:
                app.processEvents()
                progress = float(root.property("presentationProgress"))
                if 0.05 <= progress <= 0.95:
                    aircraft = root.currentAircraftPositions().toVariant()["A01"]
                    errors.append(
                        max(
                            abs(float(root.property("focusX")) - aircraft["x"]),
                            abs(float(root.property("focusY")) - aircraft["y"]),
                            abs(float(root.property("focusZ")) - aircraft["z"]),
                        )
                    )
                time.sleep(0.002)

            self.assertGreater(len(errors), 3)
            self.assertLess(max(errors), 0.5)
        finally:
            window.close()

    def test_aircraft_and_trail_tip_remain_coincident_during_qml_interpolation(self) -> None:
        """真实 QML 补间中途，飞机展示位置与可见尾迹末端必须逐帧重合。"""

        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow

        app = QApplication.instance() or QApplication([])

        def moving_snapshot(time_s: float, east_m: float) -> Snapshot:
            """构造沿东向运动的单机快照，尾迹真实末点始终等于飞机目标。"""

            trail = [
                TrailPoint(0.0, 0.0, 100.0, 0.0),
                TrailPoint(100.0, 0.0, 100.0, 1.0),
            ]
            if east_m > 100.0:
                trail.append(TrailPoint(east_m, 0.0, 100.0, time_s))
            return Snapshot(
                time=time_s,
                duration=100.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[NodeState("A01", "leader", east_m, 0.0, 20.0, 0.0, altitude=100.0, trail=trail)],
                links=[],
                route_segments=[],
            )

        def process_until(predicate, timeout_s: float = 1.0) -> None:  # noqa: ANN001
            """轮询 Qt 事件直至条件成立，避免依赖固定 sleep 命中动画中点。"""

            deadline = time.monotonic() + timeout_s
            while not predicate() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.002)
            app.processEvents()

        window = Situation3DWindow()
        try:
            window.show()
            window.set_snapshot(moving_snapshot(1.0, 100.0))
            root = window.quick_view.rootObject()
            process_until(lambda: float(root.property("presentationProgress")) >= 0.999)

            window.set_snapshot(moving_snapshot(2.0, 300.0))
            process_until(lambda: 0.2 <= float(root.property("presentationProgress")) <= 0.8)
            progress = float(root.property("presentationProgress"))
            self.assertGreaterEqual(progress, 0.2)
            self.assertLessEqual(progress, 0.8)

            aircraft = root.currentAircraftPositions().toVariant()["A01"]
            tip = root.currentTrailTipPositions().toVariant()["A01"]
            self.assertAlmostEqual(aircraft["x"], tip["x"], places=5)
            self.assertAlmostEqual(aircraft["y"], tip["y"], places=5)
            self.assertAlmostEqual(aircraft["z"], tip["z"], places=5)
        finally:
            window.close()

    def test_qml_presentation_queue_consumes_rapid_trail_deltas_in_order(self) -> None:
        """连续提前到帧必须有界排队并按序消费，不能丢 delta 或扩大真实尾迹容量。"""

        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow
        from src.ui.gui.trail_view_model import TrailBuffer

        app = QApplication.instance() or QApplication([])
        trail = TrailBuffer(capacity=8)

        def append_snapshot(index: int) -> Snapshot:
            """追加一个真实点并冻结本帧，供窗口级 delta 游标按顺序编码。"""

            east_m = float(index * 100)
            trail.append_position(east_m, 0.0, 100.0, float(index))
            return Snapshot(
                time=float(index),
                duration=100.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[
                    NodeState(
                        "A01",
                        "leader",
                        east_m,
                        0.0,
                        20.0,
                        0.0,
                        altitude=100.0,
                        trail=trail.snapshot(),
                    )
                ],
                links=[],
                route_segments=[],
            )

        def process_until(predicate, timeout_s: float = 2.0) -> None:  # noqa: ANN001
            """处理事件直至快速帧全部消费。"""

            deadline = time.monotonic() + timeout_s
            while not predicate() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.002)
            app.processEvents()

        window = Situation3DWindow()
        try:
            window.show()
            window.set_snapshot(append_snapshot(0))
            window.set_snapshot(append_snapshot(1))
            root = window.quick_view.rootObject()
            process_until(lambda: float(root.property("presentationProgress")) >= 0.999)
            apply_count = int(root.property("sceneApplyCount"))

            # 远快于 90ms 连续推四帧，覆盖容量 2 的排队与溢出完成当前目标分支。
            for index in range(2, 6):
                window.set_snapshot(append_snapshot(index))
                app.processEvents()

            def latest_delta_finished() -> bool:
                """判断最后场景消息已应用且共同补间完成。"""

                return bool(
                    str(root.property("sceneTime")) == "5.0s"
                    and float(root.property("presentationProgress")) >= 0.999
                )

            process_until(latest_delta_finished)
            aircraft = root.currentAircraftPositions().toVariant()["A01"]
            tip = root.currentTrailTipPositions().toVariant()["A01"]
            self.assertEqual(int(root.property("sceneApplyCount")), apply_count + 4)
            self.assertEqual(aircraft["x"], 500.0)
            self.assertEqual(tip["x"], 500.0)
            self.assertEqual(trail.capacity, 8)
            self.assertLessEqual(len(root.property("pendingSceneUpdates").toVariant()), 2)
        finally:
            window.close()

    def test_qml_presentation_queue_continues_after_empty_aircraft_frame(self) -> None:
        """动画后的空节点帧没有 onFinished 信号，也必须继续消费其后的非空消息。"""

        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow

        app = QApplication.instance() or QApplication([])

        def snapshot_at(time_s: float, east_m: float | None) -> Snapshot:
            """构造单机或空节点场景，复现配置切换期间的展示消息序列。"""

            nodes = []
            if east_m is not None:
                nodes = [
                    NodeState(
                        "A01",
                        "leader",
                        east_m,
                        0.0,
                        20.0,
                        0.0,
                        altitude=100.0,
                        trail=[TrailPoint(east_m, 0.0, 100.0, time_s)],
                    )
                ]
            return Snapshot(
                time=time_s,
                duration=100.0,
                step=0.1,
                run_state="READY",
                control_report="待命",
                disturbance="无",
                nodes=nodes,
                links=[],
            )

        def process_until(predicate, timeout_s: float = 2.0) -> None:  # noqa: ANN001
            """处理事件直到空帧后的最后消息已应用。"""

            deadline = time.monotonic() + timeout_s
            while not predicate() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.002)
            app.processEvents()

        window = Situation3DWindow()
        try:
            window.show()
            window.set_snapshot(snapshot_at(0.0, 0.0))
            root = window.quick_view.rootObject()
            process_until(lambda: float(root.property("presentationProgress")) >= 0.999)

            window.set_snapshot(snapshot_at(1.0, 100.0))
            app.processEvents()
            window.set_snapshot(snapshot_at(2.0, None))
            app.processEvents()
            window.set_snapshot(snapshot_at(3.0, 300.0))
            app.processEvents()
            process_until(
                lambda: str(root.property("sceneTime")) == "3.0s"
                and float(root.property("presentationProgress")) >= 0.999
            )

            self.assertEqual(str(root.property("sceneTime")), "3.0s")
            self.assertEqual(root.property("pendingSceneUpdates").toVariant(), [])
            self.assertEqual(root.currentAircraftPositions().toVariant()["A01"]["x"], 300.0)
        finally:
            window.close()

    def test_reset_camera_does_not_replay_latest_scene_outside_presentation_queue(self) -> None:
        """重置相机只能读取相机字段，不能绕过 FIFO 重放最新场景并打乱尾迹 delta 游标。"""

        from PySide6.QtCore import QMetaObject
        from PySide6.QtWidgets import QApplication
        from src.ui.gui.situation3d.window import Situation3DWindow
        from src.ui.gui.trail_view_model import TrailBuffer

        app = QApplication.instance() or QApplication([])
        trail = TrailBuffer(capacity=8)

        def append_snapshot(index: int) -> Snapshot:
            """追加一帧稳定尾迹，构造可验证应用次数的连续场景消息。"""

            east_m = float(index * 100)
            trail.append_position(east_m, 0.0, 100.0, float(index))
            return Snapshot(
                time=float(index),
                duration=100.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[
                    NodeState(
                        "A01",
                        "leader",
                        east_m,
                        0.0,
                        20.0,
                        0.0,
                        altitude=100.0,
                        trail=trail.snapshot(),
                    )
                ],
                links=[],
                route_segments=[],
            )

        def process_until(predicate, timeout_s: float = 2.0) -> None:  # noqa: ANN001
            """处理事件直至 FIFO 消费完成。"""

            deadline = time.monotonic() + timeout_s
            while not predicate() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.002)
            app.processEvents()

        window = Situation3DWindow()
        try:
            window.show()
            window.set_snapshot(append_snapshot(0))
            window.set_snapshot(append_snapshot(1))
            root = window.quick_view.rootObject()
            process_until(
                lambda: str(root.property("sceneTime")) == "1.0s"
                and float(root.property("presentationProgress")) >= 0.999
            )
            baseline_count = int(root.property("sceneApplyCount"))

            for index in range(2, 5):
                window.set_snapshot(append_snapshot(index))
                app.processEvents()
            self.assertTrue(QMetaObject.invokeMethod(root, "resetCamera"))

            process_until(
                lambda: str(root.property("sceneTime")) == "4.0s"
                and float(root.property("presentationProgress")) >= 0.999
            )
            self.assertEqual(int(root.property("sceneApplyCount")), baseline_count + 3)
            self.assertEqual(root.property("pendingSceneUpdates").toVariant(), [])
        finally:
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

    def test_trail_payload_uses_queue_points_without_resampling_or_smoothing(self) -> None:
        """尾迹必须逐点消费稳定队列，不能按总长度重抽样或重做整条平滑。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [
            TrailPoint(0.0, 0.0, 100.0, 0.0),
            TrailPoint(100.0, 0.0, 100.0, 1.0),
            TrailPoint(100.0, 100.0, 100.0, 2.0),
            TrailPoint(200.0, 100.0, 100.0, 3.0),
        ]

        payload = build_scene_payload(snapshot)
        stream = json.loads(payload["trailRibbons"][0]["pathValue"])
        self.assertEqual(
            stream["points"],
            [
                [0.0, 100.0, -0.0],
                [100.0, 100.0, -0.0],
                [100.0, 100.0, -100.0],
                [200.0, 100.0, -100.0],
            ],
        )
        self.assertEqual(stream["firstSequence"], 0)
        self.assertEqual(stream["endSequence"], 4)
        self.assertEqual(stream["op"], "reset")

    def test_trail_append_does_not_move_existing_payload_points(self) -> None:
        """追加队尾点后，已输出的历史坐标必须保持为新点列的稳定前缀。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [
            TrailPoint(float(index), float(index % 3), 100.0, float(index))
            for index in range(40)
        ]
        before = json.loads(build_scene_payload(snapshot)["trailRibbons"][0]["pathValue"])

        snapshot.nodes[0].trail.append(TrailPoint(40.0, 2.0, 100.0, 40.0))
        after = json.loads(build_scene_payload(snapshot)["trailRibbons"][0]["pathValue"])

        self.assertEqual(before["points"], after["points"][:-1])
        self.assertEqual(len(after["points"]), 41)

    def test_trail_payload_separates_stable_history_from_live_tip(self) -> None:
        """固定时钟队列全部进入稳定网格，活动末段从队尾连接飞机实时位置。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [
            TrailPoint(0.0, 0.0, 100.0, 0.0),
            TrailPoint(100.0, 10.0, 110.0, 1.0),
            TrailPoint(180.0, 35.0, 120.0, 2.0),
            TrailPoint(240.0, 80.0, 130.0, 3.0),
        ]

        ribbon = build_scene_payload(snapshot)["trailRibbons"][0]
        stream = json.loads(ribbon["pathValue"])

        self.assertEqual(
            stream,
            {
                "op": "reset",
                "generation": 0,
                "firstSequence": 0,
                "endSequence": 4,
                "points": [
                    [0.0, 100.0, -0.0],
                    [100.0, 110.0, -10.0],
                    [180.0, 120.0, -35.0],
                    [240.0, 130.0, -80.0],
                ],
            },
        )
        self.assertEqual(
            {key: ribbon[key] for key in ("tipPreviousX", "tipPreviousY", "tipPreviousZ")},
            {"tipPreviousX": 180.0, "tipPreviousY": 120.0, "tipPreviousZ": -35.0},
        )
        self.assertEqual(
            {key: ribbon[key] for key in ("tipStartX", "tipStartY", "tipStartZ")},
            {"tipStartX": 240.0, "tipStartY": 130.0, "tipStartZ": -80.0},
        )

    def test_single_stable_trail_point_still_builds_live_tip_segment(self) -> None:
        """刚启用尾迹时，一个稳定点也必须能通过固定小网格连接当前飞机。"""

        snapshot = self._snapshot()
        snapshot.nodes[0].trail = [TrailPoint(12.0, 34.0, 120.0, 0.0)]

        ribbon = build_scene_payload(snapshot)["trailRibbons"][0]
        stream = json.loads(ribbon["pathValue"])

        self.assertEqual(stream["points"], [[12.0, 120.0, -34.0]])
        self.assertEqual(stream["endSequence"], 1)
        self.assertEqual(
            (ribbon["tipPreviousX"], ribbon["tipPreviousY"], ribbon["tipPreviousZ"]),
            (12.0, 120.0, -34.0),
        )
        self.assertEqual(
            (ribbon["tipStartX"], ribbon["tipStartY"], ribbon["tipStartZ"]),
            (12.0, 120.0, -34.0),
        )

    def test_trail_payload_state_only_serializes_queue_delta_after_reset(self) -> None:
        """窗口级游标首帧发全量，后续只发弹头数量和新增点，避免每帧扫描全部历史。"""

        class StableTrail(list):
            """为数据桥测试提供与正式 TrailSnapshot 相同的稳定游标。"""

            generation = 3
            first_sequence = 100
            end_sequence = 104

            def __init__(self, points) -> None:  # noqa: ANN001
                """初始化点列并记录全量迭代次数。"""

                super().__init__(points)
                self.full_iteration_count = 0

            def __iter__(self):  # noqa: ANN204
                """记录全量迭代；delta 热路径不应调用本方法。"""

                self.full_iteration_count += 1
                return super().__iter__()

        snapshot = self._snapshot()
        trail = StableTrail(
            [TrailPoint(float(index), 0.0, 100.0, float(index)) for index in range(4)]
        )
        snapshot.nodes[0].trail = trail
        state = scene_data.TrailPayloadState()

        initial = json.loads(
            build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]["pathValue"]
        )
        reset_iterations = trail.full_iteration_count
        trail.pop(0)
        trail.append(TrailPoint(4.0, 0.0, 100.0, 4.0))
        trail.first_sequence = 101
        trail.end_sequence = 105
        delta = json.loads(
            build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]["pathValue"]
        )
        committed = json.loads(
            build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]["pathValue"]
        )

        self.assertEqual(initial["op"], "reset")
        self.assertEqual(len(initial["points"]), 4)
        self.assertEqual(trail.full_iteration_count, reset_iterations)
        self.assertEqual(
            delta,
            {
                "op": "delta",
                "generation": 3,
                "firstSequence": 101,
                "endSequence": 104,
                "removedCount": 1,
                "addedPoints": [],
            },
        )
        self.assertEqual(
            committed,
            {
                "op": "delta",
                "generation": 3,
                "firstSequence": 101,
                "endSequence": 105,
                "removedCount": 0,
                "addedPoints": [[4.0, 100.0, -0.0]],
            },
        )

    def test_rapid_batch_waits_one_presentation_frame_before_entering_3d_history(self) -> None:
        """高倍频批量点只能在飞机到达上一目标后固化，历史队尾不得抢到机头前。"""

        from src.ui.gui.trail_view_model import TrailBuffer

        snapshot = self._snapshot()
        trail = TrailBuffer(capacity=32)
        trail.append_position(0.0, 0.0, 100.0, 0.0)
        snapshot.time = 0.0
        snapshot.nodes[0].x = 0.0
        snapshot.nodes[0].trail = trail.snapshot()
        state = scene_data.TrailPayloadState()
        initial = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        for index in range(1, 8):
            trail.append_position(index * 10.0, 0.0, 100.0, index * 0.1)
        snapshot.time = 0.7
        snapshot.nodes[0].x = 70.0
        snapshot.nodes[0].trail = trail.snapshot()
        receiving = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        for index in range(8, 15):
            trail.append_position(index * 10.0, 0.0, 100.0, index * 0.1)
        snapshot.time = 1.4
        snapshot.nodes[0].x = 140.0
        snapshot.nodes[0].trail = trail.snapshot()
        committed = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        self.assertEqual(json.loads(initial["pathValue"])["endSequence"], 1)
        self.assertEqual(json.loads(receiving["pathValue"])["endSequence"], 1)
        self.assertEqual(receiving["tipStartX"], 0.0)
        self.assertEqual(json.loads(committed["pathValue"])["endSequence"], 8)
        self.assertEqual(committed["tipStartX"], 70.0)

    def test_3d_live_tip_keeps_previous_anchor_when_queue_head_overtakes_horizon(self) -> None:
        """上一队尾被容量淘汰时历史可清空，但活动末段仍必须从上一展示锚点起步。"""

        from src.ui.gui.trail_view_model import TrailBuffer

        snapshot = self._snapshot()
        trail = TrailBuffer(capacity=1)
        trail.append_position(0.0, 0.0, 100.0, 0.0)
        snapshot.nodes[0].x = 0.0
        snapshot.nodes[0].trail = trail.snapshot()
        state = scene_data.TrailPayloadState()
        initial = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        trail.append_position(10.0, 0.0, 100.0, 0.1)
        snapshot.nodes[0].x = 10.0
        snapshot.nodes[0].trail = trail.snapshot()
        receiving = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        trail.append_position(20.0, 0.0, 100.0, 0.2)
        snapshot.nodes[0].x = 20.0
        snapshot.nodes[0].trail = trail.snapshot()
        following = build_scene_payload(snapshot, trail_state=state)["trailRibbons"][0]

        receiving_stream = json.loads(receiving["pathValue"])
        self.assertEqual(receiving_stream["endSequence"], 1)
        self.assertEqual(receiving_stream["addedPoints"], [])
        self.assertEqual(receiving["tipStartX"], 0.0)
        self.assertEqual(json.loads(following["pathValue"])["endSequence"], 2)
        self.assertEqual(following["tipStartX"], 10.0)
        history = TrailRibbonGeometry()
        history.pathValue = initial["pathValue"]
        history.pathValue = receiving["pathValue"]
        self.assertEqual(list(history._stream_points), [])

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

        self.assertEqual(geometry.stride(), 72)
        self.assertGreater(geometry.vertexData().size(), 0)
        self.assertGreater(geometry.indexData().size(), 0)
        self.assertLessEqual(geometry.boundsMin().y(), 0.0)
        self.assertGreater(geometry.boundsMax().y(), 760.0)
        self.assertGreater(max(y_values) - min(y_values), 450.0)

    def test_procedural_terrain_tangent_basis_follows_uv_directions(self) -> None:
        """占位地形的切线与副切线应分别跟随纹理坐标 u/v 的正方向。"""

        geometry = TerrainGeometry()
        vertices = np.frombuffer(bytes(geometry.vertexData()), dtype="<f4").reshape(-1, 18)
        column_count = int(np.unique(vertices[:, 6]).size)
        row_count = int(np.unique(vertices[:, 7]).size)
        self.assertEqual(row_count * column_count, len(vertices))
        grid = vertices.reshape(row_count, column_count, 18)

        interior = grid[1:-1, 1:-1]
        slope = np.linalg.norm(interior[:, :, (3, 5)], axis=2)
        row, column = np.unravel_index(int(np.argmax(slope)), slope.shape)
        normal = interior[row, column, 3:6].astype(np.float64)
        tangent = interior[row, column, 8:11].astype(np.float64)
        binormal = interior[row, column, 11:14].astype(np.float64)
        du = (grid[1:-1, 2:, :3] - grid[1:-1, :-2, :3])[row, column].astype(np.float64)
        dv = (grid[2:, 1:-1, :3] - grid[:-2, 1:-1, :3])[row, column].astype(np.float64)
        du /= np.linalg.norm(du)
        dv /= np.linalg.norm(dv)

        self.assertAlmostEqual(float(np.linalg.norm(normal)), 1.0, places=4)
        self.assertAlmostEqual(float(np.linalg.norm(tangent)), 1.0, places=4)
        self.assertAlmostEqual(float(np.linalg.norm(binormal)), 1.0, places=4)
        self.assertAlmostEqual(float(np.dot(normal, tangent)), 0.0, places=4)
        self.assertAlmostEqual(float(np.dot(normal, binormal)), 0.0, places=4)
        self.assertAlmostEqual(float(np.dot(tangent, binormal)), 0.0, places=4)
        self.assertGreater(float(np.dot(du, tangent)), 0.99)
        self.assertGreater(float(np.dot(dv, binormal)), 0.95)

    def test_procedural_terrain_fallback_uses_matching_rock_palette(self) -> None:
        """验证无布局占位地形也使用低饱和深色岩土地貌，避免加载切换时闪回浅绿色。"""

        geometry = TerrainGeometry()
        geometry.widthValue = DEFAULT_TERRAIN_SPAN_M
        geometry.depthValue = DEFAULT_TERRAIN_SPAN_M
        geometry.amplitudeValue = 760.0
        vertex_data = bytes(geometry.vertexData())
        samples = []
        for offset in range(0, len(vertex_data), geometry.stride()):
            height = struct.unpack_from("<f", vertex_data, offset + 4)[0]
            linear = np.array(struct.unpack_from("<fff", vertex_data, offset + 56), dtype=np.float64)
            color = np.where(
                linear <= 0.0031308,
                linear * 12.92,
                1.055 * np.power(linear, 1.0 / 2.4) - 0.055,
            )
            samples.append((height, color))

        low = np.array([color for height, color in samples if height <= 120.0])
        high = np.array([color for height, color in samples if height >= 560.0])
        luminance_weights = np.array([0.2126, 0.7152, 0.0722], dtype=np.float64)
        self.assertGreater(len(low), 100)
        self.assertGreater(len(high), 20)
        self.assertLess(float((low @ luminance_weights).mean()), 0.28)
        self.assertLess(float((low[:, 1] - low[:, 0]).mean()), 0.075)
        self.assertLess(float((high.max(axis=1) - high.min(axis=1)).mean()), 0.10)
        self.assertGreater(float((high @ luminance_weights).mean()), float((low @ luminance_weights).mean()) + 0.10)

    def test_terrain_geometry_tints_exact_polygon_without_coloring_its_bounding_box(self) -> None:
        """验证贴地风险色遵循多边形本体，不退化成外接圆或轴对齐包围盒。"""

        baseline = TerrainGeometry()
        baseline.widthValue = 400.0
        baseline.depthValue = 400.0
        tinted = TerrainGeometry()
        tinted.widthValue = 400.0
        tinted.depthValue = 400.0
        tinted.riskAreasValue = json.dumps(
            [
                {
                    "id": "三角障碍",
                    "kind": "polygon",
                    "points": [[0.0, 0.0], [100.0, 0.0], [0.0, 100.0]],
                    "clearance": 0.0,
                }
            ],
            ensure_ascii=False,
        )

        baseline_vertices = np.frombuffer(bytes(baseline.vertexData()), dtype="<f4").reshape(-1, 18)
        tinted_vertices = np.frombuffer(bytes(tinted.vertexData()), dtype="<f4").reshape(-1, 18)

        def nearest_index(east: float, quick_z: float) -> int:
            """返回最靠近目标平面坐标的网格顶点下标。"""

            distance_sq = (tinted_vertices[:, 0] - east) ** 2 + (tinted_vertices[:, 2] - quick_z) ** 2
            return int(np.argmin(distance_sq))

        inside_index = nearest_index(20.0, 20.0)
        outside_index = nearest_index(80.0, 80.0)
        self.assertGreater(
            float(np.linalg.norm(tinted_vertices[inside_index, 14:17] - baseline_vertices[inside_index, 14:17])),
            0.01,
        )
        np.testing.assert_allclose(
            tinted_vertices[outside_index, 14:17],
            baseline_vertices[outside_index, 14:17],
            atol=1e-6,
        )

    def test_terrain_geometry_consumes_layout_file_and_keeps_fallback(self) -> None:
        """验证 TerrainGeometry 有布局时使用新高度场，无布局时仍回退旧行为。"""

        # 几何层是非阻塞消费:先阻塞预热,再验证布局 mesh 构建。
        terrain_field_module.get_terrain_field(TERRAIN_LAYOUT_PATH, resolution=128)
        geometry = TerrainGeometry()
        geometry.resolutionValue = 128
        geometry.layoutFile = str(TERRAIN_LAYOUT_PATH)
        layout_vertex_size = geometry.vertexData().size()

        self.assertEqual(geometry.stride(), 72)
        self.assertEqual(layout_vertex_size, 128 * 128 * geometry.stride())
        self.assertGreater(geometry.indexData().size(), 0)
        self.assertGreater(geometry.generationTimeMs, 0.0)
        self.assertGreater(geometry.boundsMax().y(), 2000.0)

        geometry.layoutFile = ""
        geometry.widthValue = DEFAULT_TERRAIN_SPAN_M
        self.assertNotEqual(geometry.vertexData().size(), layout_vertex_size)

    def test_layout_geometry_uses_display_relief_and_complete_tangent_basis(self) -> None:
        """正式地形网格应消费显示高度，并提供法线贴图所需的正交切线基。"""

        field = terrain_field_module.get_terrain_field(TERRAIN_LAYOUT_PATH, resolution=128)
        assert field.display_heights_m is not None
        geometry = TerrainGeometry()
        geometry.resolutionValue = 128
        geometry.layoutFile = str(TERRAIN_LAYOUT_PATH)
        displacement = np.abs(field.display_heights_m - field.heights_m)
        vertex_index = int(np.argmax(displacement))
        values = struct.unpack_from("<18f", bytes(geometry.vertexData()), vertex_index * geometry.stride())

        normal = np.array(values[3:6])
        tangent = np.array(values[8:11])
        binormal = np.array(values[11:14])
        self.assertEqual(geometry.stride(), 72)
        self.assertAlmostEqual(values[1], float(field.display_heights_m.reshape(-1)[vertex_index]), places=3)
        self.assertGreater(float(displacement.reshape(-1)[vertex_index]), 1.0)
        self.assertAlmostEqual(float(np.linalg.norm(normal)), 1.0, places=4)
        self.assertAlmostEqual(float(np.linalg.norm(tangent)), 1.0, places=4)
        self.assertAlmostEqual(float(np.linalg.norm(binormal)), 1.0, places=4)
        self.assertAlmostEqual(float(np.dot(normal, tangent)), 0.0, places=4)
        self.assertAlmostEqual(float(np.dot(normal, binormal)), 0.0, places=4)
        self.assertAlmostEqual(float(np.dot(tangent, binormal)), 0.0, places=4)

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

    def test_trail_ribbon_stream_updates_append_and_head_eviction_incrementally(self) -> None:
        """流式尾迹追加与弹头只改局部缓冲，不能反复清空并重建整张 mesh。"""

        def stream(first: int, points: list[list[float]]) -> str:
            return json.dumps(
                {
                    "generation": 7,
                    "firstSequence": first,
                    "endSequence": first + len(points),
                    "points": points,
                }
            )

        geometry = TrailRibbonGeometry()
        points = [[float(index * 20), 100.0, float(-(index % 2) * 8)] for index in range(6)]
        geometry.pathValue = stream(0, points)
        rebuilds = geometry.fullRebuildCount
        increments = geometry.incrementalUpdateCount
        first_segment = bytes(geometry.vertexData())[: 5 * geometry.stride()]

        points.append([120.0, 100.0, 0.0])
        geometry.pathValue = stream(0, points)

        self.assertEqual(geometry.fullRebuildCount, rebuilds)
        self.assertGreater(geometry.incrementalUpdateCount, increments)
        self.assertEqual(bytes(geometry.vertexData())[: 5 * geometry.stride()], first_segment)

        # 弹出 A、追加 H 后，中间 C-D 段只允许更新渐隐 alpha，坐标和索引不能动。
        middle_offset = 2 * 5 * geometry.stride()
        middle_before = bytes(geometry.vertexData())
        middle_positions_before = tuple(
            struct.unpack_from("<fff", middle_before, middle_offset + index * geometry.stride())
            for index in range(5)
        )
        middle_indices_before = bytes(geometry.indexData())[2 * 9 * 4 : 3 * 9 * 4]
        shifted = points[1:] + [[140.0, 100.0, -8.0]]
        geometry.pathValue = stream(1, shifted)

        self.assertEqual(geometry.fullRebuildCount, rebuilds)
        self.assertEqual(
            tuple(
                struct.unpack_from("<fff", bytes(geometry.vertexData()), middle_offset + index * geometry.stride())
                for index in range(5)
            ),
            middle_positions_before,
        )
        self.assertEqual(bytes(geometry.indexData())[2 * 9 * 4 : 3 * 9 * 4], middle_indices_before)

    def test_live_tip_uses_fixed_small_geometry_without_touching_history_mesh(self) -> None:
        """60Hz 补间只能更新独立定长小网格，历史顶点和索引在整个补间期必须逐字节不变。"""

        from src.ui.gui.situation3d.trail_tip_geometry import TrailTipGeometry

        history = TrailRibbonGeometry()
        history.pathValue = json.dumps(
            {
                "op": "reset",
                "generation": 14,
                "firstSequence": 0,
                "endSequence": 3,
                "points": [[0.0, 100.0, 0.0], [100.0, 100.0, 0.0], [180.0, 100.0, -20.0]],
            }
        )
        history_vertices = bytes(history.vertexData())
        history_indices = bytes(history.indexData())
        tip = TrailTipGeometry()
        tip.previousPosition = QVector3D(100.0, 100.0, 0.0)
        tip.startPosition = QVector3D(180.0, 100.0, -20.0)
        initial_vertex_bytes = tip.vertexData().size()
        initial_index_bytes = tip.indexData().size()

        for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
            tip.endPosition = QVector3D(180.0 + 80.0 * ratio, 100.0, -20.0 - 60.0 * ratio)
            self.assertEqual(tip.vertexData().size(), initial_vertex_bytes)
            self.assertEqual(tip.indexData().size(), initial_index_bytes)

        self.assertEqual(history.metaObject().indexOfProperty("tipPosition"), -1)
        self.assertEqual(bytes(history.vertexData()), history_vertices)
        self.assertEqual(bytes(history.indexData()), history_indices)
        self.assertEqual(initial_vertex_bytes, 6 * history.stride())
        self.assertEqual(initial_index_bytes, 9 * 4)

    def test_live_tip_bevel_closes_turn_against_stable_history_cap(self) -> None:
        """活动末段转弯时必须以定长 bevel 三角连接历史平头端帽，不能留下碎冰状楔形裂口。"""

        from src.ui.gui.situation3d.trail_tip_geometry import TrailTipGeometry

        history = TrailRibbonGeometry()
        history.widthValue = 20.0
        history.pathValue = json.dumps(
            {
                "op": "reset",
                "generation": 15,
                "firstSequence": 0,
                "endSequence": 2,
                "points": [[0.0, 100.0, 0.0], [100.0, 100.0, 0.0]],
            }
        )
        history_data = bytes(history.vertexData())
        stable_end_edges = {
            struct.unpack_from("<fff", history_data, vertex_index * history.stride())
            for vertex_index in (2, 3)
        }

        tip = TrailTipGeometry()
        tip.widthValue = 20.0
        tip.previousPosition = QVector3D(0.0, 100.0, 0.0)
        tip.startPosition = QVector3D(100.0, 100.0, 0.0)
        tip.endPosition = QVector3D(100.0, 100.0, -100.0)
        tip_data = bytes(tip.vertexData())
        positions = [
            struct.unpack_from("<fff", tip_data, index * tip.stride())
            for index in range(6)
        ]
        body_indices = struct.unpack_from("<IIIIII", bytes(tip.indexData()), 0)
        join_indices = struct.unpack_from("<III", bytes(tip.indexData()), 6 * 4)

        self.assertEqual(body_indices, (0, 2, 1, 1, 2, 3))
        self.assertIn(positions[join_indices[0]], stable_end_edges)
        self.assertEqual(positions[join_indices[2]], (100.0, 100.0, 0.0))
        self.assertGreater(len(set(join_indices)), 1)

    def test_trail_ribbon_stream_reused_slot_indices_use_physical_vertex_base(self) -> None:
        """弹头后复用非零段槽时，主体索引必须指向该槽自己的五个顶点。"""

        geometry = TrailRibbonGeometry()
        points = [[float(index * 20), 100.0, 0.0] for index in range(5)]
        geometry.pathValue = json.dumps(
            {"op": "reset", "generation": 4, "firstSequence": 0, "endSequence": 5, "points": points}
        )
        geometry.pathValue = json.dumps(
            {
                "op": "delta",
                "generation": 4,
                "firstSequence": 2,
                "endSequence": 7,
                "removedCount": 2,
                "addedPoints": [[100.0, 100.0, 0.0], [120.0, 100.0, 0.0]],
            }
        )

        # 两个回收槽按 0、1 顺序复用；检查第二个新段所在物理槽 1。
        index_data = bytes(geometry.indexData())
        body_indices = struct.unpack_from("<IIIIII", index_data, 1 * 9 * 4)
        self.assertTrue(all(5 <= index <= 8 for index in body_indices), body_indices)

    def test_trail_ribbon_stream_uses_deques_for_hot_path_head_eviction(self) -> None:
        """流式点列和段槽必须使用 deque，禁止在稳态弹头时搬移整个 list。"""

        geometry = TrailRibbonGeometry()
        geometry.pathValue = json.dumps(
            {
                "op": "reset",
                "generation": 8,
                "firstSequence": 0,
                "endSequence": 3,
                "points": [[0.0, 100.0, 0.0], [20.0, 100.0, 0.0], [40.0, 100.0, 0.0]],
            }
        )

        self.assertIsInstance(geometry._stream_points, deque)
        self.assertIsInstance(geometry._stream_segment_slots, deque)

    def test_trail_ribbon_delta_does_not_scan_all_points_for_bounds(self) -> None:
        """稳态 delta 只能扩张保守包围盒，不能调用全点包围盒扫描。"""

        class BoundsCountingGeometry(TrailRibbonGeometry):
            """记录完整包围盒扫描次数。"""

            def __init__(self) -> None:
                """先初始化计数，再让基类构造空几何。"""

                self.bounds_scan_count = 0
                super().__init__()

            def _apply_bounds(self, points) -> None:  # noqa: ANN001
                """统计全量扫描并委托基类实现。"""

                self.bounds_scan_count += 1
                super()._apply_bounds(points)

        geometry = BoundsCountingGeometry()
        geometry.pathValue = json.dumps(
            {
                "op": "reset",
                "generation": 9,
                "firstSequence": 0,
                "endSequence": 4,
                "points": [[0.0, 100.0, 0.0], [20.0, 100.0, 0.0], [40.0, 100.0, 0.0], [60.0, 100.0, 0.0]],
            }
        )
        scans = geometry.bounds_scan_count
        geometry.pathValue = json.dumps(
            {
                "op": "delta",
                "generation": 9,
                "firstSequence": 1,
                "endSequence": 5,
                "removedCount": 1,
                "addedPoints": [[80.0, 100.0, 0.0]],
            }
        )

        self.assertEqual(geometry.bounds_scan_count, scans)

    def test_trail_ribbon_stream_keeps_bounded_head_to_tail_alpha_fade(self) -> None:
        """流式尾迹仍须队首更淡、队尾更清晰，不能为增量更新退化为整条同透明度。"""

        geometry = TrailRibbonGeometry()
        points = [[float(index * 20), 100.0, 0.0] for index in range(64)]
        geometry.pathValue = json.dumps(
            {"op": "reset", "generation": 5, "firstSequence": 0, "endSequence": 64, "points": points}
        )

        vertex_data = bytes(geometry.vertexData())
        first_alpha = struct.unpack_from("<f", vertex_data, 11 * 4)[0]
        middle_slot = 31
        middle_alpha_offset = middle_slot * 5 * geometry.stride() + 11 * 4
        middle_alpha = struct.unpack_from("<f", vertex_data, middle_alpha_offset)[0]
        last_slot = len(points) - 2
        last_alpha_offset = last_slot * 5 * geometry.stride() + 11 * 4
        last_alpha = struct.unpack_from("<f", vertex_data, last_alpha_offset)[0]
        self.assertLess(first_alpha, middle_alpha)
        self.assertLess(middle_alpha, last_alpha)

    def test_trail_ribbon_stream_middle_mutation_forces_bounded_rebuild(self) -> None:
        """只有队列中段被篡改等非追加结构变化，才允许执行一次有界全重建。"""

        geometry = TrailRibbonGeometry()
        stream = {
            "generation": 2,
            "firstSequence": 10,
            "endSequence": 14,
            "points": [[0.0, 100.0, 0.0], [20.0, 100.0, 0.0], [40.0, 100.0, 0.0], [60.0, 100.0, 0.0]],
        }
        geometry.pathValue = json.dumps(stream)
        rebuilds = geometry.fullRebuildCount

        stream["points"][2] = [40.0, 100.0, -25.0]
        geometry.pathValue = json.dumps(stream)

        self.assertEqual(geometry.fullRebuildCount, rebuilds + 1)

    def test_trail_ribbon_sharp_turn_uses_miter_limit_fallback(self) -> None:
        """近乎掉头的折角必须退化为 bevel，边缘不能产生远离中心线的尖刺。"""

        geometry = TrailRibbonGeometry()
        geometry.widthValue = 20.0
        geometry.pathValue = json.dumps(
            {
                "generation": 1,
                "firstSequence": 0,
                "endSequence": 3,
                "points": [[0.0, 100.0, 0.0], [100.0, 100.0, 0.0], [0.1, 100.0, -1.0]],
            }
        )

        self.assertGreaterEqual(geometry.bevelJoinCount, 1)
        vertex_data = bytes(geometry.vertexData())
        corner_vertices = []
        for vertex_index in (2, 3, 5, 6, 9):
            offset = vertex_index * geometry.stride()
            corner_vertices.append(struct.unpack_from("<fff", vertex_data, offset))
        for x_coord, _, z_coord in corner_vertices:
            self.assertLessEqual(((x_coord - 100.0) ** 2 + z_coord**2) ** 0.5, 20.0 + 1e-5)

    def test_trail_ribbon_bevel_fills_outer_side_for_both_turn_directions(self) -> None:
        """正反急转的 bevel 三角形都必须连接同一外侧边，不能误填内侧并留下裂口。"""

        def join_indices(final_z: float) -> tuple[int, int, int]:
            geometry = TrailRibbonGeometry()
            geometry.pathValue = json.dumps(
                {
                    "op": "reset",
                    "generation": 12,
                    "firstSequence": 0,
                    "endSequence": 3,
                    "points": [[0.0, 100.0, 0.0], [100.0, 100.0, 0.0], [0.1, 100.0, final_z]],
                }
            )
            # 第二段占槽1，每槽前6个索引是主体，后3个才是 bevel 填充面。
            return struct.unpack_from("<III", bytes(geometry.indexData()), 1 * 9 * 4 + 6 * 4)

        self.assertEqual(join_indices(1.0), (2, 5, 9))
        self.assertEqual(join_indices(-1.0), (3, 6, 9))

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
        self.assertNotIn("ListModel { id: riskZoneModel }", qml)
        self.assertNotIn("data.riskZones", qml)
        self.assertIn("data.riskZoneLines", qml)
        self.assertIn("data.riskZoneBuffers", qml)
        self.assertIn("terrainGeometry.riskAreasValue = JSON.stringify(data.terrainRiskAreas || [])", qml)
        self.assertIn("property real alertBoundaryPulse: 0.48", qml)
        self.assertIn("SequentialAnimation on alertBoundaryPulse", qml)
        # 告警呼吸周期收紧为 1 秒：500ms 变亮 + 500ms 变暗，缓动保持 InOutSine。
        self.assertEqual(qml.count("duration: 500"), 2)
        self.assertEqual(qml.count("duration: 2400"), 0)
        self.assertEqual(qml.count("easing.type: Easing.InOutSine"), 2)
        self.assertIn("pulseValue: item.pulse === true", qml)
        self.assertIn("opacity: model.pulseValue ? root.alertBoundaryPulse : 0.95", qml)
        # 危险区填充与边界共用同一呼吸源，线性映射到 0.10~0.35 的低透明度区间。
        self.assertIn("readonly property real riskFillPulse: 0.10 + (alertBoundaryPulse - 0.48) * (0.25 / 0.44)", qml)
        self.assertIn("ListModel { id: riskFillModel }", qml)
        self.assertIn("data.riskZoneFills", qml)
        self.assertIn("RiskFillGeometry", qml)
        self.assertIn("meshValue: model.meshValue", qml)
        self.assertIn("opacity: root.riskFillPulse", qml)
        # 填充是贴地提示薄层，绝不能参与阴影，否则会遮挡飞机与尾迹可读性。
        fill_block = qml[qml.index("model: riskFillModel") : qml.index("model: riskLineModel")]
        self.assertIn("castsShadows: false", fill_block)
        self.assertIn("receivesShadows: false", fill_block)
        self.assertIn("alphaMode: PrincipledMaterial.Blend", fill_block)
        self.assertNotIn("model: obstacleModel", qml)
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

    def test_terrain_material_is_matte_with_visible_micro_relief(self) -> None:
        """验证正式地形材质消除塑料高光，并保留足够强的近景岩面细节。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")
        terrain_block = qml[qml.index("id: terrainSurfaceModel") : qml.index("Repeater3D", qml.index("id: terrainSurfaceModel"))]
        self.assertIn("roughness: 0.99", terrain_block)
        self.assertNotIn("alertBoundaryPulse", terrain_block)
        self.assertIn("specularAmount: 0.0", terrain_block)
        self.assertIn("normalStrength: 0.92", terrain_block)
        self.assertIn("scaleU: 148", terrain_block)
        self.assertIn("scaleV: 148", terrain_block)
        self.assertIn("emissiveFactor: Qt.vector3d(0.002, 0.003, 0.005)", terrain_block)
        self.assertIn('source: "assets/terrain_detail_normal.png"', terrain_block)
        self.assertIn('source: "assets/terrain_detail_albedo.png"', terrain_block)
        self.assertIn("receivesShadows: true", terrain_block)
        self.assertIn("castsShadows: true", terrain_block)

        environment_block = qml[qml.index("environment: SceneEnvironment") : qml.index("Node {", qml.index("environment: SceneEnvironment"))]
        key_light_block = qml[qml.index("DirectionalLight {") : qml.index("DirectionalLight {", qml.index("DirectionalLight {") + 1)]
        self.assertIn("aoEnabled: true", environment_block)
        self.assertIn("aoStrength:", environment_block)
        self.assertIn("castsShadow: true", key_light_block)
        self.assertIn("shadowMapQuality: Light.ShadowMapQualityUltra", key_light_block)

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

    def test_aircraft_and_trail_tip_share_one_presentation_progress(self) -> None:
        """飞机与尾迹末端必须由同一展示进度计算，不能让飞机独自执行位置 Behavior。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("property real presentationProgress: 1.0", qml)
        self.assertIn("readonly property int presentationQueueCapacity: 2", qml)
        self.assertIn("id: presentationMotion", qml)
        self.assertIn("function presentationPosition(", qml)
        self.assertIn("function currentTrailTipPositions(", qml)
        self.assertIn("function enqueueSceneUpdate(", qml)
        self.assertIn("presentationMotion.restart()", qml)
        self.assertIn("position: root.presentationPosition(", qml)
        self.assertIn("geometry: TrailTipGeometry {", qml)
        self.assertIn("previousPosition: Qt.vector3d(", qml)
        self.assertIn("startPosition: Qt.vector3d(", qml)
        self.assertIn("endPosition: root.presentationPosition(", qml)
        self.assertNotIn("tipPosition: root.presentationPosition(", qml)
        self.assertNotIn("Behavior on position", qml)

    def test_trail_width_is_one_fifth_of_near_view_aircraft_span(self) -> None:
        """近景尾迹宽度应约为飞机翼展 1/5，远景不应继续变粗。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("property real trailWidthScale: Math.min(0.17, aircraftVisualScale * aircraftUnitWingspan / 5.0 / 44.0)", qml)
        self.assertIn("飞机视觉缩放单独保持远景可辨识；尾迹近景按翼展 1/5 显示，远景不继续加粗。", qml)
        self.assertNotIn("property real trailWidthScale: nearViewWidthScale", qml)
        self.assertNotIn("property real trailWidthScale: aircraftVisualScale / (1800.0 / 85.0)", qml)


if __name__ == "__main__":
    unittest.main()
