"""3D 态势场景数据适配回归测试。"""

from __future__ import annotations

from pathlib import Path
import json
import struct
import unittest

from src.ui.gui.situation3d.scene_data import DEFAULT_TERRAIN_SPAN_M, build_scene_payload, enu_to_quick3d
from src.ui.gui.situation3d.terrain_geometry import TerrainGeometry
from src.ui.gui.situation3d.trail_ribbon_geometry import TrailRibbonGeometry
from src.ui.gui.view_models import (
    LinkState,
    NodeState,
    ObstacleView,
    ReferenceRoute,
    Snapshot,
    TrailPoint,
)

QML_VIEW_PATH = Path(__file__).resolve().parents[2] / "src" / "ui" / "gui" / "situation3d" / "qml" / "Situation3DView.qml"


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
        self.assertLess(aircraft[0]["yawDeg"], 0.0)

        self.assertEqual(payload["counts"]["aircraft"], 2)
        self.assertEqual(payload["counts"]["trailRibbons"], 1)
        self.assertGreaterEqual(payload["counts"]["routePoints"], 2)
        self.assertEqual(payload["counts"]["obstacles"], 1)
        self.assertNotIn("trailPoints", payload)
        self.assertNotIn("trailSegments", payload)
        trail_ribbon = payload["trailRibbons"][0]
        self.assertEqual(trail_ribbon["nodeId"], "A01")
        self.assertEqual(trail_ribbon["width"], 44.0)
        self.assertEqual(json.loads(trail_ribbon["pathValue"]), [[1.0, 3.0, -2.0], [4.0, 6.0, -5.0]])
        self.assertEqual(payload["terrain"]["ground"]["width"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["ground"]["depth"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["surface"]["width"], DEFAULT_TERRAIN_SPAN_M)
        self.assertEqual(payload["terrain"]["surface"]["depth"], DEFAULT_TERRAIN_SPAN_M)
        self.assertGreater(payload["terrain"]["surface"]["height"], 0.0)

        obstacle_payload = payload["obstacles"][0]
        self.assertEqual(obstacle_payload["kind"], "circle")
        self.assertEqual(obstacle_payload["radius"], 30.0)
        self.assertEqual(obstacle_payload["z"], -70.0)

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

    def test_disabled_obstacle_is_not_exported_to_scene(self) -> None:
        obstacle = ObstacleView("OFF", "circle", enabled=False, center_x=1.0, center_y=2.0, radius=3.0)

        payload = build_scene_payload(self._snapshot(), [obstacle], clearance_m=10.0)

        self.assertEqual(payload["counts"]["obstacles"], 0)
        self.assertEqual(payload["obstacles"], [])

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
        self.assertIn("TrailRibbonGeometry", qml)
        self.assertIn("pathValue: model.pathValue", qml)
        self.assertIn("function syncTrailModel", qml)
        self.assertNotIn("trailModel.clear()", qml)
        self.assertNotIn("data.trailPoints", qml)

    def test_aircraft_marker_stays_small_but_distance_visible(self) -> None:
        """飞机点应保持小尺寸，并随相机距离略微放大以免缩远后消失。"""

        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("property real aircraftPointScale", qml)
        self.assertIn("Math.max(0.10, Math.min(0.55, distance / 36000.0))", qml)
        self.assertIn(
            "scale: Qt.vector3d(root.aircraftPointScale, root.aircraftPointScale, root.aircraftPointScale)",
            qml,
        )
        self.assertNotIn("scale: Qt.vector3d(2.0, 2.0, 2.0)", qml)


if __name__ == "__main__":
    unittest.main()
