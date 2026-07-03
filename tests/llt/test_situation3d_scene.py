"""3D 态势场景数据适配回归测试。"""

from __future__ import annotations

from pathlib import Path
import unittest

from src.ui.gui.situation3d.scene_data import build_scene_payload, enu_to_quick3d
from src.ui.gui.situation3d.terrain_geometry import TerrainGeometry, TerrainGridGeometry
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
        self.assertGreaterEqual(payload["counts"]["trailPoints"], 2)
        self.assertGreaterEqual(payload["counts"]["routePoints"], 2)
        self.assertEqual(payload["counts"]["obstacles"], 1)
        self.assertGreater(payload["terrain"]["ground"]["width"], 0.0)
        self.assertGreater(payload["terrain"]["surface"]["width"], 0.0)
        self.assertGreater(payload["terrain"]["surface"]["depth"], 0.0)
        self.assertGreater(payload["terrain"]["surface"]["height"], 0.0)

        obstacle_payload = payload["obstacles"][0]
        self.assertEqual(obstacle_payload["kind"], "circle")
        self.assertEqual(obstacle_payload["radius"], 30.0)
        self.assertEqual(obstacle_payload["z"], -70.0)

    def test_terrain_geometry_builds_connected_heightfield(self) -> None:
        """验证 3D 地形使用一张连续 mesh，而不是多个独立山体模型。"""

        geometry = TerrainGeometry()
        geometry.widthValue = 1200.0
        geometry.depthValue = 900.0
        geometry.amplitudeValue = 180.0

        self.assertEqual(geometry.stride(), 32)
        self.assertGreater(geometry.vertexData().size(), 0)
        self.assertGreater(geometry.indexData().size(), 0)
        self.assertLessEqual(geometry.boundsMin().y(), 0.0)
        self.assertGreater(geometry.boundsMax().y(), 180.0)

    def test_terrain_grid_geometry_follows_heightfield(self) -> None:
        """验证地形线框也由连续高度场生成。"""

        geometry = TerrainGridGeometry()
        geometry.widthValue = 1200.0
        geometry.depthValue = 900.0
        geometry.amplitudeValue = 180.0

        self.assertEqual(geometry.stride(), 12)
        self.assertGreater(geometry.vertexData().size(), 0)
        self.assertGreater(geometry.boundsMax().y(), 180.0)

    def test_disabled_obstacle_is_not_exported_to_scene(self) -> None:
        obstacle = ObstacleView("OFF", "circle", enabled=False, center_x=1.0, center_y=2.0, radius=3.0)

        payload = build_scene_payload(self._snapshot(), [obstacle], clearance_m=10.0)

        self.assertEqual(payload["counts"]["obstacles"], 0)
        self.assertEqual(payload["obstacles"], [])

    def test_qml_accepts_middle_button_for_focus_pan(self) -> None:
        qml = QML_VIEW_PATH.read_text(encoding="utf-8")

        self.assertIn("Qt.MiddleButton", qml)
        self.assertIn("mouse.buttons & Qt.MiddleButton", qml)
        self.assertIn("TerrainGeometry", qml)
        self.assertIn("TerrainGridGeometry", qml)
        self.assertNotIn("hillModel", qml)


if __name__ == "__main__":
    unittest.main()
