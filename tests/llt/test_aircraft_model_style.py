"""3D 态势机型样式策略回归测试。"""

from __future__ import annotations

from typing import cast
import unittest

from src.ui.gui.situation3d.aircraft_model_style import (
    DEFAULT_AIRCRAFT_MODEL_TYPE,
    AircraftModelType,
    BayraktarTB2Style,
    PredatorStyle,
    RQ4GlobalHawkStyle,
    available_model_options,
    create_aircraft_model_style,
)
from src.ui.gui.situation3d.scene_data import build_scene_payload
from src.ui.gui.view_models import NodeState, Snapshot


class AircraftModelStyleTests(unittest.TestCase):
    """验证 3D 态势机型策略与场景 payload 契约。"""

    def _snapshot(self) -> Snapshot:
        """构造最小 3D 态势快照。注意：只用于验证 payload 字段。"""

        return Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="PAUSED",
            control_report="暂停",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 0.0, 0.0, 10.0, 0.0, altitude=100.0)],
            links=[],
        )

    def test_factory_returns_registered_style_by_enum(self) -> None:
        """工厂应按枚举返回对应策略。注意：后续机型只需注册新策略类。"""

        expected_styles = {
            AircraftModelType.BAYRAKTAR_TB2: BayraktarTB2Style,
            AircraftModelType.PREDATOR: PredatorStyle,
            AircraftModelType.RQ4_GLOBAL_HAWK: RQ4GlobalHawkStyle,
        }

        for model_type, style_class in expected_styles.items():
            with self.subTest(model_type=model_type):
                style = create_aircraft_model_style(model_type)

                self.assertIsInstance(style, style_class)
                self.assertEqual(style.model_type, model_type)

    def test_style_payload_contains_measured_values(self) -> None:
        """机型 payload 应包含 QML 渲染所需字段。注意：baseScale 来自真实翼展和模型翼展。"""

        cases = [
            (
                AircraftModelType.BAYRAKTAR_TB2,
                {
                    "value": "tb2",
                    "label": "TB2 察打无人机",
                    "modelSource": "assets/BayraktarTB2.glb",
                    "yawOffsetDeg": 90.0,
                    "unitWingspan": 11.957,
                    "realWingspanM": 12.0,
                    "baseScale": 12.0 / 11.957,
                },
            ),
            (
                AircraftModelType.PREDATOR,
                {
                    "value": "predator",
                    "label": "捕食者无人机",
                    "modelSource": "assets/PredatorUAV.glb",
                    "yawOffsetDeg": 90.0,
                    "unitWingspan": 1.76,
                    "realWingspanM": 15.0,
                    "baseScale": 15.0 / 1.76,
                },
            ),
            (
                AircraftModelType.RQ4_GLOBAL_HAWK,
                {
                    "value": "rq4",
                    "label": "RQ-4 全球鹰",
                    "modelSource": "assets/RQ4GlobalHawk.glb",
                    "yawOffsetDeg": 90.0,
                    "unitWingspan": 0.469,
                    "realWingspanM": 39.9,
                    "baseScale": 39.9 / 0.469,
                },
            ),
        ]

        for model_type, expected in cases:
            with self.subTest(model_type=model_type):
                payload = create_aircraft_model_style(model_type).style_payload()

                self.assertEqual(payload["value"], expected["value"])
                self.assertEqual(payload["label"], expected["label"])
                self.assertEqual(payload["modelSource"], expected["modelSource"])
                self.assertEqual(payload["yawOffsetDeg"], expected["yawOffsetDeg"])
                self.assertEqual(payload["unitWingspan"], expected["unitWingspan"])
                self.assertEqual(payload["realWingspanM"], expected["realWingspanM"])
                self.assertAlmostEqual(float(payload["baseScale"]), expected["baseScale"], places=9)
                self.assertEqual(
                    set(payload),
                    {
                        "value",
                        "label",
                        "modelSource",
                        "yawOffsetDeg",
                        "baseScale",
                        "unitWingspan",
                        "realWingspanM",
                    },
                )

    def test_factory_rejects_unregistered_type(self) -> None:
        """未注册机型应抛出 ValueError。注意：避免 QML 静默加载错误资产。"""

        with self.assertRaises(ValueError):
            create_aircraft_model_style(cast(AircraftModelType, "missing"))

    def test_build_scene_payload_defaults_to_tb2_style_and_options(self) -> None:
        """场景 payload 默认带 TB2 样式和机型列表。注意：仿真单机数据仍独立输出。"""

        payload = build_scene_payload(self._snapshot())

        self.assertEqual(DEFAULT_AIRCRAFT_MODEL_TYPE, AircraftModelType.BAYRAKTAR_TB2)
        self.assertEqual(payload["aircraftStyle"]["value"], "tb2")
        self.assertEqual(payload["aircraftStyle"]["modelSource"], "assets/BayraktarTB2.glb")
        self.assertEqual(
            payload["modelOptions"],
            [
                {"value": "tb2", "label": "TB2 察打无人机"},
                {"value": "predator", "label": "捕食者无人机"},
                {"value": "rq4", "label": "RQ-4 全球鹰"},
            ],
        )
        self.assertEqual(payload["aircraft"][0]["nodeId"], "A01")
        self.assertIn("yawDeg", payload["aircraft"][0])

    def test_build_scene_payload_can_use_predator_style(self) -> None:
        """场景 payload 应能输出捕食者样式。注意：调用方只传枚举，不直接依赖策略类。"""

        payload = build_scene_payload(self._snapshot(), model_type=AircraftModelType.PREDATOR)

        self.assertEqual(payload["aircraftStyle"]["value"], "predator")
        self.assertEqual(payload["aircraftStyle"]["label"], "捕食者无人机")
        self.assertEqual(payload["aircraftStyle"]["modelSource"], "assets/PredatorUAV.glb")
        self.assertEqual(payload["aircraftStyle"]["yawOffsetDeg"], 90.0)
        self.assertAlmostEqual(float(payload["aircraftStyle"]["baseScale"]), 15.0 / 1.76, places=9)

    def test_available_model_options_match_registered_enum(self) -> None:
        """下拉选项应与已注册枚举一致。注意：顺序由注册表声明顺序决定。"""

        options = available_model_options()

        self.assertEqual(
            [item["value"] for item in options],
            [
                AircraftModelType.BAYRAKTAR_TB2.value,
                AircraftModelType.PREDATOR.value,
                AircraftModelType.RQ4_GLOBAL_HAWK.value,
            ],
        )
        self.assertEqual(
            [item["label"] for item in options],
            [BayraktarTB2Style.label, PredatorStyle.label, RQ4GlobalHawkStyle.label],
        )
        self.assertEqual({item["value"] for item in options}, {model_type.value for model_type in AircraftModelType})


if __name__ == "__main__":
    unittest.main()
