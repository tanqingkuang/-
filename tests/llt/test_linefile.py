"""航线文件策略和管理器回归测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

from src.data.linefile import LineFileManager, LineFileStrategyFactory
from src.data.linefile.diamond_xml_strategy import DiamondXmlLineFileStrategy
from src.data.config_loader import resolve_config_references
from src.runner.gui_application import persist_config_route_file


def _write_diamond_template(path: Path) -> None:
    path.write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<Root>
  <Item source="基础航线">
    <SkywayNo>1</SkywayNo>
    <SkypointNo>7</SkypointNo>
    <IdAllNum>99</IdAllNum>
    <ByLineName>钻石默认航线</ByLineName>
    <StLine_Type>9</StLine_Type>
    <StLineExp>继承说明</StLineExp>
    <ExtraField>保留内容</ExtraField>
    <CreatTimer>2024/1/2 3:04:05</CreatTimer>
  </Item>
</Root>
""",
        encoding="utf-8",
    )


class LineFileTests(unittest.TestCase):
    """覆盖 route_file 的解析、生成和策略工厂选择。"""

    def test_json_strategy_loads_route_object_by_relative_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "line.json").write_text(
                json.dumps({"speed_mps": 20.0, "waypoints": [{"x_m": 1.0, "y_m": 2.0, "altitude_m": 3.0}]}),
                encoding="utf-8",
            )

            route = LineFileManager().load_route(root / "base.json", "element/line.json")

        self.assertEqual(route["speed_mps"], 20.0)
        self.assertEqual(route["waypoints"], [{"x_m": 1.0, "y_m": 2.0, "altitude_m": 3.0}])

    def test_json_strategy_saves_route_object(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manager = LineFileManager()
            route = {"speed_mps": 18.0, "waypoints": [{"x_m": 10.0, "y_m": 0.0, "altitude_m": 1000.0}]}

            written = manager.save_route(root / "base.json", "element/line.json", route)

            payload = json.loads(written.read_text(encoding="utf-8"))
        self.assertEqual(payload, route)

    def test_diamond_xml_strategy_loads_route_and_defaults_speed(self) -> None:
        """钻石 XML 输入不校验航线号内容，但必须展开为经纬高 route。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route_file = root / "航线1 任意名称.XML"
            route_file.write_text(
                """<?xml version="1.0" encoding="utf-8"?>
<Root>
  <Item>
    <SkywayNo>999</SkywayNo>
    <SkypointNo>ABC</SkypointNo>
    <IdAllNum>2</IdAllNum>
    <ByLineName>任意输入航线</ByLineName>
    <StLineExp />
    <CreatTimer>2025/9/11 20:32:49</CreatTimer>
  </Item>
  <Item1 id="1">
    <Longitude>118.722553</Longitude>
    <Latitude>31.0991</Latitude>
    <Altitude>2400</Altitude>
    <WaypointTask>0010</WaypointTask>
  </Item1>
  <Item2 id="2">
    <Longitude>118.658953</Longitude>
    <Latitude>31.049993</Latitude>
    <Altitude>2410.5</Altitude>
    <WaypointTask>0020</WaypointTask>
  </Item2>
</Root>
""",
                encoding="utf-8",
            )

            route = LineFileManager().load_route(root / "base.json", str(route_file))

        self.assertEqual(route["speed_mps"], 45.0)
        self.assertEqual(len(route["waypoints"]), 2)
        first = route["waypoints"][0]
        self.assertAlmostEqual(first["longitude_deg"], 118.722553)
        self.assertAlmostEqual(first["latitude_deg"], 31.0991)
        self.assertAlmostEqual(first["altitude_m"], 2400.0)

    def test_diamond_xml_strategy_rejects_incomplete_format(self) -> None:
        """钻石 XML 的 IdAllNum 必须能对应连续完整航点节点。"""
        with tempfile.TemporaryDirectory() as tmp:
            route_file = Path(tmp) / "bad.XML"
            route_file.write_text(
                """<?xml version="1.0" encoding="utf-8"?>
<Root>
  <Item>
    <SkywayNo>1</SkywayNo>
    <SkypointNo>1</SkypointNo>
    <IdAllNum>2</IdAllNum>
    <ByLineName>坏航线</ByLineName>
    <StLineExp />
    <CreatTimer>2025/9/11 20:32:49</CreatTimer>
  </Item>
  <Item1 id="1">
    <Longitude>118.722553</Longitude>
    <Latitude>31.0991</Latitude>
    <Altitude>2400</Altitude>
    <WaypointTask>0010</WaypointTask>
  </Item1>
</Root>
""",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "Root.Item2"):
                LineFileManager().load_route(route_file, str(route_file))

    def test_diamond_xml_strategy_saves_canonical_avoidance_file(self) -> None:
        """钻石 XML 输出继承基础头，仅更新名称、时间、航点数和文件名。"""
        fixed_now = datetime(2025, 9, 11, 20, 32, 49)
        manager = LineFileManager(LineFileStrategyFactory([DiamondXmlLineFileStrategy(lambda: fixed_now)]))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            template_path = root / "航线1 钻石默认航线 2026年7月6日10时8分18秒.XML"
            _write_diamond_template(template_path)
            route = {
                "speed_mps": 45.0,
                "waypoints": [
                    {"longitude_deg": 118.722553, "latitude_deg": 31.0991, "altitude_m": 2400.0},
                    {"longitude_deg": 118.658953, "latitude_deg": 31.049993, "altitude_m": 2400.0},
                ],
            }

            written = manager.save_route(
                root / "base.json",
                "客户随便取名.xml",
                route,
                template_route_file=str(template_path),
            )

            self.assertEqual(written.name, "航线1 威胁避让航线 2025年9月11日20时32分49秒.XML")
            xml_root = ET.parse(written).getroot()
        header = xml_root.find("Item")
        self.assertEqual(header.attrib, {"source": "基础航线"})
        self.assertEqual(
            [child.tag for child in header],
            [
                "SkywayNo",
                "SkypointNo",
                "IdAllNum",
                "ByLineName",
                "StLine_Type",
                "StLineExp",
                "ExtraField",
                "CreatTimer",
            ],
        )
        self.assertEqual(header.findtext("SkywayNo"), "1")
        self.assertEqual(header.findtext("SkypointNo"), "7")
        self.assertEqual(header.findtext("IdAllNum"), "2")
        self.assertEqual(header.findtext("ByLineName"), "威胁避让航线")
        self.assertEqual(header.findtext("StLine_Type"), "9")
        self.assertEqual(header.findtext("StLineExp"), "继承说明")
        self.assertEqual(header.findtext("ExtraField"), "保留内容")
        self.assertEqual(header.findtext("CreatTimer"), "2025/9/11 20:32:49")
        self.assertEqual(xml_root.find("Item1").attrib["id"], "1")
        self.assertEqual(xml_root.findtext("Item1/Longitude"), "118.722553")
        self.assertEqual(xml_root.findtext("Item1/Latitude"), "31.0991")
        self.assertEqual(xml_root.findtext("Item1/Altitude"), "2400")
        self.assertEqual(xml_root.findtext("Item1/WaypointTask"), "0010")

    def test_diamond_xml_strategy_provides_default_output_filename(self) -> None:
        """钻石 XML 策略应向 GUI 提供规范默认文件名，避免界面硬编码 JSON 名称。"""
        fixed_now = datetime(2025, 9, 11, 20, 32, 49)
        manager = LineFileManager(LineFileStrategyFactory([DiamondXmlLineFileStrategy(lambda: fixed_now)]))
        with tempfile.TemporaryDirectory() as tmp:
            template_path = Path(tmp) / "航线1 钻石默认航线 2026年7月6日10时8分18秒.XML"
            _write_diamond_template(template_path)

            filename = manager.default_output_filename(template_path)

        self.assertEqual(filename, "航线1 威胁避让航线 2025年9月11日20时32分49秒.XML")

    def test_diamond_xml_strategy_rejects_mismatched_filename(self) -> None:
        """文件名中的航线号和航线名必须与 XML 一致，避免误识别日期数字。"""
        manager = LineFileManager(LineFileStrategyFactory([DiamondXmlLineFileStrategy()]))
        with tempfile.TemporaryDirectory() as tmp:
            template_path = Path(tmp) / "航线99 钻石默认航线 2026年7月6日10时8分18秒.XML"
            _write_diamond_template(template_path)

            with self.assertRaisesRegex(ValueError, "filename must match"):
                manager.default_output_filename(template_path)

    def test_diamond_xml_strategy_rejects_arc_output(self) -> None:
        """钻石 XML 不支持圆弧航段，保存时必须报错而不是静默丢字段。"""
        with tempfile.TemporaryDirectory() as tmp:
            route = {
                "waypoints": [
                    {"longitude_deg": 118.7, "latitude_deg": 31.0, "altitude_m": 2400.0},
                    {
                        "longitude_deg": 118.8,
                        "latitude_deg": 31.1,
                        "altitude_m": 2400.0,
                        "turn_sign": 1.0,
                        "center": {"longitude_deg": 118.75, "latitude_deg": 31.05, "altitude_m": 2400.0},
                    },
                ],
            }

            with self.assertRaisesRegex(ValueError, "不支持圆弧航段"):
                LineFileManager().save_route(Path(tmp) / "base.json", "out.XML", route)

    def test_config_loader_resolves_diamond_xml_route_file(self) -> None:
        """主配置 route_file 指向 .XML 时应由工厂创建钻石策略并转为内部 ENU。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            (element / "line.XML").write_text(
                """<?xml version="1.0" encoding="utf-8"?>
<Root>
  <Item>
    <SkywayNo>1</SkywayNo>
    <SkypointNo>1</SkypointNo>
    <IdAllNum>2</IdAllNum>
    <ByLineName>输入航线</ByLineName>
    <StLineExp />
    <CreatTimer>2025/9/11 20:32:49</CreatTimer>
  </Item>
  <Item1 id="1"><Longitude>118.0</Longitude><Latitude>31.0</Latitude><Altitude>2400</Altitude><WaypointTask>0010</WaypointTask></Item1>
  <Item2 id="2"><Longitude>118.01</Longitude><Latitude>31.01</Latitude><Altitude>2410</Altitude><WaypointTask>0010</WaypointTask></Item2>
</Root>
""",
                encoding="utf-8",
            )

            resolved = resolve_config_references({"route_file": "element/line.XML"}, root / "base.json")

        route = resolved["route"]
        self.assertEqual(route["speed_mps"], 45.0)
        self.assertIn("_geo_origin", route)
        self.assertAlmostEqual(route["waypoints"][0]["x_m"], 0.0)
        self.assertAlmostEqual(route["waypoints"][0]["y_m"], 0.0)

    def test_persist_config_route_file_updates_relative_reference_after_validation(self) -> None:
        """选择有效航线后应把相对引用写回主配置。"""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config_path = root / "configs" / "base.json"
            route_path = root / "routes" / "selected.json"
            config_path.parent.mkdir()
            route_path.parent.mkdir()
            config_path.write_text(json.dumps({"duration_s": 1.0}), encoding="utf-8")
            route_path.write_text(
                json.dumps(
                    {
                        "speed_mps": 20.0,
                        "waypoints": [
                            {"longitude_deg": 118.0, "latitude_deg": 31.0, "altitude_m": 1000.0},
                            {"longitude_deg": 118.01, "latitude_deg": 31.01, "altitude_m": 1000.0},
                        ],
                    }
                ),
                encoding="utf-8",
            )

            reference = persist_config_route_file(config_path, route_path)
            config = json.loads(config_path.read_text(encoding="utf-8"))

        self.assertEqual(reference, "../routes/selected.json")
        self.assertEqual(config["route_file"], reference)

    def test_persist_config_route_file_does_not_change_config_for_invalid_route(self) -> None:
        """航线解析失败时必须保留原配置。"""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config_path = root / "base.json"
            route_path = root / "bad.XML"
            original = '{"duration_s": 1.0}\n'
            config_path.write_text(original, encoding="utf-8")
            route_path.write_text("<bad>", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "not valid XML"):
                persist_config_route_file(config_path, route_path)

            self.assertEqual(config_path.read_text(encoding="utf-8"), original)

    def test_factory_rejects_unsupported_route_file_format(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported route_file format"):
            LineFileStrategyFactory().create("line.csv")

    def test_route_file_must_be_non_empty_string(self) -> None:
        with self.assertRaisesRegex(ValueError, "route_file must be a non-empty string"):
            LineFileManager().load_route("base.json", "")

    def test_config_loader_rejects_removed_rally_route_fields(self) -> None:
        """旧集结航线字段必须显式报错，避免配置被静默忽略。"""

        for field in ("rally_route_file", "rally_route"):
            with self.subTest(field=field):
                with self.assertRaisesRegex(ValueError, rf"{field}.*route_file"):
                    resolve_config_references({field: "removed"}, "scenario.json")

    def test_config_loader_resolves_formation_files(self) -> None:
        """formation_files 应按主配置位置展开成内部 formations 列表。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element" / "formations"
            element.mkdir(parents=True)
            (element / "triangle.json").write_text(
                json.dumps({
                    "name": "TRIANGLE",
                    "slots": [
                        {"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                        {"node_id": "A02", "x_m": -54.0, "y_m": 0.0, "z_m": -58.0},
                    ],
                }),
                encoding="utf-8",
            )

            resolved = resolve_config_references(
                {
                    "formation": {
                        "coordinate_system": "x_forward_y_up_z_right",
                        "formation_files": ["element/formations/triangle.json"],
                    }
                },
                root / "base.json",
            )

        formations = resolved["formation"]["formations"]  # type: ignore[index]
        self.assertEqual(formations[0]["name"], "TRIANGLE")
        self.assertEqual(formations[0]["slots"][1]["node_id"], "A02")

    def test_config_loader_rejects_inline_formation_config(self) -> None:
        """文件入口不再接受旧的 pattern/slots 内联队形写法。"""
        with self.assertRaisesRegex(ValueError, "formation_files"):
            resolve_config_references(
                {
                    "formation": {
                        "pattern": "TRIANGLE",
                        "coordinate_system": "x_forward_y_up_z_right",
                        "slots": [{"node_id": "A01", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0}],
                    }
                },
                "base.json",
            )


if __name__ == "__main__":
    unittest.main()
