"""钻石项目 XML 航线文件策略。注意：适配客户固定输入输出格式。"""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET

from src.data.linefile.strategy import LineFileStrategy


DEFAULT_SPEED_MPS = 45.0
OUTPUT_SKYWAY_NO = "25"
OUTPUT_SKYPOINT_NO = "1"
OUTPUT_LINE_NAME = "芜湖自动避障航线"
OUTPUT_LINE_TYPE = "0"
DEFAULT_WAYPOINT_TASK = "0010"

_LAT_KEYS = ("latitude_deg", "lat_deg", "lat", "Latitude")
_LON_KEYS = ("longitude_deg", "lon_deg", "lon", "Longitude")


class DiamondXmlLineFileStrategy(LineFileStrategy):
    """钻石项目航线 XML 策略。注意：输入不校验航线号，输出强制避障航线规范。"""

    def __init__(self, now: Callable[[], datetime] | None = None) -> None:
        """初始化策略。注意：now 仅供测试固定输出时间戳。"""
        self._now = now or datetime.now

    def supports(self, path: Path) -> bool:
        """按 .xml 后缀识别钻石项目航线文件。注意：大小写不敏感。"""
        return path.suffix.lower() == ".xml"

    def load(self, path: Path) -> dict[str, object]:
        """读取钻石 XML 航线并返回 route 对象。注意：XML 本身无速度，统一补 45 m/s。"""
        root = self._parse_root(path)
        header = _required_child(root, "Item", "Root.Item")
        for field in ("SkywayNo", "SkypointNo", "StLineExp"):
            # SkywayNo 和 SkypointNo 只要求标签存在，不约束具体内容。
            _required_child(header, field, f"Item.{field}")
        for field in ("ByLineName", "CreatTimer"):
            _required_text(header, field, f"Item.{field}")
        count = _read_positive_int(_required_text(header, "IdAllNum", "Item.IdAllNum"), "Item.IdAllNum")

        waypoints: list[dict[str, object]] = []
        for index in range(1, count + 1):
            item = _required_child(root, f"Item{index}", f"Root.Item{index}")
            if "id" not in item.attrib:
                raise ValueError(f"diamond xml route_file missing Root.Item{index}.id")
            waypoint = {
                "longitude_deg": _read_float_text(item, "Longitude", f"Item{index}.Longitude"),
                "latitude_deg": _read_float_text(item, "Latitude", f"Item{index}.Latitude"),
                "altitude_m": _read_float_text(item, "Altitude", f"Item{index}.Altitude"),
                "waypoint_task": _required_text(item, "WaypointTask", f"Item{index}.WaypointTask"),
            }
            waypoints.append(waypoint)
        return {"speed_mps": DEFAULT_SPEED_MPS, "waypoints": waypoints}

    def save(self, path: Path, route: dict[str, object]) -> Path:
        """输出钻石 XML 避障航线并返回规范化文件名。注意：圆弧字段会被拒绝。"""
        timestamp = self._now()
        output_path = _diamond_output_path(path, timestamp)
        waypoints = _validated_output_waypoints(route)
        root = ET.Element("Root")
        header = ET.SubElement(root, "Item")
        _add_text(header, "SkywayNo", OUTPUT_SKYWAY_NO)
        _add_text(header, "SkypointNo", OUTPUT_SKYPOINT_NO)
        _add_text(header, "IdAllNum", str(len(waypoints)))
        _add_text(header, "ByLineName", OUTPUT_LINE_NAME)
        _add_text(header, "StLine_Type", OUTPUT_LINE_TYPE)
        ET.SubElement(header, "StLineExp")
        _add_text(header, "CreatTimer", _format_xml_timestamp(timestamp))

        for index, waypoint in enumerate(waypoints, start=1):
            item = ET.SubElement(root, f"Item{index}", {"id": str(index)})
            _add_text(item, "Longitude", _format_number(waypoint["longitude_deg"]))
            _add_text(item, "Latitude", _format_number(waypoint["latitude_deg"]))
            _add_text(item, "Altitude", _format_number(waypoint["altitude_m"]))
            _add_text(item, "WaypointTask", str(waypoint["waypoint_task"]))

        output_path.parent.mkdir(parents=True, exist_ok=True)
        _write_xml(output_path, root)
        return output_path

    def default_output_filename(self) -> str:
        """返回钻石 XML 避障航线默认输出名。注意：文件名带当前时间戳。"""
        return _diamond_output_filename(self._now())

    @staticmethod
    def _parse_root(path: Path) -> ET.Element:
        """解析 XML 根节点。注意：解析失败统一映射为配置错误。"""
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            raise ValueError(f"diamond xml route_file is not valid XML: {exc}") from exc
        if root.tag != "Root":
            raise ValueError("diamond xml route_file root must be Root")
        return root


def _required_child(parent: ET.Element, tag: str, field_name: str) -> ET.Element:
    """读取必填子节点。注意：只查直接子节点，避免误吞嵌套错误。"""
    child = parent.find(tag)
    if child is None:
        raise ValueError(f"diamond xml route_file missing {field_name}")
    return child


def _required_text(parent: ET.Element, tag: str, field_name: str) -> str:
    """读取必填文本。注意：空白文本按缺失处理。"""
    text = _required_child(parent, tag, field_name).text
    if text is None or not text.strip():
        raise ValueError(f"diamond xml route_file missing {field_name}")
    return text.strip()


def _read_positive_int(text: str, field_name: str) -> int:
    """读取正整数。注意：IdAllNum 决定后续 Item 节点数量。"""
    try:
        value = int(text)
    except ValueError as exc:
        raise ValueError(f"diamond xml route_file {field_name} must be an integer") from exc
    if value <= 0:
        raise ValueError(f"diamond xml route_file {field_name} must be positive")
    return value


def _read_float_text(parent: ET.Element, tag: str, field_name: str) -> float:
    """读取浮点文本。注意：经纬高字段必须可转换为数字。"""
    text = _required_text(parent, tag, field_name)
    try:
        return float(text)
    except ValueError as exc:
        raise ValueError(f"diamond xml route_file {field_name} must be numeric") from exc


def _validated_output_waypoints(route: dict[str, object]) -> list[dict[str, object]]:
    """校验并提取可输出航点。注意：钻石 XML 只支持纯折线经纬高。"""
    raw_waypoints = route.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("diamond xml route waypoints must be a non-empty list")
    waypoints: list[dict[str, object]] = []
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            raise ValueError(f"diamond xml route.waypoints[{index}] must be an object")
        _reject_arc_fields(raw, index)
        waypoints.append(
            {
                "longitude_deg": _read_point_float(raw, _LON_KEYS, index, "longitude_deg"),
                "latitude_deg": _read_point_float(raw, _LAT_KEYS, index, "latitude_deg"),
                "altitude_m": float(raw.get("altitude_m", raw.get("h", 0.0))),
                "waypoint_task": str(raw.get("waypoint_task", DEFAULT_WAYPOINT_TASK)),
            }
        )
    return waypoints


def _reject_arc_fields(raw: dict[str, object], index: int) -> None:
    """拒绝圆弧字段。注意：避免把客户 XML 不支持的曲率静默丢掉。"""
    turn_sign = float(raw.get("turn_sign", raw.get("turnSign", 0.0)))
    if turn_sign != 0.0 or "center" in raw or "center_x_m" in raw or "center_y_m" in raw:
        raise ValueError(f"钻石 XML 航线不支持圆弧航段：route.waypoints[{index}]")


def _read_point_float(raw: dict[str, object], keys: tuple[str, ...], index: int, field_name: str) -> float:
    """从航点读取浮点字段。注意：兼容内部正式字段和客户 XML 原字段名。"""
    for key in keys:
        if key in raw:
            try:
                return float(raw[key])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"diamond xml route.waypoints[{index}].{field_name} must be numeric") from exc
    raise ValueError(f"diamond xml route.waypoints[{index}] missing {field_name}")


def _diamond_output_path(path: Path, timestamp: datetime) -> Path:
    """生成钻石规范输出文件名。注意：忽略用户输入文件名，只保留目录。"""
    return path.parent / _diamond_output_filename(timestamp)


def _diamond_output_filename(timestamp: datetime) -> str:
    """生成钻石规范输出文件名。注意：供保存和 GUI 默认名称共用。"""
    return f"航线{OUTPUT_SKYWAY_NO} {OUTPUT_LINE_NAME} {_format_filename_timestamp(timestamp)}.XML"


def _format_filename_timestamp(timestamp: datetime) -> str:
    """格式化文件名时间戳。注意：与客户样例保持中文年月日时分秒形式。"""
    return (
        f"{timestamp.year}年{timestamp.month}月{timestamp.day}日"
        f"{timestamp.hour}时{timestamp.minute}分{timestamp.second}秒"
    )


def _format_xml_timestamp(timestamp: datetime) -> str:
    """格式化 XML 内时间戳。注意：日期不补零，分秒补齐两位。"""
    return f"{timestamp.year}/{timestamp.month}/{timestamp.day} {timestamp.hour}:{timestamp.minute:02d}:{timestamp.second:02d}"


def _format_number(value: object) -> str:
    """格式化 XML 数字文本。注意：去掉无意义尾零，保持样例风格。"""
    number = float(value)
    if abs(number - round(number)) <= 1e-9:
        return str(int(round(number)))
    return f"{number:.7f}".rstrip("0").rstrip(".")


def _add_text(parent: ET.Element, tag: str, text: str) -> None:
    """追加文本节点。注意：集中处理便于保持字段顺序。"""
    child = ET.SubElement(parent, tag)
    child.text = text


def _write_xml(path: Path, root: ET.Element) -> None:
    """写入 XML 文本。注意：手动声明头以匹配客户样例的双引号形式。"""
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    path.write_text(f'<?xml version="1.0" encoding="utf-8"?>\n{body}\n', encoding="utf-8")
