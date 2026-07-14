"""3D 态势显示数据适配。注意：这里只做 GUI 坐标映射，不改变仿真坐标语义。"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import os
from functools import lru_cache
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from src.ui.gui.situation3d.aircraft_model_style import (
    DEFAULT_AIRCRAFT_MODEL_TYPE,
    AircraftModelType,
    available_model_options,
    create_aircraft_model_style,
)
from src.ui.gui.situation3d.terrain_field import (
    DEFAULT_TERRAIN_RESOLUTION,
    TerrainField,
    TerrainRiskZone,
    peek_terrain_field,
    risk_zones_from_layout,
    terrain_extent_from_layout,
    load_terrain_layout,
)
from src.ui.gui.view_models import (
    ObstacleView,
    ReferenceRoute,
    Snapshot,
    reference_route_points,
)

MAX_ROUTE_POINTS_PER_SEGMENT = 32
ROUTE_DASH_LENGTH_M = 140.0
ROUTE_DASH_GAP_M = 90.0
MAX_ROUTE_DASHES_PER_SEGMENT = 96
ROUTE_DASH_WIDTH_M = 16.0
ROUTE_DASH_COLOR = "#22d3ee"
BLOCKED_ROUTE_DASH_COLOR = "#ff5a45"
DEFAULT_GROUND_MARGIN_M = 420.0
DEFAULT_TERRAIN_SPAN_M = 20000.0
DEFAULT_TERRAIN_RELIEF_M = 2200.0


class TrailPayloadState:
    """记录 3D 接收端已消费的尾迹游标。注意：状态属于单个 3D 窗口，不能跨窗口共享。"""

    def __init__(self) -> None:
        """初始化空游标表，确保每架飞机第一次出现时发送 reset。"""

        self._cursors: dict[str, tuple[int, int, int]] = {}
        # observed 记录当前消息看到的数据队尾，eligible 只保存上一条展示消息的地平线。
        # 两套游标分离后，高倍频一次补入多点也不会让历史网格抢到补间飞机前方。
        # 同一时刻重推也算下一条消息；QML 展示队列会等当前补间完成后再按序应用它。
        self._observed_ends: dict[str, tuple[int, int]] = {}
        self._eligible_ends: dict[str, tuple[int, int]] = {}
        # 坐标锚独立于 TrailBuffer 生命周期；队首越过展示地平线后仍能连接上一飞机位置。
        self._observed_anchors: dict[str, tuple[int, object, object]] = {}
        self._eligible_anchors: dict[str, tuple[int, object, object]] = {}

    def begin_frame(self) -> None:
        """开始构造一条展示消息。注意：本轮只能固化上一条消息观察到的队尾。"""

        # 新展示消息只允许固化上一条消息已观察到的尾后序号；本轮批量点继续留在活动末段。
        self._eligible_ends = dict(self._observed_ends)
        self._eligible_anchors = dict(self._observed_anchors)

    def presentation_length(
        self,
        node_id: str,
        generation: int,
        first_sequence: int,
        end_sequence: int,
        trail: Sequence[object],
    ) -> int:
        """返回本帧可固化的稳定前缀长度。注意：首次出现可直接提交完整队列。"""

        point_count = len(trail)
        self._observed_ends[node_id] = (generation, end_sequence)
        if point_count:
            self._observed_anchors[node_id] = (
                generation,
                trail[-2] if point_count >= 2 else trail[-1],
                trail[-1],
            )
        eligible = self._eligible_ends.get(node_id)
        if eligible is None or eligible[0] != generation:
            return point_count
        # 允许稳定前缀清空；活动小网格会改用独立保存的上一展示锚，不抢用本轮新队首。
        eligible_length = eligible[1] - first_sequence
        return min(point_count, max(0, eligible_length))

    def presentation_anchor(self, node_id: str, generation: int) -> tuple[object, object] | None:
        """返回上一条消息的末段方向点与队尾锚。注意：锚可已从数据队列淘汰。"""

        anchor = self._eligible_anchors.get(node_id)
        if anchor is None or anchor[0] != generation:
            return None
        return anchor[1], anchor[2]

    def cursor(self, node_id: str) -> tuple[int, int, int] | None:
        """返回飞机上次发送的 generation、队首序号和尾后序号。"""

        return self._cursors.get(node_id)

    def update(self, node_id: str, generation: int, first_sequence: int, end_sequence: int) -> None:
        """提交飞机最新游标。注意：仅在对应 payload 已构造成功后调用。"""

        self._cursors[node_id] = (generation, first_sequence, end_sequence)

    def discard(self, node_id: str) -> None:
        """丢弃已移除飞机或空尾迹的全部展示游标。"""

        self._cursors.pop(node_id, None)
        self._observed_ends.pop(node_id, None)
        self._eligible_ends.pop(node_id, None)
        self._observed_anchors.pop(node_id, None)
        self._eligible_anchors.pop(node_id, None)

    def retain(self, node_ids: set[str]) -> None:
        """只保留本帧仍有 ribbon 的飞机，防止消失后重现时误发 delta。"""

        for node_id in set(self._cursors) - node_ids:
            self._cursors.pop(node_id, None)
        for node_id in set(self._observed_ends) - node_ids:
            self._observed_ends.pop(node_id, None)
        for node_id in set(self._eligible_ends) - node_ids:
            self._eligible_ends.pop(node_id, None)
        for node_id in set(self._observed_anchors) - node_ids:
            self._observed_anchors.pop(node_id, None)
        for node_id in set(self._eligible_anchors) - node_ids:
            self._eligible_anchors.pop(node_id, None)

# 3D payload 设计说明：
# 1. QML 侧只接收 JSON 字符串，因此 payload 不能携带完整高度场大数组。
# 2. 布局地形只把 layoutFile、resolution、中心和范围传给 TerrainGeometry。
# 3. TerrainGeometry 在 Python/QML 类型内部生成网格，避免 JSON 序列化数十 MB 数据。
# 4. riskZones 是语义数据；riskZoneLines 同时承载真实障碍边界和无障碍库旧场景的兼容提示。
# 5. 风险线保持米制 Quick3D 坐标，复用 TrailRibbonGeometry 的三角带实现细线。
# 6. 旧配置没有 terrain_display_file 时，payload 仍然走 procedural 地形路径。
# 7. 旧路径的 surface.width/depth/height 字段保持不变，避免历史 QML 绑定失效。
# 8. 新布局模式只影响显示层，不改变 Snapshot 的 ENU 坐标、航线和障碍语义。
# 9. 障碍物 obstacles 表示避障模块二维障碍；riskZones 优先由当前启用障碍生成，无避障数据时才回退布局标记。
# 10. 航线和风险细线使用点数组；尾迹额外携带队列序号，使几何层能原子识别追加和弹头。
# 11. 兼容红色风险线宽固定为 7m，明显细于历史粗网纹和默认尾迹。
# 12. 兼容淡青缓冲圈拆成 24 个短弧，视觉上是虚线，QML 不需要计算三角函数。
# 13. 真实障碍转换成地形局部坐标，供 TerrainGeometry 直接混入顶点色。
# 14. 圆形并入安全间距，多边形保留边界与 clearance，由几何层做圆角膨胀。
# 15. 布局文件读取使用 lru_cache，实时刷新快照时不会重复解析 JSON。
# 16. 文件不可读只回退旧地形，不阻断 3D 窗口打开。
# 17. 但 terrain_field 的单元测试会直接读取正式布局，确保验收文件本身有效。
# 18. 相机 bounds 仍由飞机、航线和障碍推导；布局地形范围由 terrain payload 独立声明。
# 19. 这样无地形配置时相机行为完全沿用旧场景。
# 20. 有地形配置时 QML 的 terrainSpan 会辅助俯视/侧视按钮拉开距离。
# 21. riskZones 计数进入 sceneSummary，便于截图时确认布局风险数据已到达 QML。
# 22. 路径字段使用绝对路径传给 QML，避免工作目录变化导致 TerrainGeometry 找不到文件。
# 23. `terrain_display_file` 可以由 Snapshot 或 build_scene_payload 参数传入，测试可直接覆盖。
# 24. payload 字段名使用 QML 现有 camelCase 风格，减少绑定样板。
# 25. 这里不做 P2 的可通行性判断；S 绕航线仅作为冻结布局参考线。
# 26. 当前生效航线来自 snapshot.route_segments，保证飞机飞行坐标语义不变。
# 27. 采用避障航线后，当前航线仍走 route_segments，保持既有渲染管线不变。
# 28. 被替代的配置航线走 blocked_route_segments，以红色虚线保留封锁状态。
# 29. 基础材质仍由 QML 控制，障碍风险色由 Python 地形几何按本文件提供的范围混合。
# 30. 只有布局回退风险区生成 10 条红色网格线，覆盖但不遮蔽山体细节。
# 31. 兼容线段 y 坐标取风险峰高度，缓冲圈 y 坐标取低值，形成上下层次。
# 32. 通过 json.dumps separators 压缩 pathValue，降低实时 payload 字符串体积。
# 33. 失败回退只吞 IO/JSON/值错误，普通编程错误仍由测试暴露。
# 34. terrain payload 的 ground 字段保留给后续需要地平面或雾墙时复用。
# 35. 这里的 Quick3D z 是 -north，所有风险线同样遵守 enu_to_quick3d。
# 36. route dash 切分仍按空间距离，巡航 900m 地形不会影响虚线节奏。
# 37. obstacles 仍保留兼容 payload 与相机包围盒用途，但 QML 不再把它们渲染成柱体或方盒。
# 38. scene_data 不缓存生成后的 mesh，避免和 TerrainGeometry 的重建生命周期竞争。
# 39. 如果布局文件变更，改变路径或清理 _cached_layout 即可刷新元数据。
# 40. P2 风险区与当前启用的真实障碍同源；旧场景未提供避障数据时继续使用正式 JSON 布局标记。
# 41. 注释中的“显示层”均指 GUI/QML，不包含 runner、algorithm 或 environment 模块。
# 42. 所有数值都用 float 输出，QML ListModel 不需要再做类型归一化。
# 43. 兼容风险线材质发光很弱，最终视觉厚度主要由几何 width 控制。
# 44. 兼容风险缓冲虚线 alpha 更低，避免抢过航线蓝色主视觉。
# 45. payload 中保留 label，后续需要 3D 标注时不用再改 terrain_field。
# 46. riskZoneFills 是真实障碍安全区的贴地半透明填充三角网，与告警边界同轮廓同呼吸相位。
# 47. 填充网按地形网格量级采样高度并整体抬升，QML 只动 opacity，不做逐帧几何重建。
# 48. 填充三角化结果按(轮廓,地形版本)缓存，10Hz 快照重推时直接复用缓存字符串。
# 49. 航点球只表示航线端点和航段交接点；圆弧中间采样点仅供虚线几何与场景包围盒使用。


def build_scene_payload(
    snapshot: Snapshot,
    obstacles: Iterable[ObstacleView] = (),
    *,
    clearance_m: float = 0.0,
    model_type: AircraftModelType = DEFAULT_AIRCRAFT_MODEL_TYPE,
    terrain_display_file: str | None = None,
    trail_state: TrailPayloadState | None = None,
) -> dict[str, object]:
    """把 UI 快照转换为 QML 场景数据。注意：输入仍采用 x/y/z=东/北/天。"""

    obstacle_views = list(obstacles)
    if trail_state is not None:
        # 一次 build 对应一条 QML 展示消息，固化进度由消息顺序而非仿真时间差驱动。
        trail_state.begin_frame()
    aircraft = [_aircraft_payload(node) for node in snapshot.nodes]
    aircraft_style = create_aircraft_model_style(model_type).style_payload()
    trail_ribbons = [
        ribbon
        for node in snapshot.nodes
        for ribbon in _trail_ribbon_payload(
            node.node_id,
            node.trail,
            _node_color(node.role, node.health),
            trail_state,
        )
    ]
    if trail_state is not None:
        # QML 会删除本帧缺失的 delegate；同步遗忘游标，保证该节点再出现时从 reset 开始。
        trail_state.retain({str(item["nodeId"]) for item in trail_ribbons})
    route_segments = _route_segments(snapshot)
    route_polylines = [_route_polyline(segment) for segment in route_segments]
    route_points = _route_marker_payload(route_segments)
    route_geometry_points = [
        point
        for polyline in route_polylines
        for point in _route_sample_payload(polyline)
    ]
    route_dashes = [
        dash
        for polyline in route_polylines
        for dash in _route_dash_payload(polyline)
    ]
    blocked_route_segments = _blocked_route_segments(snapshot)
    blocked_route_polylines = [_route_polyline(segment) for segment in blocked_route_segments]
    blocked_route_points = _route_marker_payload(blocked_route_segments, color=BLOCKED_ROUTE_DASH_COLOR)
    blocked_route_geometry_points = [
        point
        for polyline in blocked_route_polylines
        for point in _route_sample_payload(polyline)
    ]
    blocked_route_dashes = [
        dash
        for polyline in blocked_route_polylines
        for dash in _route_dash_payload(polyline, color=BLOCKED_ROUTE_DASH_COLOR)
    ]
    obstacle_items = [_obstacle_payload(obstacle, clearance_m) for obstacle in obstacle_views if obstacle.enabled]
    # 相机包围盒同时纳入封锁航线，避免覆盖路线偏移较大时红线落到初始视野外。
    bounds = _scene_bounds(aircraft, route_geometry_points + blocked_route_geometry_points, obstacle_items)
    display_file = terrain_display_file or snapshot.terrain_display_file
    terrain = _terrain_payload(bounds, display_file, obstacle_views)
    risk_zones = list(terrain.get("riskZones", []))
    if obstacle_views:
        # 真实障碍只有精确边界参与呼吸；旧外接圆网格和缓冲圈都不再叠加到地形上。
        risk_zone_lines = _obstacle_boundary_payloads(obstacle_views, clearance_m, terrain.get("surface", {}))
        # 填充覆盖层与边界同源同轮廓，QML 侧共用同一 pulse 属性保证同相位闪烁。
        risk_zone_fills = _obstacle_fill_payloads(obstacle_views, clearance_m, terrain.get("surface", {}))
        risk_zone_buffers: list[dict[str, object]] = []
        # 填充层的呼吸最低值就是静态基色：不再额外烘焙地形顶点色，避免同一区域红两次、
        # 也让呼吸参数归零时才能真正做到"没有红色"。
        terrain_risk_areas: list[dict[str, object]] = []
    else:
        # 兼容布局风险峰不闪烁，也不生成填充覆盖层；这类旧场景没有呼吸层承接静态基色，
        # 因此仍烘焙地形顶点色，维持旧场景视觉不变。
        risk_zone_fills = []
        raw_risk_zone_lines = terrain.get("riskZoneLines")
        risk_zone_lines = list(raw_risk_zone_lines) if isinstance(raw_risk_zone_lines, list) else []
        if raw_risk_zone_lines is None:
            risk_zone_lines = [
                line
                for zone in risk_zones
                for line in _risk_zone_line_payload(zone)
            ]
        raw_risk_zone_buffers = terrain.get("riskZoneBuffers")
        risk_zone_buffers = list(raw_risk_zone_buffers) if isinstance(raw_risk_zone_buffers, list) else []
        if raw_risk_zone_buffers is None:
            risk_zone_buffers = [
                dash
                for zone in risk_zones
                for dash in _risk_zone_buffer_payload(zone)
            ]
        terrain_risk_areas = _terrain_risk_area_payloads(
            obstacle_views,
            clearance_m,
            terrain.get("surface", {}),
            risk_zones,
        )
    camera_payload = _camera_payload(bounds, aircraft)
    if terrain.get("surface", {}).get("mode") == "layout":
        camera_payload = _layout_camera_payload(terrain["surface"])
    # 静态内容签名：QML 据此决定是否重建航线/障碍/风险区模型，避免每帧 clear+append 造成卡顿。
    static_key = _static_content_key(
        route_points,
        route_dashes,
        blocked_route_points,
        blocked_route_dashes,
        obstacle_items,
        risk_zones,
        risk_zone_lines,
        risk_zone_buffers,
        risk_zone_fills,
        terrain_risk_areas,
    )
    return {
        "staticKey": static_key,
        "time": snapshot.time,
        "runState": snapshot.run_state,
        "controlReport": snapshot.control_report,
        "aircraft": aircraft,
        "trailRibbons": trail_ribbons,
        "aircraftStyle": aircraft_style,
        "modelOptions": available_model_options(),
        "routePoints": route_points,
        "routeDashes": route_dashes,
        "blockedRoutePoints": blocked_route_points,
        "blockedRouteDashes": blocked_route_dashes,
        "obstacles": obstacle_items,
        "terrain": terrain,
        "riskZones": risk_zones,
        "riskZoneLines": risk_zone_lines,
        "riskZoneBuffers": risk_zone_buffers,
        "riskZoneFills": risk_zone_fills,
        "terrainRiskAreas": terrain_risk_areas,
        "camera": camera_payload,
        "counts": {
            "aircraft": len(aircraft),
            "trailRibbons": len(trail_ribbons),
            "routePoints": len(route_points),
            "routeDashes": len(route_dashes),
            "blockedRoutePoints": len(blocked_route_points),
            "blockedRouteDashes": len(blocked_route_dashes),
            "obstacles": len(obstacle_items),
            "riskZones": len(risk_zones),
        },
    }


def _static_content_key(*static_groups: list) -> str:
    """计算静态显示内容签名。注意：内容不变时签名稳定，QML 跳过静态模型重建。"""

    # 航线/障碍/风险区在一次运行中基本不变，序列化摘要的开销远小于每帧重建 QML 对象。
    digest = hashlib.md5()
    for group in static_groups:
        digest.update(json.dumps(group, sort_keys=True, ensure_ascii=False).encode("utf-8"))
    return digest.hexdigest()


def enu_to_quick3d(east_m: float, north_m: float, up_m: float) -> dict[str, float]:
    """把 ENU 坐标映射到 Qt Quick 3D 坐标。注意：项目内部坐标不随此函数改变。"""

    # Qt Quick 3D 的屏幕前后轴与项目 north 方向相反，只在显示层翻转 z。
    return {
        "x": float(east_m),
        "y": float(up_m),
        "z": float(-north_m),
    }


def _aircraft_payload(node) -> dict[str, object]:  # noqa: ANN001
    """生成单机显示数据。注意：飞机朝向只用于显示，不回写控制器。"""

    coord = enu_to_quick3d(node.x, node.y, node.altitude)
    return {
        "nodeId": node.node_id,
        "role": node.role,
        "health": node.health,
        "color": _node_color(node.role, node.health),
        "yawDeg": _heading_yaw_deg(node.vx, node.vy),
        "speed": math.hypot(node.vx, node.vy),
        **coord,
    }


def _trail_ribbon_payload(
    node_id: str,
    trail: Sequence[object],
    color: str,
    trail_state: TrailPayloadState | None = None,
) -> list[dict[str, object]]:
    """生成历史线带与活动末段数据。注意：活动末段从稳定队尾连接飞机实时位置。"""

    if not trail:
        if trail_state is not None:
            trail_state.discard(node_id)
        return []
    trail_length = len(trail)
    metadata = _trail_metadata(trail, trail_length)
    if trail_state is not None and metadata is not None:
        generation, first_sequence, end_sequence = metadata
        trail_length = trail_state.presentation_length(
            node_id,
            generation,
            first_sequence,
            end_sequence,
            trail,
        )
    # 数据队列保留全部固定点；历史网格延后一条展示消息，活动末段始终从已展示队尾出发。
    stream = _trail_stream_value(node_id, trail, trail_length, trail_state)
    if trail_length:
        tip_start = _trail_quick3d_point(trail[trail_length - 1])
        tip_previous = _trail_quick3d_point(
            trail[trail_length - 2] if trail_length >= 2 else trail[trail_length - 1]
        )
    else:
        # 短窗口可能已淘汰上一队尾；用窗口级展示锚连接，绝不提前采用本轮新队首。
        if trail_state is None or metadata is None:
            return []
        anchor = trail_state.presentation_anchor(node_id, metadata[0])
        if anchor is None:
            return []
        tip_previous = _trail_quick3d_point(anchor[0])
        tip_start = _trail_quick3d_point(anchor[1])
    return [
        {
            "nodeId": node_id,
            "color": color,
            "width": 44.0,
            "pathValue": json.dumps(stream, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
            "tipPreviousX": tip_previous[0],
            "tipPreviousY": tip_previous[1],
            "tipPreviousZ": tip_previous[2],
            "tipStartX": tip_start[0],
            "tipStartY": tip_start[1],
            "tipStartZ": tip_start[2],
        }
    ]


def _trail_stream_value(
    node_id: str,
    trail: Sequence[object],
    trail_length: int,
    trail_state: TrailPayloadState | None,
) -> dict[str, object]:
    """编码稳定历史的 reset 或 delta。注意：队列外实时端点不会进入本数据流。"""

    source_length = len(trail)
    trail_length = max(0, min(source_length, int(trail_length)))
    metadata = _trail_metadata(trail, source_length)
    if trail_state is not None and metadata is not None:
        generation, first_sequence, end_sequence = metadata
        # 接收端游标只覆盖可展示前缀，尚未固化的批量点继续留在活动末段之外。
        end_sequence -= source_length - trail_length
        previous = trail_state.cursor(node_id)
        if previous is not None:
            old_generation, old_first, old_end = previous
            removed_count = first_sequence - old_first
            added_count = end_sequence - old_end
            can_append = (
                generation == old_generation
                and 0 <= removed_count <= old_end - old_first
                and added_count >= 0
                and first_sequence <= old_end
                and trail_length == old_end - first_sequence + added_count
            )
            if can_append:
                # TrailSnapshot 支持切片；只转换新增项，使稳态每帧开销与历史长度无关。
                added_items = trail[trail_length - added_count : trail_length] if added_count else ()
                added_points = _trail_quick3d_points(added_items)
                trail_state.update(node_id, generation, first_sequence, end_sequence)
                return {
                    "op": "delta",
                    "generation": generation,
                    "firstSequence": first_sequence,
                    "endSequence": end_sequence,
                    "removedCount": removed_count,
                    "addedPoints": added_points,
                }
    if metadata is None:
        generation, first_sequence, end_sequence = 0, 0, trail_length
    else:
        generation, first_sequence, end_sequence = metadata
        end_sequence -= source_length - trail_length
    path = _trail_quick3d_points(trail[:trail_length])
    if trail_state is not None and metadata is not None:
        trail_state.update(node_id, generation, first_sequence, end_sequence)
    return {
        "op": "reset",
        "generation": generation,
        "firstSequence": first_sequence,
        "endSequence": end_sequence,
        "points": path,
    }


def _trail_metadata(trail: Sequence[object], trail_length: int) -> tuple[int, int, int] | None:
    """读取稳定队列游标。注意：普通列表或长度不一致的对象返回 None，强制 reset。"""

    generation = _optional_integer_attribute(trail, "generation")
    first_sequence = _optional_integer_attribute(trail, "first_sequence")
    end_sequence = _optional_integer_attribute(trail, "end_sequence")
    if generation is None or first_sequence is None or end_sequence is None:
        return None
    if end_sequence - first_sequence != trail_length:
        return None
    return generation, first_sequence, end_sequence


def _optional_integer_attribute(value: object, name: str) -> int | None:
    """读取非负整数属性。注意：缺失、非法和负数都表示该对象没有可靠增量游标。"""

    try:
        normalized = int(getattr(value, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return normalized if normalized >= 0 else None


def _trail_quick3d_points(points: Iterable[object]) -> list[list[float]]:
    """把一段 ENU 尾迹转换为 Quick3D 点列。注意：调用方可只传新增尾部切片。"""

    path: list[list[float]] = []
    for point in points:
        coord = enu_to_quick3d(point.x, point.y, point.altitude)
        # 已入队的坐标只做轴映射，禁止在数据桥内重新抽样或整条平滑。
        path.append([coord["x"], coord["y"], coord["z"]])
    return path


def _trail_quick3d_point(point: object) -> list[float]:
    """转换单个尾迹点，供活动末段锚点复用同一 ENU 轴映射。"""

    coord = enu_to_quick3d(point.x, point.y, point.altitude)
    return [coord["x"], coord["y"], coord["z"]]


def _route_marker_payload(
    routes: Sequence[ReferenceRoute], *, color: str = ROUTE_DASH_COLOR
) -> list[dict[str, object]]:
    """生成航点球数据。注意：只保留语义端点，并去掉相邻航段共用的交接点。"""

    markers: list[dict[str, object]] = []
    for route in routes:
        endpoints = (
            (route.start_x, route.start_y, route.start_altitude),
            (route.end_x, route.end_y, route.end_altitude),
        )
        for east_m, north_m, altitude_m in endpoints:
            coord = enu_to_quick3d(east_m, north_m, altitude_m)
            if markers and all(
                abs(float(markers[-1][axis]) - coord[axis]) <= 1e-6
                for axis in ("x", "y", "z")
            ):
                continue
            markers.append({"color": color, "size": 9.0, **coord})
    return markers


def _route_sample_payload(polyline: list[tuple[float, float, float]]) -> list[dict[str, object]]:
    """生成航线几何采样点。注意：仅供场景包围盒使用，不暴露为 QML 航点球。"""

    return [{"x": x, "y": y, "z": z} for x, y, z in polyline]


def _route_dash_payload(polyline: list[tuple[float, float, float]], *, color: str = ROUTE_DASH_COLOR) -> list[dict[str, object]]:
    """生成 3D 航线虚线段。注意：每段 dash 复用 ribbon 几何，宽度比尾迹更细。"""

    if len(polyline) < 2:
        return []
    # 先把折线转成累计里程，再按 dash/gap 周期切片；圆弧采样点会自然落入切片中。
    cumulative = _polyline_distances(polyline)
    total_length = cumulative[-1]
    if total_length <= 1e-6:
        return []

    dashes: list[dict[str, object]] = []
    dash_length, period = _route_dash_layout(total_length)
    dash_start = 0.0
    while dash_start < total_length and len(dashes) < MAX_ROUTE_DASHES_PER_SEGMENT:
        dash_end = min(dash_start + dash_length, total_length)
        dash_path = _polyline_slice(polyline, cumulative, dash_start, dash_end)
        if len(dash_path) >= 2:
            # 每段 dash 独立成一个 ribbon，材质层保持连续线宽，段间空隙由模型缺失形成。
            dashes.append(
                {
                    "color": color,
                    "width": ROUTE_DASH_WIDTH_M,
                    "pathValue": json.dumps(dash_path, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
                }
            )
        # 无论当前 dash 是否因退化被跳过，都按完整周期推进，保持虚线节奏稳定。
        dash_start += period
    return dashes


def _route_dash_layout(total_length: float) -> tuple[float, float]:
    """返回 dash 长度和周期。注意：超长航段会放大周期，限制 QML delegate 数量。"""

    base_period = ROUTE_DASH_LENGTH_M + ROUTE_DASH_GAP_M
    estimated_count = math.ceil(total_length / base_period)
    if estimated_count <= MAX_ROUTE_DASHES_PER_SEGMENT:
        return ROUTE_DASH_LENGTH_M, base_period
    # 保留原 dash/gap 比例，把同样的虚线节奏均匀铺满整条长航段。
    period = total_length / MAX_ROUTE_DASHES_PER_SEGMENT
    dash_ratio = ROUTE_DASH_LENGTH_M / base_period
    return period * dash_ratio, period


def _route_segments(snapshot: Snapshot) -> list[ReferenceRoute]:
    """返回 3D 需要绘制的参考航段。注意：对齐俯视图的多航段优先、当前航段兜底规则。"""

    if snapshot.route_segments:
        return snapshot.route_segments
    # 单航段或控制器只暴露当前航段时，仍然要显示目标航段虚线。
    if snapshot.route is not None:
        return [snapshot.route]
    return []


def _blocked_route_segments(snapshot: Snapshot) -> list[ReferenceRoute]:
    """返回被封锁的原始航段。注意：缺失时保持空列表，不回退当前航线。"""

    # 和 _route_segments 不同：这里没有"至少显示一条"的兜底语义，
    # 没有被封锁的航线就应该什么都不画，否则会在非避障场景里凭空多出一条红线。
    return list(snapshot.blocked_route_segments)


def _route_polyline(route: ReferenceRoute) -> list[tuple[float, float, float]]:
    """把单条参考航段采样为 Quick3D 折线点。注意：高度随采样序号线性插值。"""

    xy_points = reference_route_points(route)
    sampled_xy = _evenly_sample(xy_points, MAX_ROUTE_POINTS_PER_SEGMENT)
    count = max(1, len(sampled_xy) - 1)
    points: list[tuple[float, float, float]] = []
    for index, (east_m, north_m) in enumerate(sampled_xy):
        # reference_route_points 只给水平位置，高度按采样序号沿航段端点线性插值。
        mix = index / count
        altitude_m = route.start_altitude + (route.end_altitude - route.start_altitude) * mix
        coord = enu_to_quick3d(east_m, north_m, altitude_m)
        points.append((coord["x"], coord["y"], coord["z"]))
    return points


def _polyline_distances(points: list[tuple[float, float, float]]) -> list[float]:
    """返回折线各点累计距离。注意：3D 虚线按空间距离分段，爬升航段间距也稳定。"""

    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        # 用三维距离切 dash，爬升/下降段不会因为水平距离短而虚线过密。
        distances.append(distances[-1] + math.dist(previous, current))
    return distances


def _polyline_slice(
    points: list[tuple[float, float, float]],
    distances: list[float],
    start_m: float,
    end_m: float,
) -> list[list[float]]:
    """截取折线上的一段路径。注意：保留中间采样点，使圆弧 dash 仍贴合曲线。"""

    sliced: list[list[float]] = [_interpolate_polyline(points, distances, start_m)]
    for point, distance in zip(points[1:-1], distances[1:-1]):
        # 中间点只有落在当前 dash 里才保留，保证每个 dash 自身仍是一条小折线。
        if start_m < distance < end_m:
            sliced.append([point[0], point[1], point[2]])
    sliced.append(_interpolate_polyline(points, distances, end_m))
    return sliced


def _interpolate_polyline(
    points: list[tuple[float, float, float]],
    distances: list[float],
    target_m: float,
) -> list[float]:
    """按累计距离在线性折线上插值。注意：返回 QML 可直接 JSON 化的三元组。"""

    clamped = max(0.0, min(target_m, distances[-1]))
    for index in range(1, len(distances)):
        if clamped <= distances[index]:
            previous_distance = distances[index - 1]
            span = distances[index] - previous_distance
            # 退化子段直接取前点，避免零长航段造成除零。
            mix = 0.0 if span <= 1e-9 else (clamped - previous_distance) / span
            previous = points[index - 1]
            current = points[index]
            return [
                previous[0] + (current[0] - previous[0]) * mix,
                previous[1] + (current[1] - previous[1]) * mix,
                previous[2] + (current[2] - previous[2]) * mix,
            ]
    last = points[-1]
    return [last[0], last[1], last[2]]


def _obstacle_payload(obstacle: ObstacleView, clearance_m: float) -> dict[str, object]:
    """生成障碍兼容与相机包围盒数据。注意：QML 不再把这些尺寸直接渲染成实体柱体。"""

    # 安全间距在显示层膨胀半径/包围盒，便于和避障预览里的风险边界对应。
    safe_clearance = max(0.0, float(clearance_m))
    column_height_m = 720.0
    if obstacle.kind == "circle":
        radius = max(1.0, obstacle.radius + safe_clearance)
        coord = enu_to_quick3d(obstacle.center_x, obstacle.center_y, column_height_m / 2.0)
        return {
            "kind": "circle",
            "id": obstacle.obstacle_id,
            "radius": radius,
            "width": radius * 2.0,
            "depth": radius * 2.0,
            "height": column_height_m,
            **coord,
        }
    min_x, max_x, min_y, max_y = _obstacle_bounds(obstacle)
    width = max(1.0, max_x - min_x + safe_clearance * 2.0)
    depth = max(1.0, max_y - min_y + safe_clearance * 2.0)
    coord = enu_to_quick3d((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, column_height_m / 2.0)
    return {
        "kind": "box",
        "id": obstacle.obstacle_id,
        "radius": max(width, depth) / 2.0,
        "width": width,
        "depth": depth,
        "height": column_height_m,
        **coord,
    }


def _terrain_risk_area_payloads(
    obstacles: list[ObstacleView],
    clearance_m: float,
    surface: object,
    fallback_zones: list[dict[str, object]],
) -> list[dict[str, object]]:
    """生成地形顶点着色范围。注意：坐标转为地形模型局部坐标，复杂多边形保留真实轮廓。"""

    surface_data = surface if isinstance(surface, dict) else {}
    origin_x = float(surface_data.get("x", 0.0))
    origin_z = float(surface_data.get("z", 0.0))
    safe_clearance = max(0.0, float(clearance_m))
    areas: list[dict[str, object]] = []
    for obstacle in obstacles:
        if not obstacle.enabled:
            continue
        if obstacle.kind == "circle":
            # 圆形直接把规划安全间距并入半径，着色边界与旧柱体表达的膨胀范围一致。
            areas.append(
                {
                    "id": obstacle.obstacle_id,
                    "kind": "circle",
                    "center": [obstacle.center_x - origin_x, -obstacle.center_y - origin_z],
                    "radius": max(1.0, obstacle.radius + safe_clearance),
                }
            )
            continue
        vertices = obstacle.vertices if obstacle.vertices else _obstacle_corner_points(obstacle)
        # 多边形不转外接圆或包围盒；地形几何层按边界距离完成圆角膨胀。
        areas.append(
            {
                "id": obstacle.obstacle_id,
                "kind": "polygon",
                "points": [[east - origin_x, -north - origin_z] for east, north in vertices],
                "clearance": safe_clearance,
            }
        )

    if obstacles:
        # 配置存在但全部禁用表示用户明确隐藏障碍，不能回退布局里的旧风险峰。
        return areas
    for zone in fallback_zones:
        areas.append(
            {
                "id": str(zone.get("id", "风险区")),
                "kind": "circle",
                "center": [float(zone.get("x", 0.0)) - origin_x, float(zone.get("z", 0.0)) - origin_z],
                "radius": max(1.0, float(zone.get("radius", 1.0))),
            }
        )
    return areas


def _obstacle_boundary_payloads(
    obstacles: list[ObstacleView], clearance_m: float, surface: object
) -> list[dict[str, object]]:
    """生成贴地的障碍告警边界。注意：只有真实且启用的障碍边界携带呼吸标志。"""

    surface_data = surface if isinstance(surface, dict) else {}
    field: TerrainField | None = None
    if surface_data.get("mode") == "layout":
        layout_file = str(surface_data.get("layoutFile", ""))
        resolution = int(surface_data.get("resolution", DEFAULT_TERRAIN_RESOLUTION))
        if layout_file:
            # 高度场未就绪时先用基准面；fieldReady 翻转会改变 staticKey 并自动换成贴地路径。
            field = _cached_terrain_field(layout_file, resolution)

    safe_clearance = max(0.0, float(clearance_m))
    boundaries: list[dict[str, object]] = []
    for obstacle in obstacles:
        if not obstacle.enabled:
            continue
        horizontal_points = _obstacle_boundary_points(obstacle, safe_clearance)
        if len(horizontal_points) < 3:
            continue
        # 一条障碍只生成一个闭合 ribbon；动画只改材质透明度，不触碰地形顶点色。
        path = _terrain_boundary_path(horizontal_points, surface_data, field)
        boundaries.append(
            {
                "color": "#ff684f",
                "width": 7.0,
                "pulse": True,
                "pathValue": json.dumps(path, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
            }
        )
    return boundaries


def _obstacle_boundary_points(obstacle: ObstacleView, clearance_m: float) -> list[tuple[float, float]]:
    """返回障碍安全区的水平边界。注意：坐标仍为 ENU east/north，末点暂不闭合。"""

    if obstacle.kind == "circle":
        radius = max(1.0, float(obstacle.radius) + clearance_m)
        # 圆周按周长自适应采样，同时保留最低 48 段，近看不会呈现明显多边形折角。
        segment_count = max(48, min(160, int(math.ceil(math.tau * radius / 80.0))))
        return [
            (
                float(obstacle.center_x) + radius * math.cos(index * math.tau / segment_count),
                float(obstacle.center_y) + radius * math.sin(index * math.tau / segment_count),
            )
            for index in range(segment_count)
        ]

    vertices = list(obstacle.vertices) if obstacle.vertices else _obstacle_corner_points(obstacle)
    if len(vertices) >= 2 and math.dist(vertices[0], vertices[-1]) <= 1e-6:
        # 配置允许显式重复首点；先去重再交给圆角外扩，避免产生零长边。
        vertices.pop()
    if len(vertices) < 3 or clearance_m <= 0.0:
        return vertices

    # 与俯视避障预览复用同一圆角 Minkowski 外扩，确保 2D/3D 告警边界语义一致。
    from src.ui.gui.avoidance_tools import _rounded_inflated_polygon_points

    return _rounded_inflated_polygon_points(vertices, clearance_m)


def _terrain_boundary_path(
    boundary: list[tuple[float, float]], surface: dict[str, object], field: TerrainField | None
) -> list[list[float]]:
    """把 ENU 闭合轮廓逐点披挂到地形表面。注意：输出转换为 Quick3D x/y/z。"""

    width = max(1.0, float(surface.get("width", DEFAULT_TERRAIN_SPAN_M)))
    depth = max(1.0, float(surface.get("depth", DEFAULT_TERRAIN_SPAN_M)))
    resolution = int(surface.get("resolution", 0))
    grid_segments = max(1, resolution - 1) if resolution > 1 else 191
    # 采样步长略小于地形网格，轮廓跨过陡坡时不会悬空或切入山体。
    sample_spacing = max(24.0, min(110.0, max(width, depth) / grid_segments * 0.75))
    path: list[list[float]] = []
    for start, end in zip(boundary, boundary[1:] + boundary[:1]):
        edge_length = math.dist(start, end)
        segments = max(1, int(math.ceil(edge_length / sample_spacing)))
        # 每条边不重复写末点；全部边完成后统一追加首点，保持 ribbon 闭合且无零长中间段。
        for index in range(segments):
            ratio = index / segments
            east = start[0] + (end[0] - start[0]) * ratio
            north = start[1] + (end[1] - start[1]) * ratio
            x = float(east)
            z = -float(north)
            y = _terrain_surface_height(surface, field, x, z, 34.0)
            path.append([x, y, z])
    if path:
        path.append(list(path[0]))
    return path


def _terrain_surface_height(
    surface: dict[str, object], field: TerrainField | None, x_m: float, z_m: float, offset_m: float
) -> float:
    """采样风险边界的世界高度。注意：边界始终悬浮少量距离以规避地形 z-fighting。"""

    if field is not None:
        # 正式地形显示可能叠加受限岩脊位移；边界必须采样显示面，不能埋进仅显示层的沟脊里。
        return _sample_field_display_height(field, x_m, -z_m) + offset_m
    surface_y = float(surface.get("y", 0.0))
    if surface.get("mode") != "procedural":
        # 布局高度场异步生成期间只显示低位占位线；下一次静态刷新会替换成真实高度。
        return surface_y + offset_m

    # procedural 地形使用与 TerrainGeometry 完全相同的标量高度函数，避免另造近似曲面。
    from src.ui.gui.situation3d.terrain_geometry import _height_value

    local_x = x_m - float(surface.get("x", 0.0))
    local_z = z_m - float(surface.get("z", 0.0))
    return surface_y + _height_value(
        local_x,
        local_z,
        max(1.0, float(surface.get("width", DEFAULT_TERRAIN_SPAN_M))),
        max(1.0, float(surface.get("depth", DEFAULT_TERRAIN_SPAN_M))),
        max(1.0, float(surface.get("height", DEFAULT_TERRAIN_RELIEF_M))),
    ) + offset_m


# 填充覆盖层整体抬升量：地形网格间距量级的线性插值在岩脊处误差可达十余米，
# 抬得太低会大面积穿地，太高又会悬空；20m 在验收视距(千米级)下不可分辨。
_FILL_HEIGHT_OFFSET_M = 20.0
# 单个危险区填充网的节点数上限，防止巨型障碍在低配机上生成过大的静态 mesh。
_FILL_MAX_GRID_NODES = 20000
# 填充色与告警边界同色系，视觉上是同一个"危险区"整体。
_RISK_FILL_COLOR = "#ff684f"


def _obstacle_fill_payloads(
    obstacles: list[ObstacleView], clearance_m: float, surface: object
) -> list[dict[str, object]]:
    """生成危险区贴地填充覆盖层。注意：轮廓与告警边界同源，QML 用同一 pulse 驱动透明度。"""

    surface_data = surface if isinstance(surface, dict) else {}
    safe_clearance = max(0.0, float(clearance_m))
    surface_key = _fill_surface_key(surface_data)
    fills: list[dict[str, object]] = []
    for obstacle in obstacles:
        if not obstacle.enabled:
            continue
        points = _obstacle_boundary_points(obstacle, safe_clearance)
        if len(points) < 3:
            continue
        # 轮廓取整到厘米级即可稳定缓存键，重复快照不再重复三角化。
        outline = tuple((round(float(east), 2), round(float(north), 2)) for east, north in points)
        mesh_value = _cached_fill_mesh(outline, surface_key)
        if not mesh_value:
            continue
        fills.append({"color": _RISK_FILL_COLOR, "meshValue": mesh_value})
    return fills


def _fill_surface_key(surface: dict[str, object]) -> tuple:
    """提取影响填充高度的地形字段。注意：revision 含高度场就绪标志，就绪后缓存自动失效。"""

    return (
        str(surface.get("mode", "")),
        str(surface.get("layoutFile", "")),
        str(surface.get("revision", "")),
        float(surface.get("x", 0.0)),
        float(surface.get("y", 0.0)),
        float(surface.get("z", 0.0)),
        float(surface.get("width", DEFAULT_TERRAIN_SPAN_M)),
        float(surface.get("depth", DEFAULT_TERRAIN_SPAN_M)),
        float(surface.get("height", DEFAULT_TERRAIN_RELIEF_M)),
        int(surface.get("resolution", 0) or 0),
    )


@lru_cache(maxsize=32)
def _cached_fill_mesh(outline: tuple[tuple[float, float], ...], surface_key: tuple) -> str:
    """按(轮廓,地形版本)缓存填充三角网。注意：10Hz 重推时直接返回既有字符串。"""

    mode, layout_file, _revision, x, y, z, width, depth, height, resolution = surface_key
    surface = {
        "mode": mode,
        "x": x,
        "y": y,
        "z": z,
        "width": width,
        "depth": depth,
        "height": height,
        "resolution": resolution,
    }
    field = None
    if mode == "layout" and layout_file:
        # 高度场未就绪时先按基准面出图；revision 翻转后缓存键变化会自动重建贴地版本。
        field = _cached_terrain_field(layout_file, resolution or DEFAULT_TERRAIN_RESOLUTION)
    return _fill_mesh_value(outline, surface, field)


def _fill_mesh_value(
    outline: tuple[tuple[float, float], ...], surface: dict[str, object], field: TerrainField | None
) -> str:
    """把 ENU 闭合轮廓三角化成贴地填充网。注意：输出为 RiskFillGeometry 的 meshValue JSON。

    做法：在轮廓包围盒上铺与地形网格同量级的规则格网，逐节点采样显示高度；
    完全在多边形内部的格子直接出两个三角形，与边界相交的格子把多边形裁剪到
    格子矩形后按质心扇形三角化，裁剪点高度取格子四角高度的双线性插值。
    这样相邻格子在公共边上的高度插值一致，覆盖层不会出现裂缝。
    """

    # 与告警边界共用 Quick3D 平面坐标：x=east，z=-north。
    polygon = [(float(east), -float(north)) for east, north in outline]
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_z = min(point[1] for point in polygon)
    max_z = max(point[1] for point in polygon)
    if max_x - min_x <= 1e-6 or max_z - min_z <= 1e-6:
        return ""
    width = max(1.0, float(surface.get("width", DEFAULT_TERRAIN_SPAN_M)))
    depth = max(1.0, float(surface.get("depth", DEFAULT_TERRAIN_SPAN_M)))
    resolution = int(surface.get("resolution", 0) or 0)
    grid_segments = max(1, resolution - 1) if resolution > 1 else 191
    # 采样间距对齐地形网格量级：太粗贴不住岩脊，太细则静态 mesh 无谓膨胀。
    spacing = max(30.0, min(110.0, max(width, depth) / grid_segments))
    # 按包围盒宽高除以间距向上取整，保证格网覆盖到轮廓边界，不留未采样的窄边。
    columns = int(math.ceil((max_x - min_x) / spacing))
    rows = int(math.ceil((max_z - min_z) / spacing))
    if (columns + 1) * (rows + 1) > _FILL_MAX_GRID_NODES:
        # 巨型障碍按上限反推间距，保证节点数有界，宁可略粗也不能拖垮低配机。
        scale = math.sqrt((columns + 1) * (rows + 1) / _FILL_MAX_GRID_NODES)
        spacing *= scale
        columns = int(math.ceil((max_x - min_x) / spacing))
        rows = int(math.ceil((max_z - min_z) / spacing))

    # 奇偶规则与地形顶点色共用同一套几何判定，覆盖层轮廓和地形红晕保持一致。
    from src.ui.gui.situation3d.terrain_geometry import _points_inside_polygon

    polygon_array = np.asarray(polygon, dtype=np.float64)
    node_x = min_x + np.arange(columns + 1, dtype=np.float64)[None, :] * spacing
    node_z = min_z + np.arange(rows + 1, dtype=np.float64)[:, None] * spacing
    node_x = np.broadcast_to(node_x, (rows + 1, columns + 1))
    node_z = np.broadcast_to(node_z, (rows + 1, columns + 1))
    inside = _points_inside_polygon(node_x, node_z, polygon_array)

    # 节点高度整表采样一次，全部三角形共享，避免每个格子重复查询高度场。
    heights = [
        [
            _terrain_surface_height(surface, field, float(node_x[row, column]), float(node_z[row, column]), _FILL_HEIGHT_OFFSET_M)
            for column in range(columns + 1)
        ]
        for row in range(rows + 1)
    ]
    boundary_cells = _fill_boundary_cells(polygon, min_x, min_z, spacing, columns, rows)

    vertices: list[list[float]] = []
    triangles: list[int] = []
    node_index: dict[tuple[int, int], int] = {}

    def grid_vertex(row: int, column: int) -> int:
        """按需注册共享格点顶点，返回顶点索引。"""

        key = (row, column)
        cached = node_index.get(key)
        if cached is not None:
            return cached
        index = len(vertices)
        vertices.append(
            [
                round(float(node_x[row, column]), 2),
                round(heights[row][column], 2),
                round(float(node_z[row, column]), 2),
            ]
        )
        node_index[key] = index
        return index

    for row in range(rows):
        for column in range(columns):
            corner_flags = (
                bool(inside[row, column]),
                bool(inside[row, column + 1]),
                bool(inside[row + 1, column]),
                bool(inside[row + 1, column + 1]),
            )
            on_boundary = (row, column) in boundary_cells or (any(corner_flags) and not all(corner_flags))
            if not on_boundary:
                if not all(corner_flags):
                    continue
                # 内部整格直接出两个三角形，顶点走共享格点表。
                top_left = grid_vertex(row, column)
                top_right = grid_vertex(row, column + 1)
                bottom_left = grid_vertex(row + 1, column)
                bottom_right = grid_vertex(row + 1, column + 1)
                # 两个三角形都按逆时针绕序排列，和地形几何保持同一朝向，避免背面剔除翻面。
                triangles.extend((top_left, bottom_left, top_right, top_right, bottom_left, bottom_right))
                continue
            _append_clipped_cell(
                polygon,
                row,
                column,
                min_x,
                min_z,
                spacing,
                heights,
                vertices,
                triangles,
            )
    if not triangles:
        return ""
    return json.dumps({"v": vertices, "t": triangles}, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def _fill_boundary_cells(
    polygon: list[tuple[float, float]],
    min_x: float,
    min_z: float,
    spacing: float,
    columns: int,
    rows: int,
) -> set[tuple[int, int]]:
    """标记被多边形边穿过的格子。注意：步进取四分之一间距，避免斜穿漏标。"""

    cells: set[tuple[int, int]] = set()
    step = spacing / 4.0
    # 按边闭合遍历(最后一点连回第一点)，逐段沿边采样，间距细于格距才不会跳过短边穿过的格子。
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        segments = max(1, int(math.ceil(edge_length / step)))
        for index in range(segments + 1):
            ratio = index / segments
            x = start[0] + (end[0] - start[0]) * ratio
            z = start[1] + (end[1] - start[1]) * ratio
            # clamp 到 [0, columns/rows-1]，防止边界上的采样点因浮点误差越界访问格子表。
            column = min(columns - 1, max(0, int((x - min_x) / spacing)))
            row = min(rows - 1, max(0, int((z - min_z) / spacing)))
            cells.add((row, column))
    return cells


def _append_clipped_cell(
    polygon: list[tuple[float, float]],
    row: int,
    column: int,
    min_x: float,
    min_z: float,
    spacing: float,
    heights: list[list[float]],
    vertices: list[list[float]],
    triangles: list[int],
) -> None:
    """把多边形裁剪到单个格子并按质心扇形三角化。注意：高度取格子四角双线性插值。"""

    cell_min_x = min_x + column * spacing
    cell_min_z = min_z + row * spacing
    piece = _clip_polygon_to_rect(polygon, cell_min_x, cell_min_z, cell_min_x + spacing, cell_min_z + spacing)
    if len(piece) < 3:
        return
    # 面积过滤掉退化裁剪结果，避免上传零面积三角形。
    area = 0.0
    for (x0, z0), (x1, z1) in zip(piece, piece[1:] + piece[:1]):
        area += x0 * z1 - x1 * z0
    if abs(area) * 0.5 < spacing * spacing * 1e-4:
        return

    def bilinear_height(x: float, z: float) -> float:
        """格子内部双线性高度。注意：公共边只依赖两端格点，相邻格子结果一致无裂缝。"""

        tx = min(1.0, max(0.0, (x - cell_min_x) / spacing))
        tz = min(1.0, max(0.0, (z - cell_min_z) / spacing))
        top = heights[row][column] * (1.0 - tx) + heights[row][column + 1] * tx
        bottom = heights[row + 1][column] * (1.0 - tx) + heights[row + 1][column + 1] * tx
        return top * (1.0 - tz) + bottom * tz

    center_x = sum(point[0] for point in piece) / len(piece)
    center_z = sum(point[1] for point in piece) / len(piece)
    base = len(vertices)
    # 质心扇形对近似凸的裁剪块是精确覆盖；极端凹角只产生一格内的轻微溢色，可接受。
    vertices.append([round(center_x, 2), round(bilinear_height(center_x, center_z), 2), round(center_z, 2)])
    for x, z in piece:
        vertices.append([round(x, 2), round(bilinear_height(x, z), 2), round(z, 2)])
    for index in range(len(piece)):
        next_index = (index + 1) % len(piece)
        triangles.extend((base, base + 1 + index, base + 1 + next_index))


def _clip_polygon_to_rect(
    polygon: list[tuple[float, float]],
    min_x: float,
    min_z: float,
    max_x: float,
    max_z: float,
) -> list[tuple[float, float]]:
    """Sutherland-Hodgman 矩形裁剪。注意：裁剪窗口是凸矩形，多边形本身允许凹。"""

    def clip_edge(
        points: list[tuple[float, float]],
        keep: object,
        intersect: object,
    ) -> list[tuple[float, float]]:
        """对单条裁剪边执行一轮扫描。"""

        result: list[tuple[float, float]] = []
        for current, following in zip(points, points[1:] + points[:1]):
            current_in = keep(current)
            following_in = keep(following)
            if current_in:
                result.append(current)
                if not following_in:
                    result.append(intersect(current, following))
            elif following_in:
                result.append(intersect(current, following))
        return result

    def cross_x(boundary: float):
        """返回与竖直边界求交的函数。"""

        def intersect(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
            """按线性插值求线段 a-b 与该竖直边界的交点。"""

            ratio = (boundary - a[0]) / (b[0] - a[0])
            return boundary, a[1] + (b[1] - a[1]) * ratio

        return intersect

    def cross_z(boundary: float):
        """返回与水平边界求交的函数。"""

        def intersect(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
            """按线性插值求线段 a-b 与该水平边界的交点。"""

            ratio = (boundary - a[1]) / (b[1] - a[1])
            return a[0] + (b[0] - a[0]) * ratio, boundary

        return intersect

    result = list(polygon)
    for keep, intersect in (
        (lambda point: point[0] >= min_x, cross_x(min_x)),
        (lambda point: point[0] <= max_x, cross_x(max_x)),
        (lambda point: point[1] >= min_z, cross_z(min_z)),
        (lambda point: point[1] <= max_z, cross_z(max_z)),
    ):
        if not result:
            return []
        result = clip_edge(result, keep, intersect)
    return result


def _obstacle_bounds(obstacle: ObstacleView) -> tuple[float, float, float, float]:
    """计算障碍的 ENU 包围盒。注意：polygon/rect 共用此结果做 3D 近似。"""

    if obstacle.vertices:
        xs = [point[0] for point in obstacle.vertices]
        ys = [point[1] for point in obstacle.vertices]
        return min(xs), max(xs), min(ys), max(ys)
    return obstacle.min_x, obstacle.max_x, obstacle.min_y, obstacle.max_y


def _obstacle_corner_points(obstacle: ObstacleView) -> list[tuple[float, float]]:
    """把非多边形障碍近似为顶点集合。注意：统一后续风险区的质心和半径计算。"""

    if obstacle.kind == "circle":
        # 圆没有原生顶点；用八边形近似换取和 polygon/rect 共用同一套质心+最大半径公式，
        # 不必为圆再写一条独立的风险区分支。
        return [
            (
                obstacle.center_x + obstacle.radius * math.cos(index * math.tau / 8.0),
                obstacle.center_y + obstacle.radius * math.sin(index * math.tau / 8.0),
            )
            for index in range(8)
        ]
    # 矩形/旋转矩形按四角近似，和上面的圆形分支殊途同归，供下面统一取质心与半径。
    return [
        (obstacle.min_x, obstacle.min_y),
        (obstacle.max_x, obstacle.min_y),
        (obstacle.max_x, obstacle.max_y),
        (obstacle.min_x, obstacle.max_y),
    ]


def _risk_zones_from_obstacles(
    obstacles: Iterable[ObstacleView], field: TerrainField | None
) -> list[TerrainRiskZone]:
    """从当前生效的避障障碍集合生成风险区。注意：只处理已启用障碍，替代布局手工标记。"""

    zones: list[TerrainRiskZone] = []
    for obstacle in obstacles:
        # 禁用障碍不参与规划，风险罩面也不应该继续显示，否则用户取消勾选后画面对不上。
        if not obstacle.enabled:
            continue
        vertices = obstacle.vertices if obstacle.kind == "polygon" else _obstacle_corner_points(obstacle)
        if not vertices:
            continue
        center_x = sum(point[0] for point in vertices) / len(vertices)
        center_y = sum(point[1] for point in vertices) / len(vertices)
        # 外接圆半径而不是等效面积半径：宁可罩面略大一点覆盖整个障碍，也不要出现罩面切穿山体的情况。
        radius_m = max(math.hypot(x - center_x, y - center_y) for x, y in vertices)
        # 高度直接采样真实高度场，而不是信任障碍元数据里的 height_m——地形是唯一事实来源。
        height_m = _sample_field_height(field, center_x, center_y) if field is not None else 0.0
        zones.append(
            TerrainRiskZone(
                zone_id=obstacle.obstacle_id,
                label=obstacle.obstacle_id,
                east_m=center_x,
                north_m=center_y,
                radius_m=max(1.0, radius_m),
                height_m=height_m,
                # 缓冲圈固定比风险圈外扩 25%，和布局手工标记的量级保持一致，不需要额外配置项。
                buffer_radius_m=max(1.0, radius_m) * 1.25,
            )
        )
    return zones


def _scene_bounds(
    aircraft: list[dict[str, object]],
    route_points: list[dict[str, object]],
    obstacles: list[dict[str, object]],
) -> dict[str, float]:
    """计算场景包围盒。注意：用于地面、山体和初始相机范围。"""

    xs = [float(item["x"]) for item in aircraft + route_points]
    zs = [float(item["z"]) for item in aircraft + route_points]
    ys = [float(item["y"]) for item in aircraft + route_points]
    for obstacle in obstacles:
        half_width = float(obstacle["width"]) / 2.0
        half_depth = float(obstacle["depth"]) / 2.0
        xs.extend([float(obstacle["x"]) - half_width, float(obstacle["x"]) + half_width])
        zs.extend([float(obstacle["z"]) - half_depth, float(obstacle["z"]) + half_depth])
        ys.extend([0.0, float(obstacle["height"])])
    if not xs:
        xs = [-800.0, 800.0]
        zs = [-500.0, 500.0]
        ys = [0.0, 1200.0]
    min_x = min(xs) - DEFAULT_GROUND_MARGIN_M
    max_x = max(xs) + DEFAULT_GROUND_MARGIN_M
    min_z = min(zs) - DEFAULT_GROUND_MARGIN_M
    max_z = max(zs) + DEFAULT_GROUND_MARGIN_M
    return {
        "minX": min_x,
        "maxX": max_x,
        "minZ": min_z,
        "maxZ": max_z,
        "minY": min(ys),
        "maxY": max(ys),
        "centerX": (min_x + max_x) / 2.0,
        "centerZ": (min_z + max_z) / 2.0,
        "spanX": max_x - min_x,
        "spanZ": max_z - min_z,
    }


def _terrain_payload(
    bounds: dict[str, float], terrain_display_file: str | None = None, obstacles: Iterable[ObstacleView] = ()
) -> dict[str, object]:
    """生成连续高度场地形参数。注意：只影响 3D 显示背景，不改变仿真状态。"""

    if terrain_display_file:
        layout_payload = _layout_terrain_payload(terrain_display_file, obstacles)
        if layout_payload is not None:
            return layout_payload

    # 地图默认保持 20km x 20km，较大的仿真范围再按实际包围盒外扩。
    span_x = max(bounds["spanX"], DEFAULT_TERRAIN_SPAN_M)
    span_z = max(bounds["spanZ"], DEFAULT_TERRAIN_SPAN_M)
    center_x = bounds["centerX"]
    center_z = bounds["centerZ"]
    return {
        "ground": {
            "mode": "procedural",
            "x": center_x,
            "y": -8.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": 16.0,
        },
        "surface": {
            "mode": "procedural",
            "x": center_x,
            "y": 0.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": DEFAULT_TERRAIN_RELIEF_M,
            "layoutFile": "",
            "resolution": 0,
        },
        "riskZones": [],
    }


def _layout_terrain_payload(
    terrain_display_file: str, obstacles: Iterable[ObstacleView] = ()
) -> dict[str, object] | None:
    """生成布局地形 payload。注意：任何解析/数值/结构错误都回退旧地形并留诊断。"""

    layout_path = str(Path(terrain_display_file).resolve())
    try:
        layout, revision = _cached_layout(layout_path)
        extent = terrain_extent_from_layout(layout)
        # 合法 JSON 也可能携带错误类型或非有限值,包括风险区构造在内的全部转换
        # 都必须留在保护块内,任何一步失败都整体回退 procedural。
        for key in ("min_east_m", "max_east_m", "min_north_m", "max_north_m"):
            if not math.isfinite(float(extent[key])):
                raise ValueError(f"terrain extent {key} 非有限值")
        width = float(extent["max_east_m"]) - float(extent["min_east_m"])
        depth = float(extent["max_north_m"]) - float(extent["min_north_m"])
        if width <= 0.0 or depth <= 0.0:
            raise ValueError("terrain extent 端点顺序错误或范围为零")
        resolution = _layout_resolution(layout)
        effective_span = float(layout.get("map", {}).get("effective_extent_km", 32.0)) * 1000.0 if isinstance(layout.get("map"), dict) else 32000.0
        if not math.isfinite(effective_span) or effective_span <= 0.0:
            raise ValueError("effective_extent_km 非法")
        field = _cached_terrain_field(layout_path, resolution)
        center_east = (extent["min_east_m"] + extent["max_east_m"]) / 2.0
        center_north = (extent["min_north_m"] + extent["max_north_m"]) / 2.0
        obstacle_views = list(obstacles)
        # 是否回退取决于场景有没有避障数据，而不是当前有没有启用项：
        # 全部禁用表示用户明确要求隐藏风险区，只有空集合才代表需要兼容旧布局。
        obstacle_zones = _risk_zones_from_obstacles(obstacle_views, field)
        source_zones = obstacle_zones if obstacle_views else risk_zones_from_layout(layout)
        risk_zones = [_risk_zone_payload(zone, field) for zone in source_zones]
        # 真实障碍已经由精确轮廓的地形着色表达，不再叠加外接圆线网；
        # 只有旧场景缺少障碍库时才保留布局风险峰的兼容提示。
        risk_zone_lines = (
            []
            if obstacle_views
            else [line for zone in risk_zones for line in _risk_zone_line_payload(zone, field)]
        )
        risk_zone_buffers = (
            []
            if obstacle_views
            else [dash for zone in risk_zones for dash in _risk_zone_buffer_payload(zone, field)]
        )
        payload = {
            "ground": {
                "mode": "layout",
                "x": center_east,
                "y": -10.0,
                "z": -center_north,
                "width": width,
                "depth": depth,
                "height": 20.0,
            },
            "surface": {
                "mode": "layout",
                "x": center_east,
                "y": 0.0,
                "z": -center_north,
                "width": width,
                "depth": depth,
                "height": max([zone["height"] for zone in risk_zones] + [2600.0]),
                "layoutFile": layout_path,
                "resolution": resolution,
                # revision 用字符串传递:mtime_ns 超出 QML double 精度(2^53)会丢位。
                "revision": f"{revision}:{1 if field is not None else 0}",
                "fieldReady": field is not None,
                "effectiveSpan": effective_span,
            },
            "riskZones": risk_zones,
            "riskZoneLines": risk_zone_lines,
            "riskZoneBuffers": risk_zone_buffers,
        }
        # 末端防线:payload 里任何 NaN/Inf 都在这里拦下,而不是送进 QML JSON.parse。
        json.dumps(payload, ensure_ascii=False, allow_nan=False)
        return payload
    except (OSError, ValueError, json.JSONDecodeError, TypeError, KeyError, OverflowError) as error:
        # 显示层坏配置不允许打断 3D 刷新:回退 procedural 地形并输出可见诊断。
        logging.getLogger(__name__).warning("地形布局 %s 不可用,回退旧地形: %s", layout_path, error)
        return None


def _cached_layout(path: str) -> tuple[dict[str, object], int]:
    """缓存布局文件内容并返回版本号。注意：文件原地修改后 mtime 变化自动失效,
    避免同一路径重载时 extent/风险元数据与高度场混用不同版本。"""

    mtime_ns = Path(path).stat().st_mtime_ns
    return _cached_layout_versioned(path, mtime_ns), mtime_ns


@lru_cache(maxsize=4)
def _cached_layout_versioned(path: str, mtime_ns: int) -> dict[str, object]:
    """按(路径,修改时间)缓存布局 JSON。注意：仅由 _cached_layout 调用。"""

    return load_terrain_layout(path)


def _layout_resolution(layout: dict[str, object]) -> int:
    """解析布局网格分辨率。注意：SIM3D_LOW_SPEC=1 时采用低配档,供低端集显机使用。"""

    detail = layout.get("detail") if isinstance(layout.get("detail"), dict) else {}
    key = "low_spec_grid_resolution" if os.environ.get("SIM3D_LOW_SPEC") == "1" else "grid_resolution"
    value = int(detail.get(key, DEFAULT_TERRAIN_RESOLUTION))
    if value <= 0:
        raise ValueError(f"{key} 必须为正整数")
    return value


def _cached_terrain_field(path: str, resolution: int) -> TerrainField | None:
    """非阻塞获取共享高度场。注意：与 TerrainGeometry 共用 terrain_field 进程级缓存;
    未就绪返回 None(风险线先按声明高度出图),就绪后 fieldReady 翻转驱动 QML 替换。"""

    return peek_terrain_field(path, resolution=resolution)


def _risk_zone_payload(zone, field: TerrainField | None = None) -> dict[str, object]:  # noqa: ANN001
    """生成风险区显示数据。注意：中心坐标转换到 Quick3D，半径仍为米。"""

    center_height = _sample_field_height(field, zone.east_m, zone.north_m) if field is not None else zone.height_m
    # 中心高度只作为语义参考，实际罩面形状由每条风险线逐点采样决定。
    coord = enu_to_quick3d(zone.east_m, zone.north_m, center_height + 30.0)
    foot = enu_to_quick3d(zone.east_m, zone.north_m, center_height + 18.0)
    return {
        "id": zone.zone_id,
        "label": zone.label,
        "radius": zone.radius_m,
        "bufferRadius": zone.buffer_radius_m,
        "height": zone.height_m,
        "x": coord["x"],
        "y": coord["y"],
        "z": coord["z"],
        "footY": foot["y"],
    }


def _risk_zone_line_payload(zone: dict[str, object], field: TerrainField | None = None) -> list[dict[str, object]]:
    """生成风险区红色细线罩面。注意：线宽显著小于航线和尾迹。"""

    center_x = float(zone["x"])
    center_z = float(zone["z"])
    y = float(zone["y"])
    radius = float(zone["radius"])
    lines: list[dict[str, object]] = []
    for ratio in (-0.66, -0.33, 0.0, 0.33, 0.66):
        half = radius * math.sqrt(max(0.0, 1.0 - ratio * ratio))
        offset = radius * ratio
        for angle in (0.0, math.pi / 2.0):
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            p1 = [center_x + offset * -sin_a - half * cos_a, y, center_z + offset * cos_a - half * sin_a]
            p2 = [center_x + offset * -sin_a + half * cos_a, y, center_z + offset * cos_a + half * sin_a]
            path = _surface_line_path(field, p1, p2, 34.0) if field is not None else [p1, p2]
            lines.append(
                {
                    "color": "#ff5a45",
                    # 风险网格必须显著细于主航线(16m):视觉层级上任务航线优先。
                    "width": 7.0,
                    "pulse": False,
                    "pathValue": json.dumps(path, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
                }
            )
    return lines


def _risk_zone_buffer_payload(zone: dict[str, object], field: TerrainField | None = None) -> list[dict[str, object]]:
    """生成风险区山脚淡青虚线缓冲轮廓。注意：用短弧段组成虚线，减少 QML 逻辑。"""

    center_x = float(zone["x"])
    center_z = float(zone["z"])
    y = float(zone["footY"])
    radius = float(zone["bufferRadius"])
    dashes: list[dict[str, object]] = []
    dash_count = 24
    dash_angle = math.tau / dash_count * 0.55
    for index in range(dash_count):
        start = index * math.tau / dash_count
        points = []
        for sample in range(4):
            angle = start + dash_angle * sample / 3.0
            x = center_x + math.cos(angle) * radius
            z = center_z - math.sin(angle) * radius
            points.append([x, _sample_quick3d_height(field, x, z, 18.0) if field is not None else y, z])
        dashes.append(
            {
                "color": "#62d9e7",
                # 缓冲圈是最弱的提示层,比风险网格更细。
                "width": 5.0,
                "pathValue": json.dumps(points, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
            }
        )
    return dashes


def _surface_line_path(field: TerrainField, start: list[float], end: list[float], offset_m: float) -> list[list[float]]:
    """把水平线段披挂到地形表面。注意：输入输出均为 Quick3D 坐标。"""

    samples = 18
    points: list[list[float]] = []
    for index in range(samples):
        ratio = index / max(1, samples - 1)
        # 在水平线段上均匀取样，再把每个点投到高度场上。
        x = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
        z = float(start[2]) + (float(end[2]) - float(start[2])) * ratio
        points.append([x, _sample_quick3d_height(field, x, z, offset_m), z])
    return points


def _sample_quick3d_height(field: TerrainField | None, x_m: float, z_m: float, offset_m: float) -> float:
    """采样 Quick3D 坐标下的地形高度。注意：Quick3D z 轴为 north 取反。"""

    if field is None:
        return float(offset_m)
    return _sample_field_height(field, float(x_m), -float(z_m)) + offset_m


def _sample_field_height(field: TerrainField, east_m: float, north_m: float) -> float:
    """双线性采样高度场。注意：坐标越界时夹到最近网格边界。"""

    return _sample_height_values(field, field.heights_m, east_m, north_m)


def _sample_field_display_height(field: TerrainField, east_m: float, north_m: float) -> float:
    """双线性采样渲染高度。注意：没有显示增强网格时回退真实米制高度。"""

    heights = field.display_heights_m if field.display_heights_m is not None else field.heights_m
    return _sample_height_values(field, heights, east_m, north_m)


def _sample_height_values(
    field: TerrainField, heights: np.ndarray, east_m: float, north_m: float
) -> float:
    """在指定高度数组上做双线性采样。注意：数组必须与 field.resolution 同尺寸。"""

    min_e = field.center_east_m - field.width_m / 2.0
    min_n = field.center_north_m - field.depth_m / 2.0
    # 这是逐点热路径，使用标量夹取可避免为每个轮廓点进入两次 NumPy ufunc。
    grid_limit = float(field.resolution - 1)
    u = max(0.0, min(grid_limit, (float(east_m) - min_e) / max(field.width_m, 1.0) * grid_limit))
    v = max(0.0, min(grid_limit, (float(north_m) - min_n) / max(field.depth_m, 1.0) * grid_limit))
    # 高度场行列方向仍是 ENU north/east，Quick3D 的 z 翻转已在调用侧处理。
    col = int(math.floor(u))
    row = int(math.floor(v))
    col1 = min(col + 1, field.resolution - 1)
    row1 = min(row + 1, field.resolution - 1)
    tx = float(u - col)
    ty = float(v - row)
    h00 = float(heights[row, col])
    h10 = float(heights[row, col1])
    h01 = float(heights[row1, col])
    h11 = float(heights[row1, col1])
    return (h00 * (1.0 - tx) + h10 * tx) * (1.0 - ty) + (h01 * (1.0 - tx) + h11 * tx) * ty


def _camera_payload(bounds: dict[str, float], aircraft: list[dict[str, object]]) -> dict[str, float]:
    """生成初始相机参数。注意：QML 仍允许用户拖拽、缩放和重置。"""

    focus_x = _average([float(item["x"]) for item in aircraft], bounds["centerX"])
    focus_y = _average([float(item["y"]) for item in aircraft], max(320.0, bounds["maxY"] * 0.55))
    focus_z = _average([float(item["z"]) for item in aircraft], bounds["centerZ"])
    # 相机距离跟随 20km 基准跨度，避免初始视角只看到局部山包。
    span = max(bounds["spanX"], bounds["spanZ"], bounds["maxY"] - bounds["minY"], DEFAULT_TERRAIN_SPAN_M)
    return {
        "focusX": focus_x,
        "focusY": focus_y,
        "focusZ": focus_z,
        "distance": max(950.0, span * 1.15),
        "yaw": -38.0,
        "pitch": -34.0,
    }


def _layout_camera_payload(surface: dict[str, object]) -> dict[str, float]:
    """生成布局地形相机。注意：取景按内容跨度，不按 90km 渲染裙边。"""

    effective_span = float(surface.get("effectiveSpan", 32000.0))
    # 定稿布局是"平原+走廊"的稀疏构图，大俯角远机位会让暗平原占满画面；
    # 默认机位改为顺峡谷走廊的低角度近景，让两侧受光山坡填充画面下部（style_a 同族构图）。
    distance = max(8200.0, effective_span * 0.30)
    return {
        # 焦点略偏走廊东段，主光来自西南，北山链朝南受光大坡正对相机成为画面主体。
        "focusX": float(surface.get("x", 0.0)) + effective_span * 0.05,
        "focusY": 900.0,
        "focusZ": float(surface.get("z", 0.0)),
        "distance": distance,
        "yaw": -62.0,
        "pitch": -20.0,
    }


def _node_color(role: str, health: str) -> str:
    """返回节点显示颜色。注意：异常健康状态优先于角色颜色。"""

    if health != "normal":
        return "#f59e0b"
    if role.strip().lower() in {"leader", "rally_leader"}:
        return "#60a5fa"
    return "#c084fc"


def _heading_yaw_deg(vx_mps: float, vy_mps: float) -> float:
    """把 ENU 水平速度转换为 Quick3D 绕 Y 轴偏航角。注意：显示模型机头默认朝 +X。

    Quick3D 的 z 轴在坐标映射时已相对 ENU north 取反(见 enu_to_quick3d)，
    Y 轴偏航角的正方向与该取反抵消，此处不能再对 vy 取负，否则斜向航向会被镜像。
    """

    if math.hypot(vx_mps, vy_mps) < 1e-6:
        return 0.0
    return math.degrees(math.atan2(vy_mps, vx_mps))


def _evenly_sample(items: list, limit: int) -> list:
    """按上限均匀抽样。注意：保留首尾点，避免轨迹端点丢失。"""

    if len(items) <= limit:
        return list(items)
    if limit <= 1:
        return [items[-1]]
    last = len(items) - 1
    return [items[round(last * index / (limit - 1))] for index in range(limit)]


def _average(values: list[float], fallback: float) -> float:
    """求平均值。注意：空列表使用 fallback。"""

    return sum(values) / len(values) if values else fallback
