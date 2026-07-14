"""避障配置解析、预览航线转换与规划窗口辅助。注意：只提供 UI 辅助逻辑。"""

from __future__ import annotations

import math

from PySide6.QtWidgets import QDialog, QWidget

from src.runner.sim_control import (
    AvoidanceParams,
    GeoReference,
    ObstacleSpec,
    load_gui_config,
    preview_route_marker_points,
    route_inputs_to_config,
    route_to_polyline,
)
from src.ui.gui.view_models import ObstacleView

def parse_avoidance_config(path: str) -> tuple[list[ObstacleView], float]:
    """兼容入口：通过 runner 应用层读取障碍。注意：正式窗口复用 adapter 已缓存结果。"""

    data = load_gui_config(path)
    return [obstacle_spec_to_view(obstacle) for obstacle in data.obstacles], data.obstacle_clearance_m


class AvoidanceWindow(QDialog):
    """避障规划子窗口。注意：控件由 MainWindow 填充，本类只固化窗口元数据。"""

    param_order = [
        # 1. 先锁定飞机物理转弯能力。
        "turn_radius_m",
        # 2. 再确认圆弧之间保留的直线余度。
        "leg_length_margin_m",
        # 3. 安全边界优先于搜索质量参数。
        "clearance_m",
        # 4. 栅格精度决定 A* 离散程度。
        "grid.resolution_m",
        # 5. 搜索包围盒大小影响能否绕开障碍。
        "grid.margin_m",
        # 6. 去冗余参数只在安全约束稳定后调整。
        "simplify_clearance_m",
        # 7. 方向切换惩罚用于减少碎段。
        "turn_switch_penalty_m",
        # 8. 航迹角惩罚最后再微调硬拐。
        "turn_angle_weight_m",
    ]

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化避障规划窗口。注意：非模态窗口，便于对照主画布预览。"""
        super().__init__(parent)
        self.setWindowTitle("避障规划")
        self.setModal(False)
        self.setMinimumSize(820, 560)


def parse_avoidance_params(path: str) -> AvoidanceParams | None:
    """兼容入口：通过 runner 应用层读取规划参数。"""

    return load_gui_config(path).avoidance_params


def _geo_origin_from_config_for_ui(path: str) -> GeoReference | None:
    """兼容入口：返回应用层地理原点。注意：正式窗口使用 adapter 缓存。"""

    return load_gui_config(path).geo_reference


def obstacle_view_to_spec(view: ObstacleView) -> ObstacleSpec:
    """把可变显示障碍复制为应用层规划输入。"""

    return ObstacleSpec(
        obstacle_id=view.obstacle_id,
        kind=view.kind,
        enabled=view.enabled,
        center_x=view.center_x,
        center_y=view.center_y,
        radius=view.radius,
        min_x=view.min_x,
        min_y=view.min_y,
        max_x=view.max_x,
        max_y=view.max_y,
        vertices=tuple(view.vertices),
    )


def obstacle_spec_to_view(spec: ObstacleSpec) -> ObstacleView:
    """把应用层障碍复制为 GUI 可勾选对象。"""

    return ObstacleView(
        obstacle_id=spec.obstacle_id,
        kind=spec.kind,
        enabled=spec.enabled,
        center_x=spec.center_x,
        center_y=spec.center_y,
        radius=spec.radius,
        min_x=spec.min_x,
        min_y=spec.min_y,
        max_x=spec.max_x,
        max_y=spec.max_y,
        vertices=list(spec.vertices),
    )


def _inflated_polygon_vertices(vertices: list[tuple[float, float]], inflate: float) -> list[tuple[float, float]]:
    """返回用于 GUI 显示的多边形外扩顶点。注意：面向旋转矩形/凸多边形显示近似。"""
    if inflate <= 0.0 or len(vertices) < 3:
        return list(vertices)
    signed_area = 0.0
    # 用有向面积判断顶点绕序，从而确定每条边的外法线方向。
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        signed_area += x0 * y1 - y0 * x1
    if abs(signed_area) <= 1e-9:
        return _inflate_polygon_from_centroid(vertices, inflate)

    edge_lines: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return _inflate_polygon_from_centroid(vertices, inflate)
        # Qt 画布使用 east/north 世界坐标，外扩只影响显示，不改变后端 inside(clearance) 语义。
        if signed_area > 0.0:
            normal = (dy / length, -dx / length)
        else:
            normal = (-dy / length, dx / length)
        # 每条边沿外法线平移 inflate，再取相邻平移边交点作为新的角点。
        point = (start[0] + normal[0] * inflate, start[1] + normal[1] * inflate)
        edge_lines.append((point, (dx, dy)))

    inflated: list[tuple[float, float]] = []
    for index, _ in enumerate(vertices):
        previous_point, previous_dir = edge_lines[index - 1]
        current_point, current_dir = edge_lines[index]
        # 相邻外移边的交点就是凸多边形的外扩角点；平行退化时改用径向兜底。
        intersection = _line_intersection(previous_point, previous_dir, current_point, current_dir)
        if intersection is None:
            return _inflate_polygon_from_centroid(vertices, inflate)
        inflated.append(intersection)
    return inflated


def _line_intersection(
    point_a: tuple[float, float],
    dir_a: tuple[float, float],
    point_b: tuple[float, float],
    dir_b: tuple[float, float],
) -> tuple[float, float] | None:
    """求两条参数直线交点。注意：平行或近似平行时返回 None。"""
    # 二维叉积接近 0 表示两条外移边平行，无法稳定求 miter 角点。
    cross = dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0]
    if abs(cross) <= 1e-9:
        return None
    # 解 point_a + dir_a * t = point_b + dir_b * u，只需要 t 即可还原交点。
    delta = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    t = (delta[0] * dir_b[1] - delta[1] * dir_b[0]) / cross
    return point_a[0] + dir_a[0] * t, point_a[1] + dir_a[1] * t


def _inflate_polygon_from_centroid(vertices: list[tuple[float, float]], inflate: float) -> list[tuple[float, float]]:
    """退化多边形的显示兜底：各顶点沿几何中心径向外推。"""
    center_x = sum(point[0] for point in vertices) / len(vertices)
    center_y = sum(point[1] for point in vertices) / len(vertices)
    inflated: list[tuple[float, float]] = []
    for east, north in vertices:
        dx = east - center_x
        dy = north - center_y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            inflated.append((east, north))
        else:
            # 兜底路径只保证外扩圈不和本体重合，不承诺精确等距 offset。
            scale = (length + inflate) / length
            inflated.append((center_x + dx * scale, center_y + dy * scale))
    return inflated


def _rounded_inflated_polygon_points(
    vertices: list[tuple[float, float]], inflate: float, corner_segments: int = 6
) -> list[tuple[float, float]]:
    """返回用于 GUI 显示的多边形圆角外扩折线点，与后端 inside() 的圆角膨胀语义一致。

    每条边沿外法线平移 inflate，相邻边在凸顶点用半径 = inflate 的圆弧衔接，凹顶点则连接
    两条偏移边的交点。这样凸角是圆角（等价 Minkowski 和），而不像
    _inflated_polygon_vertices 的 miter 尖角向外凸出。inflate<=0 或退化时回退到原始顶点/
    径向兜底。
    """
    if inflate <= 0.0 or len(vertices) < 3:
        return list(vertices)
    signed_area = 0.0
    # 有向面积定绕序，进而确定每条边的外法线方向（与 _inflated_polygon_vertices 保持一致）。
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        signed_area += x0 * y1 - y0 * x1
    if abs(signed_area) <= 1e-9:
        return _inflate_polygon_from_centroid(vertices, inflate)

    normals: list[tuple[float, float]] = []
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return _inflate_polygon_from_centroid(vertices, inflate)
        # 正绕序取右手法线，负绕序取反向，二者都指向多边形外侧。
        if signed_area > 0.0:
            normals.append((dy / length, -dx / length))
        else:
            normals.append((-dy / length, dx / length))

    count = len(vertices)
    points: list[tuple[float, float]] = []
    for index, vertex in enumerate(vertices):
        # 每个顶点同时查看入边和出边，避免把单边法线变化一律误判成凸角圆弧。
        previous_index = (index - 1) % count
        previous_vertex = vertices[previous_index]
        next_vertex = vertices[(index + 1) % count]
        previous_normal = normals[previous_index]
        next_normal = normals[index]
        incoming = (vertex[0] - previous_vertex[0], vertex[1] - previous_vertex[1])
        outgoing = (next_vertex[0] - vertex[0], next_vertex[1] - vertex[1])
        turn_cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]

        if turn_cross * signed_area < -1e-9:
            # 凹角不属于 Minkowski 外扩的圆角部分。两条偏移边应在交点处形成凹折角；
            # 若仍按绕序补圆弧，会走过约 270°，在界面上形成无业务含义的虚线圆圈。
            previous_offset = (
                vertex[0] + previous_normal[0] * inflate,
                vertex[1] + previous_normal[1] * inflate,
            )
            next_offset = (
                vertex[0] + next_normal[0] * inflate,
                vertex[1] + next_normal[1] * inflate,
            )
            # 直线交点参数沿入边方向求解；凹角已保证两方向不平行，分母不会退化为零。
            denominator = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            delta_x = next_offset[0] - previous_offset[0]
            delta_y = next_offset[1] - previous_offset[1]
            # 只保留一个交点，后续多边形描边会自动连接相邻偏移直段。
            along_incoming = (delta_x * outgoing[1] - delta_y * outgoing[0]) / denominator
            points.append(
                (
                    previous_offset[0] + incoming[0] * along_incoming,
                    previous_offset[1] + incoming[1] * along_incoming,
                )
            )
            continue

        angle_from = math.atan2(previous_normal[1], previous_normal[0])
        sweep = math.atan2(next_normal[1], next_normal[0]) - angle_from
        # 凸角沿多边形绕序补外侧短圆弧；共线顶点自然退化为一个点。
        if signed_area > 0.0:
            while sweep < 0.0:
                sweep += 2.0 * math.pi
        else:
            while sweep > 0.0:
                sweep -= 2.0 * math.pi
        steps = max(1, int(corner_segments * abs(sweep) / (math.pi / 2.0)))
        for step in range(steps + 1):
            angle = angle_from + sweep * (step / steps)
            points.append((vertex[0] + math.cos(angle) * inflate, vertex[1] + math.sin(angle) * inflate))
    return points
