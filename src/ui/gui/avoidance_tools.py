"""避障配置解析、预览航线转换与规划窗口辅助。注意：只提供 UI 辅助逻辑。"""

from __future__ import annotations

import math
from dataclasses import dataclass

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


@dataclass(frozen=True)
class AvoidanceParamSpec:
    """单个避障参数的控件与说明规格。"""

    # key 对应配置路径，并共同生成设计文档要求的显示顺序。
    key: str
    # caption 和 widget_attr 分别约束标签文案与 MainWindow 上的稳定属性名。
    caption: str
    widget_attr: str
    # 步长、后缀和上限共同表达界面输入单位，不参与算法计算。
    step: float
    # 提示文案包含作用、影响和建议，标签与输入框复用同一份文本。
    tooltip: str
    suffix: str = " m"
    maximum: float = 100000.0
    # 只有拉直安全间距需要专用回调，其余参数共享预览失效逻辑。
    on_change_method: str | None = None


# 规格顺序就是避障设计文档中的调参顺序，不能在构造窗口时再次排序。
AVOIDANCE_PARAM_SPECS = (
    # 一、先锁定飞机物理转弯能力。
    AvoidanceParamSpec(
        "turn_radius_m",
        "转弯半径 R",
        "turn_radius_spin",
        10.0,
        "作用：约束拐点圆弧的最小转弯半径。\n影响：越大转弯越平缓，但更容易腿太短或圆弧触障。\n"
        "建议：按飞机能力给定；无约束时先取 200~300 m。",
    ),
    # 二、再确认圆弧之间保留的直线余度。
    AvoidanceParamSpec(
        "leg_length_margin_m",
        "航段余度 L",
        "leg_margin_spin",
        10.0,
        "作用：要求相邻圆弧之间保留额外直线余度。\n影响：越大越保守，但更容易触发腿长不足。\n"
        "建议：R 确定后再调，先试 0.2R~0.5R。",
    ),
    # 三、安全边界优先于搜索质量参数。
    AvoidanceParamSpec(
        "clearance_m",
        "安全间距",
        "clearance_spin",
        10.0,
        "作用：A* 搜索时对障碍做外扩，形成安全边界。\n影响：越大越安全但绕行更远，窄通道更可能无路。\n"
        "建议：优先按业务安全距离，常用 80~150 m。",
    ),
    # 四、栅格精度决定 A* 离散程度与计算量。
    AvoidanceParamSpec(
        "grid.resolution_m",
        "栅格间距",
        "resolution_spin",
        5.0,
        "作用：决定 A* 栅格离散精度。\n影响：越小路径越细但更慢；越大更快但更粗。\n"
        "建议：先取小于等于 R/10，例如 R=300 m 时 20~30 m。",
    ),
    # 五、搜索包围盒大小决定障碍外是否留有可达空间。
    AvoidanceParamSpec(
        "grid.margin_m",
        "搜索边界余量",
        "margin_spin",
        50.0,
        "作用：扩展起终点和障碍外侧的搜索包围盒。\n影响：越大绕行空间越足但网格规模增大。\n"
        "建议：先取安全间距 + 转弯半径，或直接取 300 m。",
    ),
    # 六、去冗余安全距可独立调；旧配置未显式给值时跟随安全间距。
    AvoidanceParamSpec(
        "simplify_clearance_m",
        "拉直安全间距",
        "simplify_clearance_spin",
        10.0,
        "作用：A* 后视线去冗余使用的障碍外扩距离。\n影响：越小越容易拉直、航段更少，但更贴近障碍。\n"
        "建议：初始等于安全间距；减少碎段时试 0.5 倍安全间距。",
        on_change_method="_on_simplify_clearance_changed",
    ),
    # 七、方向切换惩罚用于减少栅格路径碎段。
    AvoidanceParamSpec(
        "turn_switch_penalty_m",
        "转向切换惩罚",
        "turn_switch_penalty_spin",
        1.0,
        "作用：惩罚 A* 中每次 8 邻域方向切换。\n影响：越大越少频繁换向，但可能绕远或贴边。\n"
        "建议：减少碎段时从 1 倍栅格间距试起。",
        suffix=" m/次",
    ),
    # 八、航迹角惩罚最后微调硬拐，后缀明确其每 45 度的单位。
    AvoidanceParamSpec(
        "turn_angle_weight_m",
        "航迹角惩罚",
        "turn_angle_weight_spin",
        1.0,
        "作用：按每 45° 航迹角变化增加线性代价。\n影响：可减少硬拐；过大时会和最短路目标拉扯。\n"
        "建议：最后再调，先试转向切换惩罚的 0.25~0.5 倍。",
        suffix=" m/45°",
    ),
)


def parse_avoidance_config(path: str) -> tuple[list[ObstacleView], float]:
    """兼容入口：通过 runner 应用层读取障碍。注意：正式窗口复用 adapter 已缓存结果。"""

    data = load_gui_config(path)
    return [obstacle_spec_to_view(obstacle) for obstacle in data.obstacles], data.obstacle_clearance_m


class AvoidanceWindow(QDialog):
    """避障规划子窗口。注意：控件由 MainWindow 填充，本类只固化窗口元数据。"""

    param_order = [spec.key for spec in AVOIDANCE_PARAM_SPECS]

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
    两条偏移边的交点。这样凸角保持与后端一致的圆角（等价 Minkowski 和）。
    inflate<=0 或退化时回退到原始顶点/径向兜底。
    """
    if inflate <= 0.0 or len(vertices) < 3:
        return list(vertices)
    signed_area = 0.0
    # 有向面积定绕序，进而确定每条边的外法线方向。
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
