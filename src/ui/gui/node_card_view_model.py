"""飞机信息卡片 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field


@dataclass(frozen=True)
class ScreenPoint:
    """一架飞机的屏幕坐标与显示优先级。注意：坐标单位固定为视口像素。"""

    # node_id 直接沿用快照标识，避免视图层另建索引。
    node_id: str
    # x、y 已经过世界到视口变换，可直接参与拾取与遮挡计算。
    x: float
    y: float
    # 长机标记只影响自动卡片的贪心占位顺序。
    is_leader: bool = False


def pick_node(
    click_x: float,
    click_y: float,
    points: Sequence[ScreenPoint],
    radius_px: float = 20.0,
) -> str | None:
    """返回点击半径内距离最近的节点。注意：距离与半径均按屏幕像素计算。"""

    # 用距离平方比较，避免为每个候选点开平方；负半径按无命中处理。
    if radius_px < 0.0:
        return None
    radius_squared = radius_px * radius_px
    # 初值落在命中边界，使恰好等于半径的候选点仍可被拾取。
    nearest_node_id: str | None = None
    nearest_distance_squared = radius_squared
    # 单次线性扫描足以覆盖当前小规模编队，无需维护额外空间索引。
    for point in points:
        distance_squared = (point.x - click_x) ** 2 + (point.y - click_y) ** 2
        # 同距时保留输入中的第一架飞机，保证重叠节点的拾取结果稳定。
        if distance_squared <= radius_squared and (
            nearest_node_id is None or distance_squared < nearest_distance_squared
        ):
            nearest_node_id = point.node_id
            nearest_distance_squared = distance_squared
    return nearest_node_id


@dataclass(frozen=True)
class CardRect:
    """屏幕空间矩形。注意：宽高与坐标均使用固定视口像素。"""

    # x、y 表示屏幕坐标系中的左上角，y 正方向朝下。
    x: float
    y: float
    # w、h 保存固定像素尺寸，不随世界坐标缩放变化。
    w: float
    h: float

    def overlaps(self, other: CardRect, margin: float = 0.0) -> bool:
        """判断本矩形外扩 margin 后是否与另一矩形重叠。注意：仅边界接触不算重叠。"""

        # 负边距没有遮挡语义，统一按零处理，避免意外缩小候选区域。
        expanded = max(0.0, margin)
        return (
            self.x - expanded < other.x + other.w
            and self.x + self.w + expanded > other.x
            and self.y - expanded < other.y + other.h
            and self.y + self.h + expanded > other.y
        )


def _anchor_candidates(
    point: ScreenPoint,
    width: float,
    height: float,
    gap_x: float,
    gap_y: float,
) -> tuple[CardRect, CardRect, CardRect, CardRect]:
    """生成右上/左上/右下/左下四个候选卡片矩形。注意：四者都与机体保持固定间距，
    绝不与机体自身重叠，方向选择只影响卡片朝哪一侧展开。
    """

    return (
        CardRect(point.x + gap_x, point.y - gap_y - height, width, height),  # 右上（默认优先）
        CardRect(point.x - gap_x - width, point.y - gap_y - height, width, height),  # 左上
        CardRect(point.x + gap_x, point.y + gap_y, width, height),  # 右下
        CardRect(point.x - gap_x - width, point.y + gap_y, width, height),  # 左下
    )


def card_rect_for(
    point: ScreenPoint,
    width: float,
    height: float,
    gap_x: float = 16.0,
    gap_y: float = 14.0,
    viewport_w: float | None = None,
    viewport_h: float | None = None,
) -> CardRect:
    """计算飞机信息卡片矩形。注意：卡片与机体的间距固定不变，绝不做覆盖式夹紧；
    提供视口尺寸时按右上→左上→右下→左下顺序挑第一个完整落入视口的锚点，
    四者都放不下时退化为与视口重叠面积最大的锚点（卡片可能被部分裁切，但仍不压住机体）。
    """

    candidates = _anchor_candidates(point, width, height, gap_x, gap_y)
    if viewport_w is None or viewport_h is None:
        return candidates[0]

    def fits_fully(rect: CardRect) -> bool:
        return 0.0 <= rect.x and 0.0 <= rect.y and rect.x + rect.w <= viewport_w and rect.y + rect.h <= viewport_h

    for candidate in candidates:
        if fits_fully(candidate):
            return candidate

    def overlap_area(rect: CardRect) -> float:
        overlap_w = max(0.0, min(rect.x + rect.w, viewport_w) - max(rect.x, 0.0))
        overlap_h = max(0.0, min(rect.y + rect.h, viewport_h) - max(rect.y, 0.0))
        return overlap_w * overlap_h

    return max(candidates, key=overlap_area)


def is_point_on_screen(x: float, y: float, viewport_w: float, viewport_h: float) -> bool:
    """判断飞机中心是否仍落在视口范围内。注意：完全离屏的节点不应生成任何卡片。"""

    return 0.0 <= x <= viewport_w and 0.0 <= y <= viewport_h


@dataclass
class CardBoardState:
    """维护卡片人工覆盖与自动遮挡记忆。注意：不读写任何 GUI 控件。"""

    # True 表示强制显示，False 表示强制退化；缺失键表示自动模式。
    overrides: dict[str, bool] = field(default_factory=dict)
    # 仅记录自动模式最近一次遮挡结果，为恢复滞回提供前态。
    visible: dict[str, bool] = field(default_factory=dict)
    # 记录节点中心最近一次是否落在视口内；离屏节点无论 override 结果如何都不生成卡片。
    on_screen: dict[str, bool] = field(default_factory=dict)

    def handle_click(
        self,
        click_x: float,
        click_y: float,
        points: Sequence[ScreenPoint],
    ) -> str | None:
        """处理一次飞机单击并返回节点 ID。注意：空白点击不清除任何人工覆盖。"""

        picked_node_id = pick_node(click_x, click_y, points)
        if picked_node_id is None:
            return None
        # 已有覆盖时第二次点击直接回到 auto，并保留此前自动遮挡记忆。
        if picked_node_id in self.overrides:
            del self.overrides[picked_node_id]
            return picked_node_id
        # 首次点击反转当前实际显示结果，形成 pinned_show 或 pinned_hide。
        self.overrides[picked_node_id] = not self.is_card_shown(picked_node_id)
        return picked_node_id

    def sync_nodes(self, active_node_ids: set[str]) -> bool:
        """清理退场节点的覆盖与可见性记录。注意：在场节点的滞回状态保持不动。"""

        changed = False
        # 分别清理三个字典，避免退场节点再次出现时继承陈旧人工选择或离屏记忆。
        for records in (self.overrides, self.visible, self.on_screen):
            departed = records.keys() - active_node_ids
            if departed:
                changed = True
            for node_id in departed:
                del records[node_id]
        return changed

    def update_visibility(
        self,
        points: Sequence[ScreenPoint],
        card_w: float,
        card_h: float,
        gap_x: float = 16.0,
        gap_y: float = 14.0,
        icon_size: float = 48.0,
        recovery_margin: float = 10.0,
        viewport_w: float | None = None,
        viewport_h: float | None = None,
    ) -> bool:
        """按优先级与滞回更新自动卡片可见性。注意：返回是否有实际状态变化。

        icon_size 默认 48px：机体剪影任意朝向下的外接圆半径约 21px，
        加画笔半宽与抗锯齿余量后取整为方框边长，确保旋转机头也不会被误判为不重叠。
        提供 viewport_w/viewport_h 时，完全离屏的节点（中心落在视口外）不生成卡片、
        不参与遮挡占位，避免画面上出现没有属主飞机的孤立卡片。
        """

        # 视口未知时一律视为在屏，兼容旧调用方；仅在提供尺寸时才做离屏判定。
        def on_screen(point: ScreenPoint) -> bool:
            if viewport_w is None or viewport_h is None:
                return True
            return is_point_on_screen(point.x, point.y, viewport_w, viewport_h)

        changed = False
        for point in points:
            now_on_screen = on_screen(point)
            if self.on_screen.get(point.node_id, True) != now_on_screen:
                changed = True
            self.on_screen[point.node_id] = now_on_screen

        # 全部图标先形成不可覆盖区域；每张卡片只排除自己的图标方框。
        half_icon = icon_size / 2.0
        icon_rects = {
            point.node_id: CardRect(point.x - half_icon, point.y - half_icon, icon_size, icon_size)
            for point in points
        }
        # pinned_show 无条件显示并先占位；离屏节点即使被强制显示也不生成卡片矩形。
        occupied_cards = [
            card_rect_for(point, card_w, card_h, gap_x, gap_y, viewport_w, viewport_h)
            for point in points
            if self.overrides.get(point.node_id) is True and self.on_screen.get(point.node_id, True)
        ]
        # 自动节点按长机优先、同类 node_id 字典序依次尝试占位。
        auto_points = sorted(
            (point for point in points if point.node_id not in self.overrides),
            key=lambda point: (not point.is_leader, point.node_id),
        )
        # 变化标记只反映实际可见性切换，供画布避免无意义重绘。
        for point in auto_points:
            previous = self.visible.get(point.node_id, True)
            if not self.on_screen.get(point.node_id, True):
                # 离屏节点没有讨论遮挡的意义，直接退化为不显示。
                next_visible = False
            else:
                # 候选矩形与 GUI 绘制复用完全相同的锚点选择与视口适配规则。
                candidate = card_rect_for(point, card_w, card_h, gap_x, gap_y, viewport_w, viewport_h)
                # 已隐藏节点用 10px 外扩检测净空；可见节点重叠后立即隐藏。
                margin = 0.0 if previous else recovery_margin
                # 已占位卡片仅来自 pinned_show 或本轮已接受的高优先级 auto 节点。
                blocked_by_card = any(candidate.overlaps(rect, margin) for rect in occupied_cards)
                # 图标方框覆盖所有其他节点，属主自己的图标按需求明确排除。
                blocked_by_icon = any(
                    node_id != point.node_id and candidate.overlaps(rect, margin)
                    for node_id, rect in icon_rects.items()
                )
                # 任一类屏幕空间阻挡都使 auto 卡片退化为原有 ID 标签。
                next_visible = not (blocked_by_card or blocked_by_icon)
                # 只有本轮实际显示的 auto 卡片才参与后续贪心占位。
                if next_visible:
                    occupied_cards.append(candidate)
            if next_visible != previous:
                changed = True
            self.visible[point.node_id] = next_visible
        return changed

    def is_card_shown(self, node_id: str) -> bool:
        """返回节点卡片的实际显示结果。注意：离屏优先于人工覆盖，人工覆盖优先于自动遮挡记忆。"""

        # 属主飞机完全离屏时任何覆盖都不应凭空产生悬浮卡片。
        if not self.on_screen.get(node_id, True):
            return False
        if node_id in self.overrides:
            return self.overrides[node_id]
        # 新节点尚未完成首次检测时默认显示，满足全员挂卡的初始语义。
        return self.visible.get(node_id, True)
