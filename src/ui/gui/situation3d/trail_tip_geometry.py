"""3D 尾迹活动末段几何。注意：本类只维护固定六顶点小网格，不持有历史队列。"""

from __future__ import annotations

import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

_FLOAT_SIZE = 4
_VERTEX_COMPONENTS = 12
_VERTEX_STRIDE = _VERTEX_COMPONENTS * _FLOAT_SIZE
_VERTEX_COUNT = 6
_INDEX_COUNT = 9
_TRAIL_ALPHA = 0.72
_EPSILON = 1e-6

Point3D = tuple[float, float, float]

# 固定小网格布局：
# 1. 顶点 0..3 是活动段起点左右、终点左右，索引 0..5 组成两个主体三角形。
# 2. 顶点 4 是活动段起点中心，顶点 5 是历史末段外侧端点，索引 6..8 填补转角楔形。
# 3. 补间起点与末端重合时九个索引全部退化，不会在飞机前方提前显示真实目标。
# 4. 历史网格停在 tipStart；本类只接收 tipPrevious、tipStart 和飞机共同展示位置。
# 5. 每次补间只上传 288 字节顶点和 36 字节索引，成本与历史点数及队列容量无关。
# 6. 转角统一采用 bevel 填充；它无需回写历史末段端点，因此两张几何资源完全解耦。
# 7. previousPosition 和 startPosition 只随真实快照更新，不能绑定 60Hz 展示时钟。
# 8. endPosition 是唯一逐展示帧变化的属性，并且永远不回写逻辑 TrailBuffer。
# 9. 三个位置属性都拒绝 NaN 和无穷值，防止一个坏点污染场景包围盒。
# 10. 宽度上下限与历史几何一致，切换相机距离时两段边缘不会产生台阶。
# 11. 主体索引始终固定为 0、2、1 和 1、2、3，不存在环形槽复用或物理基址换算。
# 12. 接头索引只引用本地六个顶点，禁止跨 QQuick3DGeometry 引用历史缓冲。
# 13. 前后方向共线时无需接头三角形，三个零索引形成不可见退化面。
# 14. 历史段或活动段水平投影退化时也不构造接头，避免从兜底法向猜测转向。
# 15. 活动段仅高度变化时仍使用固定侧向绘制主体，保证爬升尾迹不会完全消失。
# 16. 转向符号与 TrailRibbonGeometry 保持一致，正转连接 left，反转连接 right。
# 17. 材质关闭背面剔除，因此接头三角不依赖视角或转向改变绕序。
# 18. 顶点色 alpha 固定为历史最新端的 0.72，使两张几何在接缝处亮度一致。
# 19. 包围盒只围绕六个本地顶点扩张，不扫描历史点，也不继承历史保守边界。
# 20. 属性赋值顺序可能由 QML 决定；每个中间状态都必须产生合法定长缓冲。
# 21. 构造期即分配定长退化缓冲，渲染线程不会观察到未声明或变长的资源。
# 22. 本类不保存 generation、序号或容量；这些状态只能由历史增量几何消费。


class TrailTipGeometry(QQuick3DGeometry):
    """绘制尾迹活动末段。注意：所有属性变化都只重建固定大小的本地缓冲。"""

    previousPositionChanged = Signal()
    startPositionChanged = Signal()
    endPositionChanged = Signal()
    widthValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化退化小网格，确保 QML 首次绑定前后缓冲尺寸始终不变。"""

        super().__init__(parent)
        self._previous_position = QVector3D()
        self._start_position = QVector3D()
        self._end_position = QVector3D()
        self._width_value = 44.0
        self._configure_geometry()
        self._rebuild()

    @Property(QVector3D, notify=previousPositionChanged)
    def previousPosition(self) -> QVector3D:
        """返回历史末段起点，用于确定活动接头外侧。"""

        return QVector3D(self._previous_position)

    @previousPosition.setter
    def previousPosition(self, value: QVector3D) -> None:
        """更新历史末段起点。注意：非法或非有限坐标保持上一合法值。"""

        normalized = self._normalized_vector(value)
        if normalized is None or normalized == self._previous_position:
            return
        self._previous_position = normalized
        self._rebuild()
        self.previousPositionChanged.emit()

    @Property(QVector3D, notify=startPositionChanged)
    def startPosition(self) -> QVector3D:
        """返回活动末段的稳定起点，即历史大网格的真实末点。"""

        return QVector3D(self._start_position)

    @startPosition.setter
    def startPosition(self, value: QVector3D) -> None:
        """更新活动末段起点。注意：该点只在真实快照到达时变化。"""

        normalized = self._normalized_vector(value)
        if normalized is None or normalized == self._start_position:
            return
        self._start_position = normalized
        self._rebuild()
        self.startPositionChanged.emit()

    @Property(QVector3D, notify=endPositionChanged)
    def endPosition(self) -> QVector3D:
        """返回活动末段的展示末端，该位置与飞机使用同一补间进度。"""

        return QVector3D(self._end_position)

    @endPosition.setter
    def endPosition(self, value: QVector3D) -> None:
        """更新展示末端。注意：60Hz 热路径只重建本类固定大小缓冲。"""

        normalized = self._normalized_vector(value)
        if normalized is None or normalized == self._end_position:
            return
        self._end_position = normalized
        self._rebuild()
        self.endPositionChanged.emit()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回活动末段宽度，单位为显示层米。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """更新活动末段宽度。注意：范围与历史 TrailRibbonGeometry 保持一致。"""

        try:
            normalized = float(value)
        except (TypeError, ValueError):
            return
        if not math.isfinite(normalized):
            return
        normalized = max(4.0, min(360.0, normalized))
        if math.isclose(normalized, self._width_value, rel_tol=1e-6):
            return
        self._width_value = normalized
        self._rebuild()
        self.widthValueChanged.emit()

    @staticmethod
    def _normalized_vector(value: QVector3D) -> QVector3D | None:
        """把 QML 坐标复制为有限 QVector3D；不合法值返回 None。"""

        try:
            normalized = QVector3D(value)
        except (TypeError, ValueError):
            return None
        components = (normalized.x(), normalized.y(), normalized.z())
        return normalized if all(math.isfinite(component) for component in components) else None

    def _configure_geometry(self) -> None:
        """声明与历史线带相同的顶点布局，使两者可以复用同一种材质。"""

        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_VERTEX_STRIDE)
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.PositionSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.NormalSemantic,
            3 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic,
            6 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.ColorSemantic,
            8 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )

    def _rebuild(self) -> None:
        """重建固定六顶点网格。注意：缓冲长度不随活动段长度或历史容量变化。"""

        previous = self._point(self._previous_position)
        start = self._point(self._start_position)
        end = self._point(self._end_position)
        positions = [start] * _VERTEX_COUNT
        indices = [0] * _INDEX_COUNT
        if math.dist(start, end) > _EPSILON:
            current_normal = self._segment_normal(start, end)
            start_left, start_right = self._edge_positions(start, current_normal)
            end_left, end_right = self._edge_positions(end, current_normal)
            positions[:4] = [start_left, start_right, end_left, end_right]
            positions[4] = start
            indices[:6] = [0, 2, 1, 1, 2, 3]
            join = self._join_layout(previous, start, end)
            if join is not None:
                previous_outer, current_outer_index = join
                positions[5] = previous_outer
                indices[6:] = [5, current_outer_index, 4]
        # 两个接头辅助顶点不采样纹理，仍写合法 UV 以保持统一 48 字节布局。
        texture_coordinates = (
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 0.5),
            (0.0, 0.5),
        )
        vertices = b"".join(
            self._vertex_record(position, *texture_coordinates[index])
            for index, position in enumerate(positions)
        )
        self.setVertexData(QByteArray(vertices))
        self.setIndexData(QByteArray(struct.pack("<IIIIIIIII", *indices)))
        margin = self._width_value
        self.setBounds(
            QVector3D(
                min(point[0] for point in positions) - margin,
                min(point[1] for point in positions) - 2.0,
                min(point[2] for point in positions) - margin,
            ),
            QVector3D(
                max(point[0] for point in positions) + margin,
                max(point[1] for point in positions) + 2.0,
                max(point[2] for point in positions) + margin,
            ),
        )
        self.update()

    def _join_layout(
        self,
        previous: Point3D,
        start: Point3D,
        end: Point3D,
    ) -> tuple[Point3D, int] | None:
        """返回需要填补的历史外侧端点和活动外侧顶点索引；直线或退化段不填补。"""

        previous_delta = (start[0] - previous[0], start[2] - previous[2])
        current_delta = (end[0] - start[0], end[2] - start[2])
        previous_length = math.hypot(*previous_delta)
        current_length = math.hypot(*current_delta)
        if previous_length <= _EPSILON or current_length <= _EPSILON:
            return None
        previous_direction = (previous_delta[0] / previous_length, previous_delta[1] / previous_length)
        current_direction = (current_delta[0] / current_length, current_delta[1] / current_length)
        turn = previous_direction[0] * current_direction[1] - previous_direction[1] * current_direction[0]
        if abs(turn) <= _EPSILON:
            return None
        previous_normal = (-previous_direction[1], previous_direction[0])
        previous_left, previous_right = self._edge_positions(start, previous_normal)
        # 与历史线带的 bevel 约定一致：正转连接 left，反转连接 right。
        if turn >= 0.0:
            return previous_left, 0
        return previous_right, 1

    def _edge_positions(
        self,
        point: Point3D,
        normal: tuple[float, float],
    ) -> tuple[Point3D, Point3D]:
        """按当前半宽返回中心点两侧边缘。"""

        half_width = self._width_value / 2.0
        offset_x = normal[0] * half_width
        offset_z = normal[1] * half_width
        return (
            (point[0] - offset_x, point[1], point[2] - offset_z),
            (point[0] + offset_x, point[1], point[2] + offset_z),
        )

    @staticmethod
    def _segment_normal(start: Point3D, end: Point3D) -> tuple[float, float]:
        """返回活动段在 XZ 平面的单位侧向；纯竖直段使用固定 +X 侧向。"""

        delta_x = end[0] - start[0]
        delta_z = end[2] - start[2]
        length = math.hypot(delta_x, delta_z)
        if length <= _EPSILON:
            return 1.0, 0.0
        return -delta_z / length, delta_x / length

    @staticmethod
    def _point(value: QVector3D) -> Point3D:
        """把 QVector3D 转成不可变三元组，便于几何计算。"""

        return value.x(), value.y(), value.z()

    @staticmethod
    def _vertex_record(position: Point3D, u_coord: float, v_coord: float) -> bytes:
        """编码一个与历史线带兼容的 48 字节顶点。"""

        return struct.pack(
            "<ffffffffffff",
            position[0],
            position[1],
            position[2],
            0.0,
            1.0,
            0.0,
            u_coord,
            v_coord,
            1.0,
            1.0,
            1.0,
            _TRAIL_ALPHA,
        )
