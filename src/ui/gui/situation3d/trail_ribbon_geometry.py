"""Qt Quick 3D 尾迹拖尾带几何体。注意：只负责把尾迹点显示成连续 ribbon。"""

from __future__ import annotations

import json
import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

_FLOAT_SIZE = 4
_RIBBON_COMPONENTS = 12
_RIBBON_STRIDE = _RIBBON_COMPONENTS * _FLOAT_SIZE


class TrailRibbonGeometry(QQuick3DGeometry):
    """单条飞机尾迹拖尾带。注意：QML 通过 pathValue/widthValue 更新几何。"""

    pathValueChanged = Signal()
    widthValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化空拖尾带。注意：收到路径数据前保持空几何。"""

        super().__init__(parent)
        self._path_value = "[]"
        self._width_value = 220.0
        self._rebuild()

    @Property(str, notify=pathValueChanged)
    def pathValue(self) -> str:
        """返回 JSON 编码的 Quick3D 坐标点列。"""

        return self._path_value

    @pathValue.setter
    def pathValue(self, value: str) -> None:
        """更新尾迹点列。注意：无效 JSON 按空路径处理。"""

        normalized = value if isinstance(value, str) else "[]"
        if normalized == self._path_value:
            return
        self._path_value = normalized
        self._rebuild()
        self.pathValueChanged.emit()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回拖尾带宽度，单位为显示层米。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """更新拖尾带宽度。注意：宽度过小会导致远景不可见。"""

        try:
            normalized = float(value)
        except (TypeError, ValueError):
            normalized = self._width_value
        if not math.isfinite(normalized):
            normalized = self._width_value
        normalized = max(4.0, min(360.0, normalized))
        if math.isclose(normalized, self._width_value, rel_tol=1e-6):
            return
        self._width_value = normalized
        self._rebuild()
        self.widthValueChanged.emit()

    def _rebuild(self) -> None:
        """按当前路径重建 ribbon 顶点和索引。"""

        points = self._parse_points()
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_RIBBON_STRIDE)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 3 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic, 6 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.ColorSemantic, 8 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)
        if len(points) < 2:
            # 少于两个点无法形成带面，提交空几何避免 QML 渲染残留旧数据。
            self.setVertexData(QByteArray())
            self.setIndexData(QByteArray())
            self.setBounds(QVector3D(), QVector3D())
            self.update()
            return
        vertices, indices = self._build_mesh(points)
        self._apply_bounds(points)
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()

    def _parse_points(self) -> list[tuple[float, float, float]]:
        """解析 QML 传入的 JSON 点列。注意：非法点会被跳过。"""

        try:
            raw_points = json.loads(self._path_value)
        except json.JSONDecodeError:
            return []
        points: list[tuple[float, float, float]] = []
        if not isinstance(raw_points, list):
            return points
        for item in raw_points:
            if not isinstance(item, list | tuple) or len(item) != 3:
                continue
            try:
                x, y, z = (float(item[0]), float(item[1]), float(item[2]))
            except (TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
        return points

    def _build_mesh(self, points: list[tuple[float, float, float]]) -> tuple[bytearray, bytearray]:
        """构建三角带 mesh。注意：每个路径点生成左右两个顶点。"""

        half_width = self._width_value / 2.0
        vertices = bytearray()
        indices = bytearray()
        last_index = len(points) - 1
        for index, point in enumerate(points):
            side = self._side_vector(points, index)
            # alpha 沿尾迹时间方向递增，越靠近当前飞机越清晰。
            alpha = 0.08 + 0.64 * (index / max(1, last_index))
            u_coord = index / max(1, last_index)
            # 每个采样点展开成左右两点，所有采样点连起来就是一条三角带。
            left = (point[0] - side[0] * half_width, point[1], point[2] - side[2] * half_width)
            right = (point[0] + side[0] * half_width, point[1], point[2] + side[2] * half_width)
            self._append_vertex(vertices, left, u_coord, 0.0, alpha)
            self._append_vertex(vertices, right, u_coord, 1.0, alpha)
        for index in range(last_index):
            left_a = index * 2
            right_a = left_a + 1
            left_b = left_a + 2
            right_b = left_a + 3
            indices.extend(struct.pack("<IIIIII", left_a, left_b, right_a, right_a, left_b, right_b))
        return vertices, indices

    def _side_vector(self, points: list[tuple[float, float, float]], index: int) -> tuple[float, float, float]:
        """返回当前点处的水平侧向单位向量。注意：退化段使用默认横向。"""

        if index == 0:
            previous = points[index]
            current = points[index + 1]
        elif index == len(points) - 1:
            previous = points[index - 1]
            current = points[index]
        else:
            previous = points[index - 1]
            current = points[index + 1]
        dx = current[0] - previous[0]
        dz = current[2] - previous[2]
        length = math.hypot(dx, dz)
        if length <= 1e-6:
            return 1.0, 0.0, 0.0
        # Quick3D 的 y 是高度，拖尾带宽度只在水平面展开，避免竖直方向变厚。
        return -dz / length, 0.0, dx / length

    def _append_vertex(
        self,
        vertices: bytearray,
        position: tuple[float, float, float],
        u_coord: float,
        v_coord: float,
        alpha: float,
    ) -> None:
        """追加 ribbon 顶点。注意：顶点色 alpha 用于渐隐拖尾。"""

        vertices.extend(
            struct.pack(
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
                alpha,
            )
        )

    def _apply_bounds(self, points: list[tuple[float, float, float]]) -> None:
        """设置几何包围盒。注意：包围盒需包含 ribbon 宽度，避免视锥误裁剪。"""

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        # 宽度方向可能伸出原始路径包围盒，margin 必须按 ribbon 宽度外扩。
        margin = self._width_value
        self.setBounds(
            QVector3D(min(xs) - margin, min(ys) - 2.0, min(zs) - margin),
            QVector3D(max(xs) + margin, max(ys) + 2.0, max(zs) + margin),
        )
