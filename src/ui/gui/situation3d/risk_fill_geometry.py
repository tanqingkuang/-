"""Qt Quick 3D 危险区贴地填充几何。注意：三角网由 scene_data 预先计算，这里只做上传。"""

from __future__ import annotations

import json
import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

_FLOAT_SIZE = 4
# 顶点布局只保留 position(3) + normal(3)：填充层不用贴图和顶点色，越简单越省显存。
_FILL_COMPONENTS = 6
_FILL_STRIDE = _FILL_COMPONENTS * _FLOAT_SIZE

# 设计说明：
# 1. meshValue 是 JSON 对象 {"v": [[x,y,z]...], "t": [i0,i1,i2,...]}，坐标为 Quick3D 世界米。
# 2. 三角化、地形高度采样都在 scene_data 完成并缓存；本类不了解障碍或地形语义。
# 3. 覆盖层是静态几何，只有材质 opacity 参与呼吸动画，字符串不变时 setter 直接短路。
# 4. 法线固定朝上：填充只是提示色层，不承担真实地表光照，材质端关闭背面剔除。
# 5. 非法 JSON、越界索引一律清空几何，宁可不显示也不能让 3D 场景崩溃或花屏。


class RiskFillGeometry(QQuick3DGeometry):
    """危险区贴地填充 mesh。注意：QML 通过 Model.geometry 直接渲染。"""

    meshValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化空填充几何。注意：QML 未赋值前保持合法空 mesh。"""

        super().__init__(parent)
        self._mesh_value = "{}"
        self._rebuild()

    @Property(str, notify=meshValueChanged)
    def meshValue(self) -> str:
        """返回 JSON 编码的三角网数据。"""

        return self._mesh_value

    @meshValue.setter
    def meshValue(self, value: str) -> None:
        """更新三角网数据。注意：同值赋值直接短路，避免静态帧重复上传。"""

        normalized = value if isinstance(value, str) else "{}"
        if normalized == self._mesh_value:
            return
        self._mesh_value = normalized
        self._rebuild()
        self.meshValueChanged.emit()

    def _rebuild(self) -> None:
        """按 meshValue 重建顶点与索引。注意：任何坏数据都降级为空几何。"""

        vertices, triangles = self._parse_mesh(self._mesh_value)
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_FILL_STRIDE)
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
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )
        if not vertices or not triangles:
            self.setVertexData(QByteArray())
            self.setIndexData(QByteArray())
            self.setBounds(QVector3D(), QVector3D())
            self.update()
            return
        vertex_bytes = bytearray()
        for x_coord, y_coord, z_coord in vertices:
            # 法线固定向上；覆盖层贴地摆放，光照层次仍由地形本体承担。
            vertex_bytes.extend(struct.pack("<ffffff", x_coord, y_coord, z_coord, 0.0, 1.0, 0.0))
        index_bytes = bytearray()
        for index in triangles:
            index_bytes.extend(struct.pack("<I", index))
        # 先设置包围盒再提交数据，保证首帧视锥裁剪拿到正确范围。
        self._apply_bounds(vertices)
        self.setVertexData(QByteArray(bytes(vertex_bytes)))
        self.setIndexData(QByteArray(bytes(index_bytes)))
        self.update()

    @staticmethod
    def _parse_mesh(value: str) -> tuple[list[tuple[float, float, float]], list[int]]:
        """解析 meshValue。注意：任一顶点或索引非法都整体作废，不做部分渲染。"""

        try:
            raw = json.loads(value)
        except (json.JSONDecodeError, TypeError, ValueError):
            return [], []
        if not isinstance(raw, dict):
            return [], []
        raw_vertices = raw.get("v", [])
        raw_triangles = raw.get("t", [])
        if not isinstance(raw_vertices, list) or not isinstance(raw_triangles, list):
            return [], []
        vertices: list[tuple[float, float, float]] = []
        for item in raw_vertices:
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                return [], []
            try:
                point = (float(item[0]), float(item[1]), float(item[2]))
            except (TypeError, ValueError):
                return [], []
            if not all(math.isfinite(component) for component in point):
                return [], []
            vertices.append(point)
        # 索引必须按三个一组成面，并且全部落在顶点表范围内。
        if len(raw_triangles) % 3 != 0:
            return [], []
        triangles: list[int] = []
        for item in raw_triangles:
            try:
                index = int(item)
            except (TypeError, ValueError):
                return [], []
            if index < 0 or index >= len(vertices):
                return [], []
            triangles.append(index)
        return vertices, triangles

    def _apply_bounds(self, vertices: list[tuple[float, float, float]]) -> None:
        """设置包围盒。注意：Y 方向外扩少量余量，避免抬升薄层被视锥提前裁剪。"""

        xs = [point[0] for point in vertices]
        ys = [point[1] for point in vertices]
        zs = [point[2] for point in vertices]
        self.setBounds(
            QVector3D(min(xs), min(ys) - 2.0, min(zs)),
            QVector3D(max(xs), max(ys) + 2.0, max(zs)),
        )
