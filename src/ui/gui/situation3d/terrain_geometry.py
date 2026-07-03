"""Qt Quick 3D 连续地形几何体。注意：只负责显示层高度场，不参与仿真计算。"""

from __future__ import annotations

import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

# 顶点布局使用 48 字节：position(3) + normal(3) + uv(2) + color(4)。
_FLOAT_SIZE = 4
_SURFACE_COMPONENTS = 12
_SURFACE_STRIDE = _SURFACE_COMPONENTS * _FLOAT_SIZE
# 曲面分辨率要覆盖 0.8km 小丘陵，20km 地图下约 210m 一个采样点。
_SURFACE_COLUMNS = 96
_SURFACE_ROWS = 96
# 丘陵按米定义半径，适配 20km x 20km 态势地图。
# 元组字段依次为局部 x、局部 z、可见半径、相对高度。
_HILL_PROFILES = (
    (-5200.0, -3600.0, 3000.0, 0.58),
    (3600.0, -2200.0, 2200.0, 0.76),
    (-1800.0, 4300.0, 1400.0, 0.48),
    (6100.0, 4400.0, 900.0, 0.34),
)


class _TerrainGeometryBase(QQuick3DGeometry):
    """地形几何基类。注意：只承载 QML 可调参数和共同高度函数。"""

    widthValueChanged = Signal()
    depthValueChanged = Signal()
    amplitudeValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化地形参数。注意：子类负责把参数转换成具体几何数据。"""

        super().__init__(parent)
        # 默认值覆盖无快照时的空场景尺寸，首次 payload 到达后会被 QML 覆盖。
        self._width_value = 3000.0
        self._depth_value = 2200.0
        self._amplitude_value = 260.0
        self._rebuild()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回地形宽度，单位为显示层米。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """更新地形宽度。注意：宽度变化会立即重建几何数据。"""

        # QML 属性有可能先传 0 或 NaN，显示层只接受有意义的正尺寸。
        normalized = self._positive(value, self._width_value, 400.0)
        if math.isclose(normalized, self._width_value, rel_tol=1e-6):
            return
        self._width_value = normalized
        self._rebuild()
        self.widthValueChanged.emit()

    @Property(float, notify=depthValueChanged)
    def depthValue(self) -> float:
        """返回地形深度，单位为显示层米。"""

        return self._depth_value

    @depthValue.setter
    def depthValue(self, value: float) -> None:
        """更新地形深度。注意：深度变化会立即重建几何数据。"""

        # 深度下限防止极小场景下网格采样步长退化。
        normalized = self._positive(value, self._depth_value, 300.0)
        if math.isclose(normalized, self._depth_value, rel_tol=1e-6):
            return
        self._depth_value = normalized
        self._rebuild()
        self.depthValueChanged.emit()

    @Property(float, notify=amplitudeValueChanged)
    def amplitudeValue(self) -> float:
        """返回地形最大起伏控制量，单位为显示层米。"""

        return self._amplitude_value

    @amplitudeValue.setter
    def amplitudeValue(self, value: float) -> None:
        """更新地形起伏幅值。注意：幅值越大，山体越高。"""

        # 起伏幅值保留最低值，保证高度场在视觉上仍能提供空间参照。
        normalized = self._positive(value, self._amplitude_value, 30.0)
        if math.isclose(normalized, self._amplitude_value, rel_tol=1e-6):
            return
        self._amplitude_value = normalized
        self._rebuild()
        self.amplitudeValueChanged.emit()

    def _rebuild(self) -> None:
        """重建具体几何数据。注意：仅由子类覆盖。"""

        raise NotImplementedError

    def _height_at(self, x: float, z: float) -> float:
        """计算指定局部坐标的地形高度。注意：宽深参数来自当前几何实例。"""

        return _height_value(x, z, self._width_value, self._depth_value, self._amplitude_value)

    @staticmethod
    def _positive(value: float, fallback: float, minimum: float) -> float:
        """把 QML 传入值规整为正数。注意：异常输入沿用当前值。"""

        try:
            normalized = float(value)
        except (TypeError, ValueError):
            return fallback
        if not math.isfinite(normalized):
            return fallback
        return max(minimum, normalized)


class TerrainGeometry(_TerrainGeometryBase):
    """连续高度场地表。注意：QML 通过 Model.geometry 直接渲染这张 mesh。"""

    def _rebuild(self) -> None:
        """重建地表顶点、法线、纹理坐标和索引数据。"""

        width = self._width_value
        depth = self._depth_value
        vertices = bytearray()
        indices = bytearray()

        # 先生成完整顶点表，再生成索引，便于法线按相邻采样点统一计算。
        for row in range(_SURFACE_ROWS):
            z = -depth / 2.0 + depth * row / (_SURFACE_ROWS - 1)
            v_coord = row / (_SURFACE_ROWS - 1)
            for column in range(_SURFACE_COLUMNS):
                x = -width / 2.0 + width * column / (_SURFACE_COLUMNS - 1)
                u_coord = column / (_SURFACE_COLUMNS - 1)
                self._append_vertex(vertices, x, z, u_coord, v_coord)

        # 每个网格单元拆成两个三角面，保持地表是一张连续 mesh。
        for row in range(_SURFACE_ROWS - 1):
            for column in range(_SURFACE_COLUMNS - 1):
                self._append_cell(indices, row, column)

        # clear 会移除上一帧属性和数据，避免尺寸更新后残留旧布局。
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_SURFACE_STRIDE)
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.PositionSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # 法线偏移紧跟 position，供 Quick3D 做平滑光照。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.NormalSemantic,
            3 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # uv 暂时只服务材质扩展，后续接贴图时不用重排顶点。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic,
            6 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # 顶点色承担地形高度分层和稳定明暗，避免 20km 场景纯灯光过黑。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.ColorSemantic,
            8 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # 索引属性指向独立 indexData，减少重复顶点上传。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )
        self._apply_bounds()
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()

    def _append_vertex(self, vertices: bytearray, x: float, z: float, u_coord: float, v_coord: float) -> None:
        """追加单个地表顶点。注意：每个顶点包含位置、法线和纹理坐标。"""

        y = self._height_at(x, z)
        # 法线使用同一高度函数，保证光照方向和实际顶点高度一致。
        normal = self._normal_at(x, z)
        # 颜色随高度和法线变化，QML 侧使用顶点色避免大地图灯光失真。
        color = self._color_at(y, normal)
        vertices.extend(
            struct.pack(
                "<ffffffffffff",
                x,
                y,
                z,
                normal.x(),
                normal.y(),
                normal.z(),
                u_coord,
                v_coord,
                color[0],
                color[1],
                color[2],
                color[3],
            )
        )

    def _append_cell(self, indices: bytearray, row: int, column: int) -> None:
        """追加一个网格单元的两个三角面。注意：索引顺序保持法线朝上。"""

        top_left = row * _SURFACE_COLUMNS + column
        top_right = top_left + 1
        bottom_left = top_left + _SURFACE_COLUMNS
        bottom_right = bottom_left + 1
        # Qt Quick 3D 使用顶点绕序做背面剔除，三角面必须从上方可见。
        indices.extend(
            struct.pack(
                "<IIIIII",
                top_left,
                bottom_left,
                top_right,
                top_right,
                bottom_left,
                bottom_right,
            )
        )

    def _normal_at(self, x: float, z: float) -> QVector3D:
        """计算地形法线。注意：用有限差分获得平滑光照方向。"""

        # 差分步长等于网格间距，避免边缘局部坡度被过度放大。
        step_x = self._width_value / (_SURFACE_COLUMNS - 1)
        step_z = self._depth_value / (_SURFACE_ROWS - 1)
        gradient_x = (self._height_at(x + step_x, z) - self._height_at(x - step_x, z)) / (2.0 * step_x)
        gradient_z = (self._height_at(x, z + step_z) - self._height_at(x, z - step_z)) / (2.0 * step_z)
        # 高度场 y=f(x,z) 的上法线是 (-df/dx, 1, -df/dz)。
        normal = QVector3D(-gradient_x, 1.0, -gradient_z)
        normal.normalize()
        return normal

    def _color_at(self, height: float, normal: QVector3D) -> tuple[float, float, float, float]:
        """计算地形顶点色。注意：颜色包含稳定的高度分层和伪光照明暗。"""

        # 0.72 留出余量，让最高丘陵接近但不直接夹到纯顶色。
        height_ratio = max(0.0, min(1.0, (height - 4.0) / max(1.0, self._amplitude_value * 0.72)))
        # 低地偏沉稳绿色，丘顶偏浅黄绿，便于无光照模式下读高度。
        low_color = (0.30, 0.48, 0.35)
        high_color = (0.66, 0.86, 0.55)
        base = tuple(low_color[index] + (high_color[index] - low_color[index]) * height_ratio for index in range(3))
        # 伪光源只影响顶点色，不让 Qt 实时光照把背坡压成黑斑。
        light = QVector3D(-0.42, 0.78, 0.46)
        light.normalize()
        light_mix = max(0.0, QVector3D.dotProduct(normal, light))
        # shade 下限高于 0，背坡不会再掉成黑斑。
        shade = 0.78 + 0.18 * light_mix + 0.08 * height_ratio
        return (min(1.0, base[0] * shade), min(1.0, base[1] * shade), min(1.0, base[2] * shade), 1.0)

    def _apply_bounds(self) -> None:
        """设置地形包围盒。注意：包围盒影响 Qt Quick 3D 视锥裁剪。"""

        self.setBounds(
            QVector3D(-self._width_value / 2.0, 0.0, -self._depth_value / 2.0),
            QVector3D(self._width_value / 2.0, self._amplitude_value * 1.35 + 16.0, self._depth_value / 2.0),
        )


def _height_value(x: float, z: float, width: float, depth: float, amplitude: float) -> float:
    """计算连续地形高度。注意：多个宽高斯峰叠加，避免独立石块感。"""

    nx = x / width
    nz = z / depth
    # 轻微底噪只打破完全平面，不能抢过 3-4 个主体丘陵。
    rolling = 0.006 * (
        math.sin(nx * math.tau * 3.0 + 0.4)
        + math.cos(nz * math.tau * 2.4 - 0.2)
        + 0.5 * math.sin((nx + nz) * math.tau * 2.2)
    )
    height_mix = rolling
    for center_x, center_z, radius, weight in _HILL_PROFILES:
        # 每个丘陵的可见半径保持在 0.8km 到 3km，和 20km 地图尺度匹配。
        height_mix += weight * _radial_hill(x, z, center_x, center_z, radius)
    # 边缘衰减让地形接近地面，避免可见边界处像被切开的实体。
    return 4.0 + amplitude * _edge_falloff(nx, nz) * max(0.0, height_mix)


def _radial_hill(x: float, z: float, center_x: float, center_z: float, radius: float) -> float:
    """返回米制径向丘陵权重。注意：radius 近似为视觉半径而不是高斯标准差。"""

    # 有限半径避免高斯长尾把整张地图染成连续碎坡。
    distance_ratio = math.hypot(x - center_x, z - center_z) / radius
    if distance_ratio >= 1.0:
        return 0.0
    falloff = 1.0 - distance_ratio * distance_ratio
    # smootherstep 在峰顶和半径边界都更平滑。
    return falloff * falloff * falloff * (10.0 - 15.0 * falloff + 6.0 * falloff * falloff)


def _edge_falloff(nx: float, nz: float) -> float:
    """返回地形边缘衰减系数。注意：避免山体在边界突然截断。"""

    # 0.16 的归一化边距约等于地形短边三分之一的缓冲带。
    margin = 0.16
    edge_x = max(0.0, min(1.0, (0.5 - abs(nx)) / margin))
    edge_z = max(0.0, min(1.0, (0.5 - abs(nz)) / margin))
    edge = min(edge_x, edge_z)
    # smoothstep 保证边缘高度和一阶变化都更平滑。
    return edge * edge * (3.0 - 2.0 * edge)
