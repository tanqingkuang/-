"""Qt Quick 3D 连续地形几何体。注意：只负责显示层高度场，不参与仿真计算。"""

from __future__ import annotations

import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

# 顶点布局使用 32 字节：position(3) + normal(3) + uv(2)。
_FLOAT_SIZE = 4
_SURFACE_COMPONENTS = 8
_SURFACE_STRIDE = _SURFACE_COMPONENTS * _FLOAT_SIZE
# 曲面分辨率要覆盖 0.8km 小丘陵，20km 地图下约 210m 一个采样点。
_SURFACE_COLUMNS = 96
_SURFACE_ROWS = 96
# 丘陵按米定义半径，适配 20km x 20km 态势地图。
# 元组字段依次为局部 x、局部 z、长半轴、短半轴、旋转角、相对高度。
# 半轴都控制在 0.8km 到 3km 附近，避免做成整张地图的大山包。
_HILL_PROFILES = (
    (-5200.0, -3600.0, 3000.0, 1900.0, -18.0, 0.58),
    (3600.0, -2200.0, 2400.0, 1500.0, 24.0, 0.74),
    (-1800.0, 4300.0, 1500.0, 950.0, 37.0, 0.46),
    (6100.0, 4400.0, 950.0, 680.0, -32.0, 0.32),
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
        # 索引属性指向独立 indexData，减少重复顶点上传。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )
        # 先设置包围盒再提交数据，保证首帧视锥裁剪拿到最新范围。
        self._apply_bounds()
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()

    def _append_vertex(self, vertices: bytearray, x: float, z: float, u_coord: float, v_coord: float) -> None:
        """追加单个地表顶点。注意：每个顶点包含位置、法线和纹理坐标。"""

        y = self._height_at(x, z)
        # 法线使用同一高度函数，保证光照方向和实际顶点高度一致。
        normal = self._normal_at(x, z)
        vertices.extend(
            struct.pack(
                "<ffffffff",
                x,
                y,
                z,
                normal.x(),
                normal.y(),
                normal.z(),
                u_coord,
                v_coord,
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
    # 低频起伏只提供地表微弯，不再做可见纹理。
    rolling = 0.012 * (
        math.sin(nx * math.tau * 3.0 + 0.4)
        + math.cos(nz * math.tau * 2.4 - 0.2)
        + 0.5 * math.sin((nx + nz) * math.tau * 2.2)
    )
    height_mix = rolling
    for center_x, center_z, radius_x, radius_z, angle_deg, weight in _HILL_PROFILES:
        # 旋转椭圆丘陵避免俯视时出现机械圆斑。
        height_mix += weight * _elliptic_hill(x, z, center_x, center_z, radius_x, radius_z, angle_deg)
    # 这里只输出几何高度，颜色和材质交给 QML，避免伪纹理再次变成碎斑。
    # 边缘衰减让地形接近地面，避免可见边界处像被切开的实体。
    return 4.0 + amplitude * _edge_falloff(nx, nz) * max(0.0, height_mix)


def _elliptic_hill(
    x: float,
    z: float,
    center_x: float,
    center_z: float,
    radius_x: float,
    radius_z: float,
    angle_deg: float,
) -> float:
    """返回米制旋转椭圆丘陵权重。注意：半轴是丘陵的主要可见范围。"""

    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = x - center_x
    dz = z - center_z
    # 先旋转到丘陵局部坐标，再按长短半轴归一化距离。
    local_x = (dx * cos_a + dz * sin_a) / radius_x
    local_z = (-dx * sin_a + dz * cos_a) / radius_z
    distance_ratio = math.hypot(local_x, local_z)
    if distance_ratio >= 1.0:
        return 0.0
    # 余弦缓坡没有平顶，边界处也没有硬折线。
    return 0.5 + 0.5 * math.cos(math.pi * distance_ratio)


def _edge_falloff(nx: float, nz: float) -> float:
    """返回地形边缘衰减系数。注意：避免山体在边界突然截断。"""

    # 0.16 的归一化边距约等于地形短边三分之一的缓冲带。
    margin = 0.16
    edge_x = max(0.0, min(1.0, (0.5 - abs(nx)) / margin))
    edge_z = max(0.0, min(1.0, (0.5 - abs(nz)) / margin))
    edge = min(edge_x, edge_z)
    # smoothstep 保证边缘高度和一阶变化都更平滑。
    return edge * edge * (3.0 - 2.0 * edge)
