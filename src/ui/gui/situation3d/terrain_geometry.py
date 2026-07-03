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
# 线框只需要 position，减少 QQuick3D 上传的数据量。
_POSITION_STRIDE = 3 * _FLOAT_SIZE
# 曲面分辨率保持低频平滑，避免 GUI 场景因地形过密卡顿。
_SURFACE_COLUMNS = 58
_SURFACE_ROWS = 42
# 线框采样比曲面更稀疏，让网格可读而不是糊成一片。
_GRID_COLUMNS = 36
_GRID_ROWS = 26
# 线框略高于地表，避免与地表深度冲突产生闪烁。
_GRID_LIFT = 3.5


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


class TerrainGridGeometry(_TerrainGeometryBase):
    """贴合地表的网格线。注意：用于显示地形曲面起伏，不参与交互拾取。"""

    def _rebuild(self) -> None:
        """重建地表线框数据。注意：线段直接贴合高度场并略微抬高避免闪烁。"""

        vertices = bytearray()
        # 横向线提供地形深度方向的尺度参照。
        for row in range(_GRID_ROWS):
            for column in range(_GRID_COLUMNS - 1):
                self._append_line(vertices, row, column, row, column + 1)
        # 纵向线提供地形宽度方向的尺度参照。
        for column in range(_GRID_COLUMNS):
            for row in range(_GRID_ROWS - 1):
                self._append_line(vertices, row, column, row + 1, column)
        # 对角线模拟 demo 中的三角网格感，帮助用户读出曲面坡度。
        for row in range(_GRID_ROWS - 1):
            for column in range(_GRID_COLUMNS - 1):
                self._append_line(vertices, row, column, row + 1, column + 1)

        # 线框使用 Lines primitive，不占用三角面索引。
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Lines)
        self.setStride(_POSITION_STRIDE)
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.PositionSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.setBounds(
            QVector3D(-self._width_value / 2.0, 0.0, -self._depth_value / 2.0),
            QVector3D(self._width_value / 2.0, self._amplitude_value * 1.35 + 24.0, self._depth_value / 2.0),
        )
        self.setVertexData(QByteArray(bytes(vertices)))
        self.update()

    def _append_line(
        self,
        vertices: bytearray,
        start_row: int,
        start_column: int,
        end_row: int,
        end_column: int,
    ) -> None:
        """追加一条地表网格线段。注意：行列索引会映射到当前地形宽深。"""

        start = self._grid_point(start_row, start_column)
        end = self._grid_point(end_row, end_column)
        vertices.extend(struct.pack("<ffffff", start.x(), start.y(), start.z(), end.x(), end.y(), end.z()))

    def _grid_point(self, row: int, column: int) -> QVector3D:
        """返回网格采样点坐标。注意：高度比地表略高，避免与地表深度冲突。"""

        # 行列索引映射到局部坐标，使线框随 payload 尺寸等比缩放。
        x = -self._width_value / 2.0 + self._width_value * column / (_GRID_COLUMNS - 1)
        z = -self._depth_value / 2.0 + self._depth_value * row / (_GRID_ROWS - 1)
        return QVector3D(x, self._height_at(x, z) + _GRID_LIFT, z)


def _height_value(x: float, z: float, width: float, depth: float, amplitude: float) -> float:
    """计算连续地形高度。注意：多个宽高斯峰叠加，避免独立石块感。"""

    # nx/nz 采用 [-0.5, 0.5] 左右的局部比例，场景放大时山形保持相似。
    nx = x / width
    nz = z / depth
    # 轻微起伏打破完全轴对齐的人工感，但幅值足够小，不会变成噪声地形。
    rolling = 0.035 * (math.sin(nx * math.tau * 2.0 + 0.4) + math.cos(nz * math.tau * 1.8 - 0.2))
    height_mix = (
        # 左前宽峰。
        0.88 * _gaussian(nx, nz, -0.30, -0.22, 0.20, 0.18)
        # 右后主峰。
        + 0.98 * _gaussian(nx, nz, 0.28, 0.18, 0.26, 0.22)
        # 中后低峰。
        + 0.58 * _gaussian(nx, nz, 0.02, -0.34, 0.18, 0.15)
        # 中央宽脊把几个峰连成连续地形。
        + 0.34 * _gaussian(nx, nz, -0.06, 0.02, 0.48, 0.36)
        # 左后补峰让画面不只剩单一山头。
        + 0.22 * _gaussian(nx, nz, -0.34, 0.26, 0.24, 0.18)
        + rolling
    )
    # 边缘衰减让地形接近地面，避免可见边界处像被切开的实体。
    return 4.0 + amplitude * _edge_falloff(nx, nz) * max(0.0, height_mix)


def _gaussian(nx: float, nz: float, center_x: float, center_z: float, sigma_x: float, sigma_z: float) -> float:
    """返回二维高斯权重。注意：输入坐标为地形宽深归一化后的局部比例。"""

    # sigma 分别控制东西向、南北向宽度，允许生成椭圆缓坡。
    dx = (nx - center_x) / sigma_x
    dz = (nz - center_z) / sigma_z
    return math.exp(-0.5 * (dx * dx + dz * dz))


def _edge_falloff(nx: float, nz: float) -> float:
    """返回地形边缘衰减系数。注意：避免山体在边界突然截断。"""

    # 0.16 的归一化边距约等于地形短边三分之一的缓冲带。
    margin = 0.16
    edge_x = max(0.0, min(1.0, (0.5 - abs(nx)) / margin))
    edge_z = max(0.0, min(1.0, (0.5 - abs(nz)) / margin))
    edge = min(edge_x, edge_z)
    # smoothstep 保证边缘高度和一阶变化都更平滑。
    return edge * edge * (3.0 - 2.0 * edge)
