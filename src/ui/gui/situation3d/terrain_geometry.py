"""Qt Quick 3D 连续地形几何体。注意：只负责显示层高度场，不参与仿真计算。"""

from __future__ import annotations

import logging
import math
import json
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry
import numpy as np

from src.ui.gui.situation3d.terrain_field import (
    DEFAULT_TERRAIN_RESOLUTION,
    TerrainField,
    peek_terrain_field,
)

# 顶点布局使用 72 字节：position(3) + normal(3) + uv(2) + tangent(3) + binormal(3) + color(4)。
_FLOAT_SIZE = 4
_SURFACE_COMPONENTS = 18
_SURFACE_STRIDE = _SURFACE_COMPONENTS * _FLOAT_SIZE
# 192 格在 20km 地图下约 104m 一个采样点，最小丘陵也有 20 个以上采样跨度。
_SURFACE_COLUMNS = 192
_SURFACE_ROWS = 192
# 山体按 20km x 20km 基准地图定义，地图变大时按比例整体拉伸布局。
_HILL_LAYOUT_SPAN_M = 20000.0
# 元组字段依次为局部 x、局部 z、长半轴、短半轴、旋转角、相对高度。
# 高斯核无限支撑，山脚互相叠加成山脉群，避免孤立馒头。
_HILL_PROFILES = (
    (-5200.0, -3600.0, 3200.0, 2000.0, -18.0, 1.18),
    (4600.0, -3400.0, 2600.0, 1700.0, 24.0, 0.95),
    (-6800.0, 1800.0, 2200.0, 1400.0, 8.0, 0.72),
    (-2600.0, 5200.0, 1900.0, 1200.0, 37.0, 0.62),
    (1400.0, -6600.0, 1800.0, 1100.0, -52.0, 0.58),
    (6100.0, 4400.0, 1500.0, 1000.0, -32.0, 0.55),
)
# 中心保护区：飞机巡航高度只有几十米，航迹区必须保持接近平地。
_CLEAR_RADIUS_RATIO = 0.13
_CLEAR_BLEND_RATIO = 0.30
# 无布局占位地形沿用正式场景的低饱和岩土地貌：低处深绿灰、中段岩石灰绿、峰顶暖灰。
# 这里不能保留旧的亮草绿，否则后台高度场替换完成前会短暂闪回塑料山体。
_COLOR_LOW = (0.105, 0.155, 0.150)
_COLOR_MID = (0.255, 0.285, 0.270)
_COLOR_HIGH = (0.535, 0.510, 0.465)
# 中段提前进入岩石灰，压缩大面积绿色在斜俯视图中的占比。
_COLOR_SPLIT = 0.38
# 障碍风险色采用低饱和铁锈红，只改变地形色相，不制造新的发光实体。
_RISK_TINT_SRGB = (0.44, 0.22, 0.20)
_RISK_TINT_STRENGTH = 0.30

_RiskArea = tuple[str, float, float, float, tuple[tuple[float, float], ...], float]


class _TerrainGeometryBase(QQuick3DGeometry):
    """地形几何基类。注意：只承载 QML 可调参数和共同高度函数。"""

    widthValueChanged = Signal()
    depthValueChanged = Signal()
    amplitudeValueChanged = Signal()
    layoutFileChanged = Signal()
    layoutRevisionChanged = Signal()
    resolutionValueChanged = Signal()
    generationTimeMsChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化地形参数。注意：子类负责把参数转换成具体几何数据。"""

        super().__init__(parent)
        # 默认值覆盖无快照时的空场景尺寸，首次 payload 到达后会被 QML 覆盖。
        self._width_value = 3000.0
        self._depth_value = 2200.0
        self._amplitude_value = 260.0
        self._layout_file_value = ""
        self._layout_revision_value = ""
        self._resolution_value = DEFAULT_TERRAIN_RESOLUTION
        self._generation_time_ms = 0.0
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

    @Property(str, notify=layoutFileChanged)
    def layoutFile(self) -> str:
        """返回地形布局文件路径。注意：空字符串表示使用旧参数化地形。"""

        return self._layout_file_value

    @layoutFile.setter
    def layoutFile(self, value: str) -> None:
        """更新地形布局文件路径。注意：文件变化时才触发新高度场生成。"""

        normalized = str(value or "")
        if normalized == self._layout_file_value:
            return
        self._layout_file_value = normalized
        self._rebuild()
        self.layoutFileChanged.emit()

    @Property(str, notify=layoutRevisionChanged)
    def layoutRevision(self) -> str:
        """返回布局版本号(mtime+就绪标志)。注意：同路径原地改文件或高度场就绪时变化。"""

        return self._layout_revision_value

    @layoutRevision.setter
    def layoutRevision(self, value: str) -> None:
        """更新布局版本号。注意：版本变化触发重建,解决同路径重载混用旧 mesh 的问题。"""

        normalized = str(value or "")
        if normalized == self._layout_revision_value:
            return
        self._layout_revision_value = normalized
        self._rebuild()
        self.layoutRevisionChanged.emit()

    @Property(int, notify=resolutionValueChanged)
    def resolutionValue(self) -> int:
        """返回布局地形网格分辨率。注意：无布局文件时该值不影响旧地形。"""

        return self._resolution_value

    @resolutionValue.setter
    def resolutionValue(self, value: int) -> None:
        """更新布局地形网格分辨率。注意：默认 641，低配可降到 384。"""

        try:
            normalized = int(value)
        except (TypeError, ValueError):
            normalized = DEFAULT_TERRAIN_RESOLUTION
        normalized = max(96, min(1024, normalized))
        if normalized == self._resolution_value:
            return
        self._resolution_value = normalized
        self._rebuild()
        self.resolutionValueChanged.emit()

    @Property(float, notify=generationTimeMsChanged)
    def generationTimeMs(self) -> float:
        """返回最近一次布局高度场生成耗时，单位毫秒。"""

        return self._generation_time_ms

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

    riskAreasValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化地形与风险着色范围。注意：风险范围使用地形模型局部坐标。"""

        # 保存原始字符串可让 QML 每帧重复赋同值时直接短路。
        self._risk_areas_value = "[]"
        # 解析后的定长元组供 192/768 网格重建共用，避免在顶点循环里反复读 JSON。
        self._risk_areas: tuple[_RiskArea, ...] = ()
        super().__init__(parent)

    @Property(str, notify=riskAreasValueChanged)
    def riskAreasValue(self) -> str:
        """返回风险着色 JSON。注意：圆与多边形均保持原始形状。"""

        return self._risk_areas_value

    @riskAreasValue.setter
    def riskAreasValue(self, value: str) -> None:
        """更新风险着色范围。注意：仅静态障碍变化时才重建地形顶点色。"""

        normalized = str(value or "[]")
        if normalized == self._risk_areas_value:
            return
        # 先保存输入，即使坏 JSON 也能稳定短路同一错误值。
        self._risk_areas_value = normalized
        # 单条坏障碍由解析器忽略，其余合法范围仍继续显示。
        self._risk_areas = _parse_risk_areas(normalized)
        # 风险色属于顶点属性，范围变化后必须重新上传整张地形网格。
        self._rebuild()
        self.riskAreasValueChanged.emit()

    def _rebuild(self) -> None:
        """重建地表顶点、法线、纹理坐标、顶点色和索引数据。"""

        if self._layout_file_value:
            try:
                # 非阻塞获取共享高度场:未就绪先落旧地形占位,payload 的 revision
                # 在就绪后翻转,QML 重设本属性触发再次重建完成替换,首开不冻结主线程。
                field = peek_terrain_field(self._layout_file_value, resolution=self._resolution_value)
                if field is not None:
                    self._rebuild_from_field(field)
                    return
            except (OSError, ValueError, json.JSONDecodeError, TypeError, KeyError, OverflowError) as error:
                # 布局文件异常时回落旧地形，避免 3D 窗口空白;诊断进日志供排障。
                logging.getLogger(__name__).warning("地形布局 %s 不可用,回退旧地形: %s", self._layout_file_value, error)
            self._generation_time_ms = 0.0
            self.generationTimeMsChanged.emit()

        width = self._width_value
        depth = self._depth_value
        step_x = width / (_SURFACE_COLUMNS - 1)
        step_z = depth / (_SURFACE_ROWS - 1)

        # 高度先整表采样（含一圈影子点），法线直接用相邻格点差分，
        # 避免每个顶点重复调用 4 次高度函数拖慢重建。
        heights = self._sample_height_grid(step_x, step_z)
        min_height = min(min(row[1:-1]) for row in heights[1:-1])
        max_height = max(max(row[1:-1]) for row in heights[1:-1])
        local_x = np.linspace(-width / 2.0, width / 2.0, _SURFACE_COLUMNS, dtype=np.float32)
        local_z = np.linspace(-depth / 2.0, depth / 2.0, _SURFACE_ROWS, dtype=np.float32)
        x_grid, z_grid = np.meshgrid(local_x, local_z)
        # 占位地形也使用真实障碍范围，保证没有布局文件的普通 3D 场景不退回红色柱体。
        risk_weights = _risk_weight_grid(x_grid, z_grid, self._risk_areas, max(step_x, step_z) * 1.35)

        vertices = bytearray()
        indices = bytearray()
        for row in range(_SURFACE_ROWS):
            z = -depth / 2.0 + step_z * row
            v_coord = row / (_SURFACE_ROWS - 1)
            for column in range(_SURFACE_COLUMNS):
                x = -width / 2.0 + step_x * column
                u_coord = column / (_SURFACE_COLUMNS - 1)
                self._append_vertex(
                    vertices,
                    heights,
                    row,
                    column,
                    x,
                    z,
                    step_x,
                    step_z,
                    u_coord,
                    v_coord,
                    float(risk_weights[row, column]),
                )

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
        # 显式切线基让 PrincipledMaterial 的岩面法线贴图在 D3D/Metal/OpenGL 后端一致生效。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.TangentSemantic,
            8 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.BinormalSemantic,
            11 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # 顶点色按高度渐变，配合材质 vertexColorsEnabled 表达海拔层次。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.ColorSemantic,
            14 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        # 索引属性指向独立 indexData，减少重复顶点上传。
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )
        # 先设置包围盒再提交数据，保证首帧视锥裁剪拿到最新范围。
        self._apply_bounds(min_height, max_height)
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()

    def _rebuild_from_field(self, field: TerrainField) -> None:
        """把 terrain_field 输出转换为 QQuick3DGeometry。注意：顶点数据用 numpy 批量打包。"""

        rows = field.resolution
        columns = field.resolution
        local_x = np.linspace(-field.width_m / 2.0, field.width_m / 2.0, columns, dtype=np.float32)
        # Quick3D z 轴为 -north；height 行从 north 最小到最大，因此 local_z 反向排列。
        local_z = np.linspace(field.depth_m / 2.0, -field.depth_m / 2.0, rows, dtype=np.float32)
        x_grid, z_grid = np.meshgrid(local_x, local_z)
        u_grid = np.linspace(0.0, 1.0, columns, dtype=np.float32)[None, :].repeat(rows, axis=0)
        v_grid = np.linspace(0.0, 1.0, rows, dtype=np.float32)[:, None].repeat(columns, axis=1)

        display_heights = field.display_heights_m if field.display_heights_m is not None else field.heights_m
        display_normals = field.display_normals if field.display_normals is not None else field.normals
        vertices = np.empty((rows, columns, _SURFACE_COMPONENTS), dtype=np.float32)
        vertices[:, :, 0] = x_grid
        # local_z 的 linspace 从 +depth/2 递减,本身已完成 north→-z 翻转;
        # 高度/法线/颜色一律按原始行序取值,再叠 [::-1] 会把地形南北镜像,
        # 镜像面配原始法线导致朝东北的坡整体背光变黑(历史八轮"画面黑"的底层根因)。
        vertices[:, :, 1] = display_heights
        vertices[:, :, 2] = z_grid
        # y=h(east,north)、z=-north 的曲面法线为 (-dh/de, 1, +dh/dn)，_normal_grid 已按此输出。
        vertices[:, :, 3:6] = display_normals
        vertices[:, :, 6] = u_grid
        vertices[:, :, 7] = v_grid
        # u 沿 +x，v 沿 -z；先从法线恢复 x 坡度，再用叉乘构造严格正交的副切线。
        normal_y = np.maximum(display_normals[:, :, 1], 1e-6)
        gradient_x = -display_normals[:, :, 0] / normal_y
        tangent_length = np.sqrt(1.0 + gradient_x * gradient_x)
        tangents = np.dstack((1.0 / tangent_length, gradient_x / tangent_length, np.zeros_like(gradient_x))).astype(np.float32)
        binormals = np.cross(display_normals, tangents)
        binormal_length = np.maximum(np.linalg.norm(binormals, axis=2, keepdims=True), 1e-6)
        binormals = (binormals / binormal_length).astype(np.float32)
        vertices[:, :, 8:11] = tangents
        vertices[:, :, 11:14] = binormals
        # 正式布局网格直接在缓存颜色的副本上混色，不修改进程级 TerrainField 缓存。
        risk_weights = _risk_weight_grid(
            x_grid,
            z_grid,
            self._risk_areas,
            max(field.width_m / max(1, columns - 1), field.depth_m / max(1, rows - 1)) * 1.35,
        )
        vertices[:, :, 14:17] = _tint_risk_colors(field.colors, risk_weights)
        vertices[:, :, 17] = 1.0

        top_left = (np.arange(rows - 1, dtype=np.uint32)[:, None] * columns) + np.arange(columns - 1, dtype=np.uint32)[None, :]
        top_right = top_left + 1
        bottom_left = top_left + columns
        bottom_right = bottom_left + 1
        # 行方向 z 递减(北向),绕序必须与之匹配保持上表面为正面;
        # 绕序反了会让双面光照按取反法线着色,整张地形呈"从背面照亮"的均匀暗色。
        indices = np.stack((top_left, top_right, bottom_left, top_right, bottom_right, bottom_left), axis=2).astype(np.uint32)

        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_SURFACE_STRIDE)
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
            QQuick3DGeometry.Attribute.Semantic.TangentSemantic,
            8 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.BinormalSemantic,
            11 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.ColorSemantic,
            14 * _FLOAT_SIZE,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.IndexSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.U32Type,
        )
        self.setBounds(
            QVector3D(-field.width_m / 2.0, min(0.0, float(np.min(display_heights)) - 4.0), -field.depth_m / 2.0),
            QVector3D(field.width_m / 2.0, float(np.max(display_heights)) + 16.0, field.depth_m / 2.0),
        )
        self.setVertexData(QByteArray(vertices.reshape(-1, _SURFACE_COMPONENTS).tobytes()))
        self.setIndexData(QByteArray(indices.reshape(-1).tobytes()))
        self._width_value = field.width_m
        self._depth_value = field.depth_m
        self._amplitude_value = float(np.max(display_heights))
        self._generation_time_ms = field.generation_time_ms
        self.generationTimeMsChanged.emit()
        self.update()

    def _sample_height_grid(self, step_x: float, step_z: float) -> list[list[float]]:
        """整表采样高度场。注意：四周多采一圈影子点供边缘法线差分。"""

        width = self._width_value
        depth = self._depth_value
        heights: list[list[float]] = []
        for row in range(-1, _SURFACE_ROWS + 1):
            z = -depth / 2.0 + step_z * row
            line = [self._height_at(-width / 2.0 + step_x * column, z) for column in range(-1, _SURFACE_COLUMNS + 1)]
            heights.append(line)
        return heights

    def _append_vertex(
        self,
        vertices: bytearray,
        heights: list[list[float]],
        row: int,
        column: int,
        x: float,
        z: float,
        step_x: float,
        step_z: float,
        u_coord: float,
        v_coord: float,
        risk_weight: float,
    ) -> None:
        """追加单个地表顶点。注意：法线和颜色都来自同一张高度表。"""

        # 影子圈占一格，网格下标整体偏移 1。
        grid_row = row + 1
        grid_column = column + 1
        y = heights[grid_row][grid_column]
        # 中央差分梯度和顶点高度共用采样表，保证光照与几何一致。
        gradient_x = (heights[grid_row][grid_column + 1] - heights[grid_row][grid_column - 1]) / (2.0 * step_x)
        gradient_z = (heights[grid_row + 1][grid_column] - heights[grid_row - 1][grid_column]) / (2.0 * step_z)
        # 高度场 y=f(x,z) 的上法线是 (-df/dx, 1, -df/dz)。
        length = math.sqrt(gradient_x * gradient_x + 1.0 + gradient_z * gradient_z)
        normal_x = -gradient_x / length
        normal_y = 1.0 / length
        normal_z = -gradient_z / length
        tangent_length = math.sqrt(1.0 + gradient_x * gradient_x)
        tangent_x = 1.0 / tangent_length
        tangent_y = gradient_x / tangent_length
        # 占位地形的 v 沿 +z，T×N 形成与纹理坐标一致的副切线。
        binormal_x = tangent_y * normal_z
        binormal_y = -tangent_x * normal_z
        binormal_z = tangent_x * normal_y - tangent_y * normal_x
        red, green, blue = _tint_risk_color(_height_color(y, self._amplitude_value), risk_weight)
        vertices.extend(
            struct.pack(
                "<ffffffffffffffffff",
                x,
                y,
                z,
                normal_x,
                normal_y,
                normal_z,
                u_coord,
                v_coord,
                tangent_x,
                tangent_y,
                0.0,
                binormal_x,
                binormal_y,
                binormal_z,
                red,
                green,
                blue,
                1.0,
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

    def _apply_bounds(self, min_height: float, max_height: float) -> None:
        """按实测高度设置包围盒。注意：包围盒影响 Qt Quick 3D 视锥裁剪。"""

        self.setBounds(
            QVector3D(-self._width_value / 2.0, min(0.0, min_height - 4.0), -self._depth_value / 2.0),
            QVector3D(self._width_value / 2.0, max_height + 16.0, self._depth_value / 2.0),
        )


def _parse_risk_areas(value: str) -> tuple[_RiskArea, ...]:
    """解析 QML 传入的风险范围。注意：坏条目单独忽略，不能让 3D 地形整体消失。"""

    try:
        raw_areas = json.loads(value)
    except (json.JSONDecodeError, TypeError, ValueError):
        # 显示层坏数据按空风险区降级，基础地形仍可正常渲染。
        return ()
    if not isinstance(raw_areas, list):
        return ()
    areas: list[_RiskArea] = []
    for raw in raw_areas:
        # payload 允许以后混入额外元数据，非对象项不应影响当前障碍。
        if not isinstance(raw, dict):
            continue
        kind = str(raw.get("kind", ""))
        try:
            if kind == "circle":
                center = raw.get("center", [])
                # 圆心必须是地形局部 x/z 二元坐标。
                if not isinstance(center, list) or len(center) != 2:
                    continue
                center_x = float(center[0])
                center_z = float(center[1])
                radius = float(raw.get("radius", 0.0))
                if not all(math.isfinite(item) for item in (center_x, center_z, radius)) or radius <= 0.0:
                    continue
                # 圆形已经在 scene_data 中并入安全间距，这里无需再保存 clearance。
                areas.append((kind, center_x, center_z, radius, (), 0.0))
                continue
            if kind != "polygon":
                continue
            raw_points = raw.get("points", [])
            if not isinstance(raw_points, list):
                continue
            # 多边形点保持输入顺序，奇偶规则才能兼容当前山脉凹轮廓。
            points = tuple((float(point[0]), float(point[1])) for point in raw_points if isinstance(point, list) and len(point) == 2)
            clearance = max(0.0, float(raw.get("clearance", 0.0)))
            if len(points) < 3 or not math.isfinite(clearance):
                continue
            if not all(math.isfinite(coordinate) for point in points for coordinate in point):
                continue
            # clearance 留到距离场阶段处理，可得到真实圆角外扩而不是轴对齐放大。
            areas.append((kind, 0.0, 0.0, 0.0, points, clearance))
        except (TypeError, ValueError, OverflowError):
            # 单个数值转换失败只淘汰当前条目，避免清空其他已启用障碍。
            continue
    return tuple(areas)


def _risk_weight_grid(
    x_grid: np.ndarray,
    z_grid: np.ndarray,
    areas: tuple[_RiskArea, ...],
    feather_m: float,
) -> np.ndarray:
    """计算风险色混合权重。注意：边缘按一个网格量级柔化，避免多边形锯齿。"""

    weights = np.zeros(x_grid.shape, dtype=np.float32)
    if not areas:
        return weights
    # 柔化宽度至少 1m，防止极小测试网格产生除零或硬锯齿。
    feather = max(1.0, float(feather_m))
    for kind, center_x, center_z, radius, points, clearance in areas:
        if kind == "circle":
            # 正值位于圆内，负值位于圆外，可直接进入统一平滑函数。
            signed_distance = radius - np.hypot(x_grid - center_x, z_grid - center_z)
            weights = np.maximum(weights, _smooth_risk_weight(signed_distance, feather))
            continue
        point_array = np.asarray(points, dtype=np.float64)
        margin = clearance + feather
        # 先按包围盒裁出候选网格，避免长地形对每条多边形边做全图距离计算。
        candidate = (
            (x_grid >= float(point_array[:, 0].min()) - margin)
            & (x_grid <= float(point_array[:, 0].max()) + margin)
            & (z_grid >= float(point_array[:, 1].min()) - margin)
            & (z_grid <= float(point_array[:, 1].max()) + margin)
        )
        if not bool(np.any(candidate)):
            continue
        candidate_x = x_grid[candidate].astype(np.float64)
        candidate_z = z_grid[candidate].astype(np.float64)
        # 奇偶规则提供内外符号，最近边距离提供安全间距与柔化尺度。
        inside = _points_inside_polygon(candidate_x, candidate_z, point_array)
        edge_distance = _minimum_polygon_edge_distance(candidate_x, candidate_z, point_array)
        signed_distance = np.where(inside, edge_distance, -edge_distance) + clearance
        # 多障碍重叠时取最大权重，色彩不会因重复叠加越来越红。
        weights[candidate] = np.maximum(weights[candidate], _smooth_risk_weight(signed_distance, feather))
    return weights


def _points_inside_polygon(x_values: np.ndarray, z_values: np.ndarray, points: np.ndarray) -> np.ndarray:
    """批量判断点是否位于多边形内。注意：使用奇偶规则兼容凹多边形。"""

    inside = np.zeros(x_values.shape, dtype=bool)
    previous = points[-1]
    for current in points:
        z_delta = float(previous[1] - current[1])
        # 水平边不会跨过水平射线，跳过还能避免零除。
        if abs(z_delta) > 1e-12:
            crosses = (current[1] > z_values) != (previous[1] > z_values)
            boundary_x = (previous[0] - current[0]) * (z_values - current[1]) / z_delta + current[0]
            # 每穿过一条边翻转一次，最终奇数次即位于多边形内部。
            inside ^= crosses & (x_values < boundary_x)
        previous = current
    return inside


def _minimum_polygon_edge_distance(x_values: np.ndarray, z_values: np.ndarray, points: np.ndarray) -> np.ndarray:
    """计算点到多边形最近边的距离。注意：同时用于安全间距膨胀与边缘柔化。"""

    minimum_sq = np.full(x_values.shape, np.inf, dtype=np.float64)
    previous = points[-1]
    for current in points:
        edge_x = float(current[0] - previous[0])
        edge_z = float(current[1] - previous[1])
        length_sq = edge_x * edge_x + edge_z * edge_z
        if length_sq <= 1e-12:
            # 重复顶点形成的退化边没有距离贡献。
            previous = current
            continue
        # 投影比例夹到 [0,1] 后，同一公式同时覆盖线段内部与两个端点。
        ratio = np.clip(((x_values - previous[0]) * edge_x + (z_values - previous[1]) * edge_z) / length_sq, 0.0, 1.0)
        nearest_x = previous[0] + ratio * edge_x
        nearest_z = previous[1] + ratio * edge_z
        distance_sq = (x_values - nearest_x) ** 2 + (z_values - nearest_z) ** 2
        minimum_sq = np.minimum(minimum_sq, distance_sq)
        previous = current
    return np.sqrt(minimum_sq)


def _smooth_risk_weight(signed_distance: np.ndarray, feather_m: float) -> np.ndarray:
    """把有符号边界距离映射成平滑权重。注意：障碍内部为 1，外部逐渐衰减到 0。"""

    # smoothstep 比线性过渡更柔和，且在两端导数为零，不会形成明显色带。
    ratio = np.clip((signed_distance + feather_m) / (2.0 * feather_m), 0.0, 1.0)
    return (ratio * ratio * (3.0 - 2.0 * ratio)).astype(np.float32)


def _tint_risk_colors(colors: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """批量叠加低饱和风险色。注意：输入输出均为线性色彩，保持材质光照正确。"""

    # TerrainField 颜色在线性空间，目标色也必须先从 sRGB 转换再混合。
    target = np.asarray([_srgb_to_linear(component) for component in _RISK_TINT_SRGB], dtype=np.float32)
    # 0.30 上限只产生色相提示，仍让高度分层、阴影和岩面纹理占主导。
    blend = np.clip(weights * _RISK_TINT_STRENGTH, 0.0, 1.0)[..., None]
    return (colors * (1.0 - blend) + target * blend).astype(np.float32)


def _tint_risk_color(color: tuple[float, float, float], weight: float) -> tuple[float, float, float]:
    """为单个占位地形顶点叠加风险色。注意：与布局地形使用相同参数。"""

    # 标量路径只服务旧 procedural 网格，参数必须与 numpy 批量路径完全一致。
    blend = max(0.0, min(1.0, weight * _RISK_TINT_STRENGTH))
    target = tuple(_srgb_to_linear(component) for component in _RISK_TINT_SRGB)
    return tuple(base * (1.0 - blend) + risk * blend for base, risk in zip(color, target))


def _height_value(x: float, z: float, width: float, depth: float, amplitude: float) -> float:
    """计算连续地形高度。注意：高斯山脉 + 中频起伏，中心航迹区保持平坦。"""

    nx = x / width
    nz = z / depth
    # 中频起伏填满山体之间的空地，幅度约占 amplitude 的两成。
    rolling = (
        0.07 * math.sin(nx * math.tau * 2.6 + 0.4)
        + 0.07 * math.cos(nz * math.tau * 2.2 - 0.7)
        + 0.05 * math.sin((nx + nz) * math.tau * 3.4 + 1.3)
        + 0.03 * math.sin(nx * math.tau * 6.8) * math.cos(nz * math.tau * 5.9)
    )
    # 山体布局跟随地图尺寸整体缩放，保持基准构图不变。
    scale_x = width / _HILL_LAYOUT_SPAN_M
    scale_z = depth / _HILL_LAYOUT_SPAN_M
    hill_sum = 0.0
    for center_x, center_z, radius_x, radius_z, angle_deg, weight in _HILL_PROFILES:
        hill_sum += weight * _elliptic_hill(
            x,
            z,
            center_x * scale_x,
            center_z * scale_z,
            radius_x * scale_x,
            radius_z * scale_z,
            angle_deg,
        )
    # 高频细节按山体质量调制：平原保持干净，山坡出现沟脊棱线。
    ridge = math.sin(nx * math.tau * 11.0 + 2.0) * math.cos(nz * math.tau * 9.0 - 1.0)
    height_mix = rolling + hill_sum * (1.0 + 0.15 * ridge)
    # 谷地允许低于基准面形成沟壑，但限制深度避免出现深坑。
    height_mix = max(height_mix, -0.06)
    # 这里只输出几何高度，颜色由顶点色按高度渐变承担。
    return 4.0 + amplitude * _edge_falloff(nx, nz) * _center_clearance(x, z, width, depth) * height_mix


def _elliptic_hill(
    x: float,
    z: float,
    center_x: float,
    center_z: float,
    radius_x: float,
    radius_z: float,
    angle_deg: float,
) -> float:
    """返回米制旋转椭圆高斯山体权重。注意：高斯裙边互相叠加形成连续山脉。"""

    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = x - center_x
    dz = z - center_z
    # 先旋转到山体局部坐标，再按长短半轴归一化距离。
    local_x = (dx * cos_a + dz * sin_a) / radius_x
    local_z = (-dx * sin_a + dz * cos_a) / radius_z
    distance_sq = local_x * local_x + local_z * local_z
    # 高斯核在半轴处衰减到约四分之一，山脚自然融入起伏。
    return math.exp(-1.4 * distance_sq)


def _center_clearance(x: float, z: float, width: float, depth: float) -> float:
    """返回中心保护区系数。注意：航迹集中在场景中心，山体必须让出净空。"""

    span = min(width, depth)
    clear_radius = span * _CLEAR_RADIUS_RATIO
    blend_radius = span * _CLEAR_BLEND_RATIO
    distance = math.hypot(x, z)
    if distance <= clear_radius:
        return 0.0
    if distance >= blend_radius:
        return 1.0
    ratio = (distance - clear_radius) / (blend_radius - clear_radius)
    # smoothstep 让保护区边缘的坡度连续，不出现环形折痕。
    return ratio * ratio * (3.0 - 2.0 * ratio)


def _edge_falloff(nx: float, nz: float) -> float:
    """返回地形边缘衰减系数。注意：避免山体在边界突然截断。"""

    # 0.16 的归一化边距约等于地形短边三分之一的缓冲带。
    margin = 0.16
    edge_x = max(0.0, min(1.0, (0.5 - abs(nx)) / margin))
    edge_z = max(0.0, min(1.0, (0.5 - abs(nz)) / margin))
    edge = min(edge_x, edge_z)
    # smoothstep 保证边缘高度和一阶变化都更平滑。
    return edge * edge * (3.0 - 2.0 * edge)


def _height_color(height: float, amplitude: float) -> tuple[float, float, float]:
    """按海拔返回顶点色。注意：单调渐变不含噪声，避免历史上的碎斑问题。"""

    normalized = max(0.0, min(1.0, height / max(amplitude, 1.0)))
    if normalized < _COLOR_SPLIT:
        mixed = _lerp_color(_COLOR_LOW, _COLOR_MID, normalized / _COLOR_SPLIT)
    else:
        mixed = _lerp_color(_COLOR_MID, _COLOR_HIGH, min(1.0, (normalized - _COLOR_SPLIT) / (1.0 - _COLOR_SPLIT)))
    # Quick3D 光照在线性空间进行,sRGB 调色板必须先转线性,否则整体被洗白。
    return (_srgb_to_linear(mixed[0]), _srgb_to_linear(mixed[1]), _srgb_to_linear(mixed[2]))


def _srgb_to_linear(component: float) -> float:
    """把 sRGB 分量转换为线性空间。注意：输入输出都在 0 到 1 区间。"""

    if component <= 0.04045:
        return component / 12.92
    return ((component + 0.055) / 1.055) ** 2.4


def _lerp_color(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    mix: float,
) -> tuple[float, float, float]:
    """线性插值颜色。注意：输入输出都是 0 到 1 的 RGB 分量。"""

    return (
        start[0] + (end[0] - start[0]) * mix,
        start[1] + (end[1] - start[1]) * mix,
        start[2] + (end[2] - start[2]) * mix,
    )
