"""Qt Quick 3D 连续线带几何。注意：尾迹流增量更新，航线等普通路径保持兼容。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from itertools import islice
import json
import math
import struct

from PySide6.QtCore import QByteArray, Property, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry

_FLOAT_SIZE = 4
_RIBBON_COMPONENTS = 12
_RIBBON_STRIDE = _RIBBON_COMPONENTS * _FLOAT_SIZE
_STREAM_VERTICES_PER_SEGMENT = 5
_STREAM_INDICES_PER_SEGMENT = 9
_STREAM_SEGMENT_VERTEX_BYTES = _STREAM_VERTICES_PER_SEGMENT * _RIBBON_STRIDE
_STREAM_SEGMENT_INDEX_BYTES = _STREAM_INDICES_PER_SEGMENT * 4
_MIN_STREAM_SEGMENT_CAPACITY = 64
_MITER_LIMIT = 2.0
_STREAM_FADE_SEGMENTS = 16
_TRAIL_ALPHA_MIN = 0.08
_TRAIL_ALPHA_MIDDLE = 0.20
_TRAIL_ALPHA_MAX = 0.72

Point3D = tuple[float, float, float]

# 流式几何设计约束：
# 1. 普通 JSON 点数组继续服务航线、风险线等静态几何，不能被尾迹协议破坏。
# 2. JSON 对象表示尾迹消息，reset 携带全量点，delta 只携带新增尾部点。
# 3. generation 区分两轮仿真；换代必须重建，禁止把新一轮路径接在旧路径后。
# 4. firstSequence/endSequence 使用左闭右开区间，差值必须等于接收端当前点数。
# 5. removedCount 只允许从队首删除，协议不支持在队列中间修改或从尾部回退。
# 6. 窗口数据桥保证正常 delta 连续；几何仍会拒绝游标断裂的消息，避免错误拼接。
# 7. 同代 reset 会比较稳定前后缀，测试或无状态调用也能自动转成增量操作。
# 8. 每架飞机始终只有一个 QQuick3DGeometry，不按历史线段创建 Model 或材质对象。
# 9. 流缓冲从 64 段起按二次幂扩容，避免一开始按 32768 点硬上限预占显存。
# 10. 扩容只发生在跨越容量边界时，因此完整上传次数随点数按对数增长。
# 11. 容量增长后不主动缩容，防止尾迹点数在时间窗边界附近来回抖动造成重分配。
# 12. 一个物理段槽固定占五个顶点：起点左右、终点左右和 bevel 中心点。
# 13. 一个物理段槽固定占九个索引：六个主体索引和三个可退化接头索引。
# 14. 未使用槽的索引全部为零，形成退化三角形，不会显示残留面。
# 15. 段槽可以按任意物理顺序组成同一 mesh，三角形索引显式描述逻辑连接关系。
# 16. 增量写入临时缓冲时，主体索引仍必须使用最终物理槽的全局顶点基址。
# 17. 点队列和逻辑段槽都使用 deque，稳态弹头通过 popleft 保持 O(1)。
# 18. 被淘汰的物理槽放回空闲队首，使下一段优先复用刚释放的环形空间。
# 19. 中间段的物理槽不会因队首淘汰而搬移，历史位置字节因此保持稳定。
# 20. 新点只创建一个新段；前一段仅在它从尾端变成折角时更新一次末端接头。
# 21. 新首段仅恢复自己的平头端帽，不会重算后续中间段。
# 22. setVertexData(offset, data) 只能覆盖已分配范围，不能在缓冲尾部自动扩容。
# 23. 因此局部写入前必须已有固定容量；容量不足时才走一次完整扩容重建。
# 24. setIndexData 也遵循同一覆盖规则，释放槽时只把对应九个索引清零。
# 25. 流式带面宽度仅在 Quick3D 的 XZ 水平面展开，高度 Y 不参与带宽方向。
# 26. 每段自己的法向生成主体四边形，折角处再决定是否共享斜接边缘。
# 27. 合法 miter 的长度不得超过半宽乘 _MITER_LIMIT，避免锐角产生长尖刺。
# 28. 超限、近乎掉头或分母退化时使用 bevel，两段各保留自己的法向端点。
# 29. bevel 的第五个中心顶点只填补外侧缺口，不会把整个折角拉成宽三角带。
# 30. 材质关闭背面剔除，因此 bevel 填充三角形不依赖转向后的顶点绕序。
# 31. miter 接头的三个附加索引保持退化，主体两段在相同边缘位置自然闭合。
# 32. 纯竖直段在 XZ 平面没有方向时使用 +Z 兜底，确保顶点数据仍为有限值。
# 33. 尾迹不能恢复整条 Chaikin 平滑；那会在每次追加后移动所有历史折角。
# 34. 数据桥也不能恢复均匀重抽样；总点数变化会重新选择旧点并造成漂移。
# 35. 流式 alpha 使用双端渐隐：最老端最淡，中段低亮，最新端最清晰。
# 36. 长尾迹中段固定为较低 alpha，避免数万段历史轨迹形成整条高亮飘带。
# 37. 首尾各最多十七段参与渐变，每帧颜色更新数量与历史总长度无关。
# 38. alpha 通过直达颜色分量的四字节 offset 更新，不重传对应顶点位置。
# 39. 短于两倍渐隐窗口的路径直接做单调渐变，避免首尾窗口重叠后亮度反转。
# 40. solid 模式始终输出 alpha=1，确保航线虚线和风险线不会被尾迹渐隐影响。
# 41. 包围盒首次 reset、换代、扩容或样式重建时按当前全量点计算。
# 42. 稳态追加只用 addedPoints 扩张包围盒，不扫描 deque，也不创建三轴临时列表。
# 43. 队首淘汰暂不收缩包围盒；保守偏大只影响裁剪效率，不会错误裁掉可见尾迹。
# 44. 新 generation 会重新计算包围盒，因此保守范围不会跨两轮仿真无限累积。
# 45. 宽度变化需要重新计算所有边缘点和 margin，允许对当前有界队列完整重建。
# 46. alphaMode 变化同样需要重写顶点色，但不会改变流 generation 和逻辑游标。
# 47. 地形后台完成或机型切换可能重推同一快照，空 delta 必须完全跳过 GPU 写入。
# 48. QML 删除不足两点的 delegate 后，窗口数据桥会遗忘游标，下次出现重新 reset。
# 49. 无效 delta 不具备恢复全量历史所需的数据，因此宁可忽略也不能猜测或错接。
# 50. fullRebuildCount 和 incrementalUpdateCount 只用于回归诊断，不参与 QML 渲染逻辑。
# 51. bevelJoinCount 由当前活动槽映射计算，弹头和槽复用时必须同步删除旧标记。
# 52. clear() 只允许出现在完整重建路径；稳态 delta 调用 clear 会抹掉预分配缓冲。
# 53. 每批 delta 最后只调用一次 update()，避免一个新增点触发多次场景图同步。
# 54. CPU 侧仍保留当前有界点 deque，用于扩容恢复和局部折角计算，不复制到每帧 payload。
# 55. 所有位置使用 scene_data 已映射的 [east, up, -north]，本类不再解释 ENU 语义。
# 56. 普通数组兼容分支保留原双顶点布局和全长 alpha，既有航线尺寸测试继续有效。
# 57. 流分支与普通分支共用 48 字节顶点结构，QML 材质无需根据协议切换属性布局。
# 58. 顶点法向固定向上，线带不承担真实气动表面光照，只需保持可读的地面投影效果。
# 59. 纹理坐标在每个物理段内从 0 到 1，槽复用不会依赖全路径归一化而移动旧 UV。
# 60. 以上不变量优先保证热路径有界、历史位置稳定和单机单 mesh 三项目标同时成立。
# 61. reset 的一次性全量成本与当前有界队列长度成正比，不能被误判为每帧热路径成本。
# 62. delta 的 JSON 解析成本只与 removedCount 字段和 addedPoints 数量相关。
# 63. 多帧跳跃允许一次删除和追加多个点，循环次数与真实变化量而不是存量相关。
# 64. 若一帧跨过全部旧窗口，几何可清空活动槽后从同一预分配缓冲重新追加新窗口。
# 65. 队列序号连续性由 TrailBuffer 保证，本类不从浮点坐标反推点身份。
# 66. 坐标相同的悬停点仍是不同序号，不能因去重而破坏后续 removedCount 对齐。
# 67. 路径中段坐标若在同代 reset 中被篡改，稳定前缀检查会强制有界重建。
# 68. 普通数组没有序号元数据，因此每次变化都按兼容全量路径处理。
# 69. 物理槽索引和逻辑时间顺序是两个概念，任何代码都不能用槽号推断新旧关系。
# 70. alpha 的逻辑顺序来自 deque 迭代位置，槽复用后仍能正确刷新最新端渐变。
# 71. 保守包围盒必须始终包含半宽 margin，否则相机边缘会提前裁掉线带外侧。
# 72. 高度方向只外扩两米，因为 ribbon 本身不在 Y 方向产生几何厚度。
# 73. 流式增量不会调用 json.dumps；序列化职责留在 scene_data 与桥接层。
# 74. 诊断计数不做 Signal 通知，避免测试属性给场景图增加额外绑定工作。
# 75. 任何后续优化都必须同时通过槽复用索引、弹头 deque 和无全量 bounds 扫描测试。


@dataclass(frozen=True)
class _TrailMessage:
    """一次原子尾迹消息。注意：reset 的 points 是全量，delta 的 points 仅是新增尾部。"""

    op: str
    generation: int
    first_sequence: int
    end_sequence: int
    removed_count: int
    points: tuple[Point3D, ...]


class TrailRibbonGeometry(QQuick3DGeometry):
    """单条连续线带。注意：流式尾迹使用固定槽位，普通点数组仍按旧契约完整构建。"""

    pathValueChanged = Signal()
    widthValueChanged = Signal()
    alphaModeChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化空几何与流游标。注意：流缓冲按需以二次幂扩容，不预占最大队列容量。"""

        super().__init__(parent)
        self._path_value = "[]"
        self._width_value = 44.0
        self._alpha_mode = "trail"
        self._stream_generation: int | None = None
        self._stream_first_sequence = 0
        self._stream_end_sequence = 0
        self._stream_points: deque[Point3D] = deque()
        self._stream_segment_slots: deque[int] = deque()
        self._free_segment_slots: deque[int] = deque()
        self._stream_segment_capacity = 0
        self._segment_join_bevel: dict[int, bool] = {}
        self._segment_alpha: dict[int, float] = {}
        self._full_rebuild_count = 0
        self._incremental_update_count = 0
        # 构造期先声明一份合法空几何，QML 即使早于首帧读取也不会看到未初始化属性。
        self._rebuild_legacy([])

    @Property(str, notify=pathValueChanged)
    def pathValue(self) -> str:
        """返回 JSON 编码的普通点列或尾迹 reset/delta 消息。"""

        return self._path_value

    @pathValue.setter
    def pathValue(self, value: str) -> None:
        """消费路径消息。注意：非法 JSON 按空普通路径处理，不保留旧残影。"""

        normalized = value if isinstance(value, str) else "[]"
        if normalized == self._path_value:
            return
        self._path_value = normalized
        # 一次字符串赋值对应一条原子消息，避免 generation 与点列经多个 QML role 分步到达。
        points, message = self._parse_path_value(normalized)
        if message is None:
            self._rebuild_legacy(points)
        else:
            self._consume_trail_message(message)
        self.pathValueChanged.emit()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回线带宽度，单位为显示层米。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """更新线带宽度。注意：宽度变化会有界重建当前一机 mesh。"""

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
        self._rebuild_for_style_change()
        self.widthValueChanged.emit()

    @Property(str, notify=alphaModeChanged)
    def alphaMode(self) -> str:
        """返回顶点透明度模式。注意：trail 使用尾迹透明度，solid 供航线等宽线使用。"""

        return self._alpha_mode

    @alphaMode.setter
    def alphaMode(self, value: str) -> None:
        """更新透明度模式。注意：非法值退回 trail，兼容历史 QML 调用。"""

        normalized = value if value in {"trail", "solid"} else "trail"
        if normalized == self._alpha_mode:
            return
        self._alpha_mode = normalized
        self._rebuild_for_style_change()
        self.alphaModeChanged.emit()

    @property
    def fullRebuildCount(self) -> int:
        """返回完整缓冲重建次数，供回归测试和性能诊断使用。"""

        return self._full_rebuild_count

    @property
    def incrementalUpdateCount(self) -> int:
        """返回成功消费的局部尾迹更新次数。"""

        return self._incremental_update_count

    @property
    def bevelJoinCount(self) -> int:
        """返回当前 mesh 中因超过斜接上限而退化的 bevel 接头数量。"""

        return sum(self._segment_join_bevel.values())

    def _rebuild_for_style_change(self) -> None:
        """按当前数据重建样式相关顶点。注意：位置流游标保持不变。"""

        if self._stream_generation is not None:
            self._rebuild_stream(
                list(self._stream_points),
                self._stream_generation,
                self._stream_first_sequence,
                self._stream_end_sequence,
                preserve_capacity=True,
            )
            return
        points, _ = self._parse_path_value(self._path_value)
        self._rebuild_legacy(points)

    def _parse_path_value(self, value: str) -> tuple[list[Point3D], _TrailMessage | None]:
        """解析普通点列或流消息。注意：对象消息的游标与点数不一致时仍由消费层拒绝。"""

        try:
            raw_value = json.loads(value)
        except (json.JSONDecodeError, TypeError):
            return [], None
        if isinstance(raw_value, dict):
            # 对象协议只给正式尾迹使用；普通数组永远走无状态兼容路径。
            return self._parse_trail_message(raw_value)
        return self._normalize_points(raw_value), None

    def _parse_trail_message(self, raw_value: dict) -> tuple[list[Point3D], _TrailMessage]:
        """解析尾迹对象消息。注意：没有 op 的历史全量对象按 reset 兼容。"""

        op = str(raw_value.get("op", "reset"))
        # delta 的 addedPoints 与 reset 的 points 含义不同，不能混用默认字段。
        raw_points = raw_value.get("addedPoints", []) if op == "delta" else raw_value.get("points", [])
        points = self._normalize_points(raw_points)
        generation = self._non_negative_int(raw_value.get("generation"), 0)
        first_sequence = self._non_negative_int(raw_value.get("firstSequence"), 0)
        default_end = first_sequence + len(points)
        end_sequence = self._non_negative_int(raw_value.get("endSequence"), default_end)
        removed_count = self._non_negative_int(raw_value.get("removedCount"), 0)
        message = _TrailMessage(
            op="delta" if op == "delta" else "reset",
            generation=generation,
            first_sequence=first_sequence,
            end_sequence=end_sequence,
            removed_count=removed_count,
            points=tuple(points),
        )
        return points, message

    @staticmethod
    def _non_negative_int(value: object, fallback: int) -> int:
        """把消息字段归一化为非负整数。注意：非法值使用调用方提供的安全回退。"""

        try:
            normalized = int(value)
        except (TypeError, ValueError):
            return fallback
        return normalized if normalized >= 0 else fallback

    @staticmethod
    def _normalize_points(raw_points: object) -> list[Point3D]:
        """归一化 Quick3D 三元组。注意：非法、非有限点会被跳过。"""

        points: list[Point3D] = []
        if not isinstance(raw_points, list):
            return points
        for item in raw_points:
            if not isinstance(item, list | tuple) or len(item) != 3:
                continue
            try:
                x_coord, y_coord, z_coord = (float(item[0]), float(item[1]), float(item[2]))
            except (TypeError, ValueError):
                continue
            if math.isfinite(x_coord) and math.isfinite(y_coord) and math.isfinite(z_coord):
                points.append((x_coord, y_coord, z_coord))
        return points

    def _consume_trail_message(self, message: _TrailMessage) -> None:
        """消费 reset 或 delta。注意：纯追加和弹头走槽位局部写入，其余情况有界重建。"""

        if message.op == "delta":
            # 紧凑包不含存量坐标，必须先严格校验接收端游标才能消费。
            self._consume_delta(message)
            return
        self._consume_reset(message)

    def _consume_reset(self, message: _TrailMessage) -> None:
        """消费全量 reset。注意：同代稳定前后缀仍会自动降为局部更新。"""

        if self._stream_generation == message.generation:
            # 无状态 reset 仍可通过旧后缀==新前缀识别为“弹头+追加”。
            evicted = message.first_sequence - self._stream_first_sequence
            appended = message.end_sequence - self._stream_end_sequence
            survivors = len(self._stream_points) - evicted
            surviving_points = tuple(islice(self._stream_points, max(0, evicted), None))
            can_apply = (
                0 <= evicted <= len(self._stream_points)
                and appended >= 0
                and message.first_sequence <= self._stream_end_sequence
                and message.end_sequence - message.first_sequence == len(message.points)
                and survivors >= 0
                and surviving_points == message.points[:survivors]
                and len(message.points) - survivors == appended
            )
            if can_apply:
                self._apply_stream_delta(
                    evicted,
                    list(message.points[survivors:]),
                    message.first_sequence,
                    message.end_sequence,
                )
                return
        self._rebuild_stream(
            list(message.points),
            message.generation,
            message.first_sequence,
            message.end_sequence,
            preserve_capacity=False,
        )

    def _consume_delta(self, message: _TrailMessage) -> None:
        """消费紧凑 delta。注意：游标不连续时忽略该包，防止把错误增量接到旧 mesh。"""

        expected_first = self._stream_first_sequence + message.removed_count
        expected_end = self._stream_end_sequence + len(message.points)
        # 两个等式分别约束队首删除和队尾追加，最后一个等式再核对目标窗口长度。
        valid = (
            self._stream_generation == message.generation
            and message.removed_count <= len(self._stream_points)
            and message.first_sequence == expected_first
            and message.end_sequence == expected_end
            and message.end_sequence - message.first_sequence
            == len(self._stream_points) - message.removed_count + len(message.points)
        )
        if not valid:
            return
        self._apply_stream_delta(
            message.removed_count,
            list(message.points),
            message.first_sequence,
            message.end_sequence,
        )

    def _apply_stream_delta(
        self,
        evicted_count: int,
        appended_points: list[Point3D],
        first_sequence: int,
        end_sequence: int,
    ) -> None:
        """局部应用队首淘汰与队尾追加。注意：容量不足时仅执行对数级偶发扩容重建。"""

        resulting_length = len(self._stream_points) - evicted_count + len(appended_points)
        required_segments = max(0, resulting_length - 1)
        # 点数 N 对应 N-1 个物理段；单点状态不占段槽，也不会产生可见三角形。
        if required_segments > self._stream_segment_capacity:
            # 只有跨越 64/128/... 容量边界时才物化全量点列，稳态 delta 不走这里。
            resulting_points = list(islice(self._stream_points, evicted_count, None)) + appended_points
            self._rebuild_stream(
                resulting_points,
                int(self._stream_generation or 0),
                first_sequence,
                end_sequence,
                preserve_capacity=True,
            )
            return
        if evicted_count == 0 and not appended_points:
            # 地形后台重推或机型切换会产生空 delta；只推进游标，不触碰 GPU 缓冲。
            self._stream_first_sequence = first_sequence
            self._stream_end_sequence = end_sequence
            return
        self._evict_stream_head(evicted_count)
        # 先删后加与共享 TrailBuffer 的队列语义一致，也使刚释放槽可立即复用。
        for point in appended_points:
            self._append_stream_point(point)
        self._refresh_head_fade()
        self._stream_first_sequence = first_sequence
        self._stream_end_sequence = end_sequence
        self._incremental_update_count += 1
        # 弹头不收缩包围盒仍然是安全的；新增点只做 O(delta) 扩张，避免每帧扫描历史队列。
        self._expand_stream_bounds(appended_points)
        self.update()

    def _rebuild_stream(
        self,
        points: list[Point3D],
        generation: int,
        first_sequence: int,
        end_sequence: int,
        *,
        preserve_capacity: bool,
    ) -> None:
        """完整构建单机流 mesh。注意：只在首次、换代、样式变化、异常结构或扩容时调用。"""

        required_segments = max(1, len(points) - 1)
        previous_capacity = self._stream_segment_capacity if preserve_capacity else 0
        capacity = max(previous_capacity, self._next_stream_capacity(required_segments))
        # 完整重建会一次性建立 CPU 字节数组，避免对大 reset 做成千上万次 offset 调用。
        self._stream_generation = generation
        self._stream_first_sequence = first_sequence
        self._stream_end_sequence = end_sequence
        self._stream_points = deque(points)
        self._stream_segment_capacity = capacity
        self._stream_segment_slots = deque(range(max(0, len(points) - 1)))
        self._free_segment_slots = deque(range(len(self._stream_segment_slots), capacity))
        self._segment_join_bevel = {}
        self._segment_alpha = {}
        self._configure_geometry()
        vertices = bytearray(capacity * _STREAM_SEGMENT_VERTEX_BYTES)
        indices = bytearray(capacity * _STREAM_SEGMENT_INDEX_BYTES)
        # 先写独立主体段，再在第二遍修正相邻段共享的 miter/bevel 接头。
        segment_count = len(self._stream_segment_slots)
        for point_index, slot in enumerate(self._stream_segment_slots):
            alpha = self._stream_segment_alpha(point_index, segment_count)
            self._segment_alpha[slot] = alpha
            self._encode_segment(vertices, indices, slot, points[point_index], points[point_index + 1], alpha)
        for point_index in range(1, len(points) - 1):
            previous_slot = self._stream_segment_slots[point_index - 1]
            current_slot = self._stream_segment_slots[point_index]
            is_bevel = self._encode_joint(
                vertices,
                indices,
                previous_slot,
                current_slot,
                points[point_index - 1],
                points[point_index],
                points[point_index + 1],
                self._segment_alpha[previous_slot],
                self._segment_alpha[current_slot],
            )
            self._segment_join_bevel[current_slot] = is_bevel
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self._apply_bounds(points)
        self._full_rebuild_count += 1
        self.update()

    @staticmethod
    def _next_stream_capacity(required_segments: int) -> int:
        """返回按二次幂增长的段槽容量。注意：避免每追加一个点就重新分配整个缓冲。"""

        capacity = _MIN_STREAM_SEGMENT_CAPACITY
        while capacity < required_segments:
            capacity *= 2
        return capacity

    def _evict_stream_head(self, count: int) -> None:
        """淘汰队首点及对应首段。注意：中间段的物理槽位和顶点字节保持不动。"""

        for _ in range(count):
            # 每弹出一个点恰好失活一个首段；只剩单点时已经没有段槽可回收。
            if not self._stream_points:
                break
            if self._stream_segment_slots:
                slot = self._stream_segment_slots.popleft()
                self._deactivate_segment(slot)
            self._stream_points.popleft()
        if self._stream_segment_slots and len(self._stream_points) >= 2:
            # 新首段不再与已淘汰前驱相接，只局部恢复平头端帽。
            first_slot = self._stream_segment_slots[0]
            self._write_first_segment_cap(first_slot, self._stream_points[0], self._stream_points[1])

    def _append_stream_point(self, point: Point3D) -> None:
        """追加一个队尾点。注意：只写新段，并在必要时修正刚成为折角的前一段尾端。"""

        if not self._stream_points:
            self._stream_points.append(point)
            return
        if not self._free_segment_slots:
            raise RuntimeError("尾迹段槽容量不足，调用方应先扩容重建")
        previous_point = self._stream_points[-1]
        slot = self._free_segment_slots.popleft()
        # 逻辑尾段可落在任意物理槽，后续主体索引必须使用这个 slot 的全局基址。
        self._stream_points.append(point)
        self._stream_segment_slots.append(slot)
        self._segment_alpha[slot] = self._stream_segment_alpha(
            len(self._stream_segment_slots) - 1,
            len(self._stream_segment_slots),
        )
        self._write_new_segment(slot, previous_point, point, self._segment_alpha[slot])
        if len(self._stream_segment_slots) < 2:
            return
        previous_slot = self._stream_segment_slots[-2]
        is_bevel = self._write_joint(
            previous_slot,
            slot,
            self._stream_points[-3],
            previous_point,
            point,
            self._segment_alpha[previous_slot],
            self._segment_alpha[slot],
        )
        self._segment_join_bevel[slot] = is_bevel

    def _deactivate_segment(self, slot: int) -> None:
        """把一个环形段槽退化为空三角形并回收到空闲队列。"""

        offset = slot * _STREAM_SEGMENT_INDEX_BYTES
        self.setIndexData(offset, QByteArray(bytes(_STREAM_SEGMENT_INDEX_BYTES)))
        self._segment_join_bevel.pop(slot, None)
        self._segment_alpha.pop(slot, None)
        # appendleft 让刚弹出的环形头槽优先复用，物理缓冲规模保持稳定。
        self._free_segment_slots.appendleft(slot)

    def _write_new_segment(self, slot: int, start: Point3D, end: Point3D, alpha: float) -> None:
        """把新段写入一个空槽。注意：初始首尾均为平头，下一点到来时才形成接头。"""

        vertices = bytearray(_STREAM_SEGMENT_VERTEX_BYTES)
        indices = bytearray(_STREAM_SEGMENT_INDEX_BYTES)
        self._encode_segment(vertices, indices, 0, start, end, alpha, vertex_slot=slot)
        self.setVertexData(slot * _STREAM_SEGMENT_VERTEX_BYTES, QByteArray(bytes(vertices)))
        self.setIndexData(slot * _STREAM_SEGMENT_INDEX_BYTES, QByteArray(bytes(indices)))
        self._segment_join_bevel[slot] = False

    def _write_first_segment_cap(self, slot: int, start: Point3D, end: Point3D) -> None:
        """恢复新首段起点的平头端帽，并清除其原有 bevel 三角形。"""

        normal = self._segment_normal(start, end)
        left, right = self._edge_positions(start, normal, self._width_value / 2.0)
        alpha = self._segment_alpha.get(slot, _TRAIL_ALPHA_MAX)
        data = self._vertex_record(left, 0.0, 0.0, alpha)
        data += self._vertex_record(right, 0.0, 1.0, alpha)
        self.setVertexData(slot * _STREAM_SEGMENT_VERTEX_BYTES, QByteArray(data))
        center_offset = slot * _STREAM_SEGMENT_VERTEX_BYTES + 4 * _RIBBON_STRIDE
        self.setVertexData(center_offset, QByteArray(self._vertex_record(start, 0.0, 0.5, alpha)))
        join_offset = slot * _STREAM_SEGMENT_INDEX_BYTES + 6 * 4
        self.setIndexData(join_offset, QByteArray(bytes(3 * 4)))
        self._segment_join_bevel[slot] = False

    def _write_joint(
        self,
        previous_slot: int,
        current_slot: int,
        previous: Point3D,
        current: Point3D,
        following: Point3D,
        previous_alpha: float,
        current_alpha: float,
    ) -> bool:
        """局部写入一个 miter/bevel 接头。注意：最多改前一段尾端和新段起端。"""

        layout = self._joint_layout(previous, current, following)
        previous_left, previous_right, current_left, current_right, is_bevel, turn = layout
        previous_data = self._vertex_record(previous_left, 1.0, 0.0, previous_alpha)
        previous_data += self._vertex_record(previous_right, 1.0, 1.0, previous_alpha)
        previous_offset = previous_slot * _STREAM_SEGMENT_VERTEX_BYTES + 2 * _RIBBON_STRIDE
        # 前一段只有两个末端顶点会因新折角到来而改变，其余历史顶点保持原位。
        self.setVertexData(previous_offset, QByteArray(previous_data))
        current_data = self._vertex_record(current_left, 0.0, 0.0, current_alpha)
        current_data += self._vertex_record(current_right, 0.0, 1.0, current_alpha)
        current_offset = current_slot * _STREAM_SEGMENT_VERTEX_BYTES
        self.setVertexData(current_offset, QByteArray(current_data))
        center_offset = current_offset + 4 * _RIBBON_STRIDE
        self.setVertexData(center_offset, QByteArray(self._vertex_record(current, 0.0, 0.5, current_alpha)))
        join_indices = self._join_indices(previous_slot, current_slot, is_bevel, turn)
        join_offset = current_slot * _STREAM_SEGMENT_INDEX_BYTES + 6 * 4
        self.setIndexData(join_offset, QByteArray(struct.pack("<III", *join_indices)))
        return is_bevel

    def _encode_segment(
        self,
        vertices: bytearray,
        indices: bytearray,
        slot: int,
        start: Point3D,
        end: Point3D,
        alpha: float,
        *,
        vertex_slot: int | None = None,
    ) -> None:
        """向完整缓冲编码一个带面段，预留第五个中心顶点供 bevel 接头使用。"""

        normal = self._segment_normal(start, end)
        half_width = self._width_value / 2.0
        start_left, start_right = self._edge_positions(start, normal, half_width)
        end_left, end_right = self._edge_positions(end, normal, half_width)
        records = (
            self._vertex_record(start_left, 0.0, 0.0, alpha),
            self._vertex_record(start_right, 0.0, 1.0, alpha),
            self._vertex_record(end_left, 1.0, 0.0, alpha),
            self._vertex_record(end_right, 1.0, 1.0, alpha),
            self._vertex_record(start, 0.0, 0.5, alpha),
        )
        vertex_offset = slot * _STREAM_SEGMENT_VERTEX_BYTES
        vertices[vertex_offset : vertex_offset + _STREAM_SEGMENT_VERTEX_BYTES] = b"".join(records)
        # 局部临时缓冲使用 slot=0，但索引必须引用最终物理槽，不能错误回指顶点0..4。
        base = (slot if vertex_slot is None else vertex_slot) * _STREAM_VERTICES_PER_SEGMENT
        index_offset = slot * _STREAM_SEGMENT_INDEX_BYTES
        indices[index_offset : index_offset + _STREAM_SEGMENT_INDEX_BYTES] = struct.pack(
            "<IIIIIIIII",
            base,
            base + 2,
            base + 1,
            base + 1,
            base + 2,
            base + 3,
            0,
            0,
            0,
        )

    def _encode_joint(
        self,
        vertices: bytearray,
        indices: bytearray,
        previous_slot: int,
        current_slot: int,
        previous: Point3D,
        current: Point3D,
        following: Point3D,
        previous_alpha: float,
        current_alpha: float,
    ) -> bool:
        """向完整缓冲编码折角。注意：超过 miter limit 时增加一个 bevel 填充三角形。"""

        layout = self._joint_layout(previous, current, following)
        previous_left, previous_right, current_left, current_right, is_bevel, turn = layout
        self._store_vertex(vertices, previous_slot, 2, previous_left, 1.0, 0.0, previous_alpha)
        self._store_vertex(vertices, previous_slot, 3, previous_right, 1.0, 1.0, previous_alpha)
        self._store_vertex(vertices, current_slot, 0, current_left, 0.0, 0.0, current_alpha)
        self._store_vertex(vertices, current_slot, 1, current_right, 0.0, 1.0, current_alpha)
        self._store_vertex(vertices, current_slot, 4, current, 0.0, 0.5, current_alpha)
        join_indices = self._join_indices(previous_slot, current_slot, is_bevel, turn)
        join_offset = current_slot * _STREAM_SEGMENT_INDEX_BYTES + 6 * 4
        indices[join_offset : join_offset + 3 * 4] = struct.pack("<III", *join_indices)
        return is_bevel

    def _joint_layout(
        self,
        previous: Point3D,
        current: Point3D,
        following: Point3D,
    ) -> tuple[Point3D, Point3D, Point3D, Point3D, bool, float]:
        """计算折角边缘。注意：斜接长度超限或近乎掉头时退化为 bevel。"""

        previous_direction = self._segment_direction(previous, current)
        following_direction = self._segment_direction(current, following)
        previous_normal = (-previous_direction[1], previous_direction[0])
        following_normal = (-following_direction[1], following_direction[0])
        miter_x = previous_normal[0] + following_normal[0]
        miter_z = previous_normal[1] + following_normal[1]
        miter_length = math.hypot(miter_x, miter_z)
        half_width = self._width_value / 2.0
        turn = previous_direction[0] * following_direction[1] - previous_direction[1] * following_direction[0]
        if miter_length > 1e-6:
            # 单位斜接向量与后一段法向的点积决定保持固定半宽所需的伸长倍数。
            miter_x /= miter_length
            miter_z /= miter_length
            denominator = miter_x * following_normal[0] + miter_z * following_normal[1]
            scale = half_width / abs(denominator) if abs(denominator) > 1e-6 else math.inf
            if scale <= half_width * _MITER_LIMIT:
                miter_normal = (miter_x, miter_z)
                left, right = self._edge_positions(current, miter_normal, scale)
                return left, right, left, right, False, turn
        # bevel 保留相邻两段各自的法向端点，再用中心三角形填补外侧缺口。
        previous_left, previous_right = self._edge_positions(current, previous_normal, half_width)
        current_left, current_right = self._edge_positions(current, following_normal, half_width)
        return previous_left, previous_right, current_left, current_right, True, turn

    @staticmethod
    def _join_indices(previous_slot: int, current_slot: int, is_bevel: bool, turn: float) -> tuple[int, int, int]:
        """返回接头三角形索引。注意：miter 使用退化索引，不产生额外面。"""

        if not is_bevel:
            return 0, 0, 0
        previous_base = previous_slot * _STREAM_VERTICES_PER_SEGMENT
        current_base = current_slot * _STREAM_VERTICES_PER_SEGMENT
        if turn >= 0.0:
            # 正向转弯的 +normal 一侧在弯内，外侧应连接两段各自的 -normal（left）顶点。
            return previous_base + 2, current_base, current_base + 4
        # 反向转弯则由两段 +normal（right）顶点和中心点补齐外侧楔形。
        return previous_base + 3, current_base + 1, current_base + 4

    def _store_vertex(
        self,
        vertices: bytearray,
        slot: int,
        local_index: int,
        position: Point3D,
        u_coord: float,
        v_coord: float,
        alpha: float,
    ) -> None:
        """把一条顶点记录写入完整 CPU 缓冲的指定槽位。"""

        offset = slot * _STREAM_SEGMENT_VERTEX_BYTES + local_index * _RIBBON_STRIDE
        vertices[offset : offset + _RIBBON_STRIDE] = self._vertex_record(position, u_coord, v_coord, alpha)

    @staticmethod
    def _segment_direction(start: Point3D, end: Point3D) -> tuple[float, float]:
        """返回 XZ 平面的单位方向。注意：纯竖直退化段使用 +Z 方向兜底。"""

        delta_x = end[0] - start[0]
        delta_z = end[2] - start[2]
        length = math.hypot(delta_x, delta_z)
        if length <= 1e-6:
            return 0.0, 1.0
        return delta_x / length, delta_z / length

    @classmethod
    def _segment_normal(cls, start: Point3D, end: Point3D) -> tuple[float, float]:
        """返回 XZ 平面的单位侧向量。"""

        direction_x, direction_z = cls._segment_direction(start, end)
        return -direction_z, direction_x

    @staticmethod
    def _edge_positions(point: Point3D, normal: tuple[float, float], scale: float) -> tuple[Point3D, Point3D]:
        """按给定侧向和长度返回中心线两侧边缘点。"""

        offset_x = normal[0] * scale
        offset_z = normal[1] * scale
        left = (point[0] - offset_x, point[1], point[2] - offset_z)
        right = (point[0] + offset_x, point[1], point[2] + offset_z)
        return left, right

    def _stream_segment_alpha(self, index: int, segment_count: int) -> float:
        """返回双端渐隐值。注意：长尾迹中段保持低亮，避免整条历史成为高亮飘带。"""

        if self._alpha_mode == "solid":
            return 1.0
        if segment_count <= 1:
            return _TRAIL_ALPHA_MAX
        if segment_count <= 2 * _STREAM_FADE_SEGMENTS + 1:
            # 短尾迹没有独立中段，直接在当前全部段上保持单调渐变。
            ratio = index / max(1, segment_count - 1)
            return _TRAIL_ALPHA_MIN + (_TRAIL_ALPHA_MAX - _TRAIL_ALPHA_MIN) * ratio
        if index <= _STREAM_FADE_SEGMENTS:
            # 最老端只升到低亮中段值，避免历史主体和飞机附近一样醒目。
            ratio = index / _STREAM_FADE_SEGMENTS
            return _TRAIL_ALPHA_MIN + (_TRAIL_ALPHA_MIDDLE - _TRAIL_ALPHA_MIN) * ratio
        tail_start = segment_count - 1 - _STREAM_FADE_SEGMENTS
        if index >= tail_start:
            # 最新端从低亮中段平滑提升到最大 alpha，让尾迹头贴近飞机且清晰可见。
            ratio = (index - tail_start) / _STREAM_FADE_SEGMENTS
            return _TRAIL_ALPHA_MIDDLE + (_TRAIL_ALPHA_MAX - _TRAIL_ALPHA_MIDDLE) * ratio
        return _TRAIL_ALPHA_MIDDLE

    def _refresh_head_fade(self) -> None:
        """刷新固定数量的首尾 alpha。注意：只改颜色 float，历史位置和索引完全不动。"""

        segment_count = len(self._stream_segment_slots)
        refresh_count = min(segment_count, _STREAM_FADE_SEGMENTS + 1)
        candidates = list(enumerate(islice(self._stream_segment_slots, refresh_count)))
        # reversed(deque) 从逻辑尾部迭代，不会把整条段槽队列复制成列表。
        tail_candidates = (
            (segment_count - 1 - reverse_index, slot)
            for reverse_index, slot in enumerate(islice(reversed(self._stream_segment_slots), refresh_count))
        )
        candidates.extend(tail_candidates)
        for index, slot in candidates:
            desired = self._stream_segment_alpha(index, segment_count)
            if math.isclose(self._segment_alpha.get(slot, -1.0), desired, abs_tol=1e-7):
                continue
            self._write_segment_alpha(slot, desired)

    def _write_segment_alpha(self, slot: int, alpha: float) -> None:
        """局部更新一个段槽的五个 alpha 分量。注意：offset 直达颜色字段，不重传顶点位置。"""

        alpha_data = QByteArray(struct.pack("<f", alpha))
        # 颜色 alpha 位于每个 48 字节顶点的最后四字节，五次小覆盖不触碰坐标。
        for local_index in range(_STREAM_VERTICES_PER_SEGMENT):
            alpha_offset = (
                slot * _STREAM_SEGMENT_VERTEX_BYTES
                + local_index * _RIBBON_STRIDE
                + 11 * _FLOAT_SIZE
            )
            self.setVertexData(alpha_offset, alpha_data)
        self._segment_alpha[slot] = alpha

    def _expand_stream_bounds(self, points: list[Point3D]) -> None:
        """按新增点扩张保守包围盒。注意：队首淘汰不收缩，以换取稳态 O(delta) 开销。"""

        if not points:
            return
        bounds_min = self.boundsMin()
        bounds_max = self.boundsMax()
        # 读取现有保守范围后只与新增点比较；被删除的极值允许暂时留在范围内。
        margin = self._width_value
        min_x, min_y, min_z = bounds_min.x(), bounds_min.y(), bounds_min.z()
        max_x, max_y, max_z = bounds_max.x(), bounds_max.y(), bounds_max.z()
        for point in points:
            min_x = min(min_x, point[0] - margin)
            min_y = min(min_y, point[1] - 2.0)
            min_z = min(min_z, point[2] - margin)
            max_x = max(max_x, point[0] + margin)
            max_y = max(max_y, point[1] + 2.0)
            max_z = max(max_z, point[2] + margin)
        self.setBounds(QVector3D(min_x, min_y, min_z), QVector3D(max_x, max_y, max_z))

    def _rebuild_legacy(self, points: list[Point3D]) -> None:
        """重建普通数组路径。注意：航线、风险线和兼容调用继续使用紧凑的双顶点折线。"""

        self._reset_stream_state()
        self._configure_geometry()
        if len(points) < 2:
            self.setVertexData(QByteArray())
            self.setIndexData(QByteArray())
            self.setBounds(QVector3D(), QVector3D())
            self._full_rebuild_count += 1
            self.update()
            return
        vertices, indices = self._build_legacy_mesh(points)
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self._apply_bounds(points)
        self._full_rebuild_count += 1
        self.update()

    def _reset_stream_state(self) -> None:
        """清空流式游标和槽位映射，切回普通路径模式。"""

        self._stream_generation = None
        self._stream_first_sequence = 0
        self._stream_end_sequence = 0
        self._stream_points = deque()
        self._stream_segment_slots = deque()
        self._free_segment_slots = deque()
        self._stream_segment_capacity = 0
        self._segment_join_bevel = {}
        self._segment_alpha = {}

    def _configure_geometry(self) -> None:
        """重置并声明共享顶点布局。注意：流式局部更新期间不会再次 clear。"""

        self.clear()
        # 属性声明必须与 QML PrincipledMaterial 的 vertexColorsEnabled 契约保持一致。
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(_RIBBON_STRIDE)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 3 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic, 6 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.ColorSemantic, 8 * _FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)

    def _build_legacy_mesh(self, points: list[Point3D]) -> tuple[bytearray, bytearray]:
        """构建兼容三角带。注意：每个普通路径点生成左右两个顶点。"""

        half_width = self._width_value / 2.0
        vertices = bytearray()
        indices = bytearray()
        last_index = len(points) - 1
        for index, point in enumerate(points):
            side = self._legacy_side_vector(points, index)
            alpha = self._legacy_vertex_alpha(index, last_index)
            u_coord = index / max(1, last_index)
            left = (point[0] - side[0] * half_width, point[1], point[2] - side[2] * half_width)
            right = (point[0] + side[0] * half_width, point[1], point[2] + side[2] * half_width)
            vertices.extend(self._vertex_record(left, u_coord, 0.0, alpha))
            vertices.extend(self._vertex_record(right, u_coord, 1.0, alpha))
        for index in range(last_index):
            left_a = index * 2
            right_a = left_a + 1
            left_b = left_a + 2
            right_b = left_a + 3
            indices.extend(struct.pack("<IIIIII", left_a, left_b, right_a, right_a, left_b, right_b))
        return vertices, indices

    def _legacy_vertex_alpha(self, index: int, last_index: int) -> float:
        """返回普通路径顶点透明度。注意：solid 等透明度，legacy trail 保持历史渐变。"""

        if self._alpha_mode == "solid":
            return 1.0
        return 0.08 + 0.64 * (index / max(1, last_index))

    @staticmethod
    def _legacy_side_vector(points: list[Point3D], index: int) -> Point3D:
        """返回普通路径点的水平侧向单位向量。注意：退化段使用默认横向。"""

        if index == 0:
            previous = points[index]
            current = points[index + 1]
        elif index == len(points) - 1:
            previous = points[index - 1]
            current = points[index]
        else:
            previous = points[index - 1]
            current = points[index + 1]
        delta_x = current[0] - previous[0]
        delta_z = current[2] - previous[2]
        length = math.hypot(delta_x, delta_z)
        if length <= 1e-6:
            return 1.0, 0.0, 0.0
        return -delta_z / length, 0.0, delta_x / length

    @staticmethod
    def _vertex_record(position: Point3D, u_coord: float, v_coord: float, alpha: float) -> bytes:
        """编码一个顶点。注意：法向固定向上，顶点色 alpha 由调用模式决定。"""

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
            alpha,
        )

    def _apply_bounds(self, points: list[Point3D]) -> None:
        """设置几何包围盒。注意：弹头时重算 CPU 包围盒，但不上传中间顶点。"""

        if not points:
            self.setBounds(QVector3D(), QVector3D())
            return
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        z_values = [point[2] for point in points]
        margin = self._width_value
        self.setBounds(
            QVector3D(min(x_values) - margin, min(y_values) - 2.0, min(z_values) - margin),
            QVector3D(max(x_values) + margin, max(y_values) + 2.0, max(z_values) + margin),
        )
