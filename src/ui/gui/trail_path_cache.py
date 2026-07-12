"""二维尾迹的增量分块路径缓存。注意：缓存只保存投影几何，不拥有仿真数据。"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Hashable, Sequence
from dataclasses import dataclass
from heapq import heappop, heappush
from typing import Any

from PySide6.QtCore import QPointF
from PySide6.QtGui import QPainterPath


# 默认每块 128 条原始线段；配合块内 4 段显示预算，最大容量下只需约 1024 段栅格化。
DEFAULT_TRAIL_PATH_CHUNK_SIZE = 128
# 透明度只分少量档位，绘制端即可把任意长尾迹压到固定次数的 drawPath 调用。
DEFAULT_TRAIL_OPACITY_BUCKETS = 8
# 每块显示路径最多四段；分块边界提供局部性，不会随全局尾迹长度重新选点。
DEFAULT_TRAIL_MAX_SEGMENTS_PER_CHUNK = 4
# 小于一米的投影抖动无需成为显示折点；真正拐弯仍会由几何误差优先保留。
DEFAULT_TRAIL_SIMPLIFY_TOLERANCE = 0.75

# 缓存遵守以下不变量：
# 一、_vertices 与共享队列当前窗口一一对应，且顺序始终相同。
# 二、每个块最多拥有 chunk_size 条线段。
# 三、相邻块只共享一个端点，不会重复拥有同一条线段。
# 四、中间块一旦封口，只有投影语义改变才会重建。
# 五、队列弹头只影响当前首块，队列追加只影响当前末块。
# 六、画家平移和缩放不是投影语义，不能进入缓存键。
# 七、批次路径可以重组稳定块，但每帧 drawPath 数量必须有固定上界。
# 八、每块显示锚点只由该块原始点决定，不读取全局点数或其它块内容。
# 九、每块显示线段有硬上限，QPainter 不再栅格化全部原始采样段。


@dataclass(frozen=True)
class TrailPathCacheStats:
    """公开的二维尾迹缓存统计。注意：构建计数为缓存生命周期内累计值。"""

    point_count: int
    chunk_count: int
    chunk_path_builds: int
    batch_path_builds: int
    full_rebuilds: int
    last_appended_points: int
    last_removed_points: int
    last_reused_chunk_paths: int
    max_draw_path_calls: int
    display_segment_count: int


@dataclass(frozen=True)
class TrailRenderBatch:
    """一次批量尾迹绘制的数据。注意：同一批次共用颜色透明度和虚线相位。"""

    path: QPainterPath
    opacity_factor: float
    start_path_distance: float
    segment_count: int


@dataclass
class _ProjectedVertex:
    """缓存中的稳定尾迹顶点。注意：point 始终引用数据层原始点。"""

    point: Any
    key: Hashable
    projected: QPointF
    time: float
    path_distance: float


@dataclass
class _PathChunk:
    """一块连续路径。注意：相邻块共享边界顶点但不重复线段。"""

    uid: int
    vertices: list[_ProjectedVertex]
    display_vertices: list[_ProjectedVertex]
    path: QPainterPath
    bounds: tuple[float, float, float, float]
    version: int = 0

    @property
    def segment_count(self) -> int:
        """返回块内原始线段数。注意：仅用于容量分块，不代表实际栅格化数量。"""

        return max(0, len(self.vertices) - 1)

    @property
    def display_segment_count(self) -> int:
        """返回块内显示线段数。注意：该值受局部简化预算硬约束。"""

        return max(0, len(self.display_vertices) - 1)


def simplify_polyline_indices(
    points: Sequence[QPointF],
    *,
    tolerance: float = DEFAULT_TRAIL_SIMPLIFY_TOLERANCE,
    max_segments: int = DEFAULT_TRAIL_MAX_SEGMENTS_PER_CHUNK,
) -> tuple[int, ...]:
    """返回局部折线的显示锚点索引。注意：始终保留首尾，并优先保留最大几何误差点。"""

    point_count = len(points)
    if point_count <= 0:
        return ()
    if point_count == 1:
        return (0,)

    segment_budget = max(1, int(max_segments))
    vertex_budget = segment_budget + 1
    tolerance_squared = max(0.0, float(tolerance)) ** 2
    selected = {0, point_count - 1}
    # 堆顶保存当前所有候选区间中误差最大的点；负号把 Python 小顶堆转换为最大误差优先。
    candidates: list[tuple[float, int, int, int]] = []
    _push_simplification_candidate(candidates, points, 0, point_count - 1)
    while candidates and len(selected) < vertex_budget:
        negative_error, start_index, end_index, point_index = heappop(candidates)
        error_squared = -negative_error
        if error_squared <= tolerance_squared:
            # 堆顶已低于容差时，其余区间误差只会更小，可直接结束。
            break
        selected.add(point_index)
        _push_simplification_candidate(candidates, points, start_index, point_index)
        _push_simplification_candidate(candidates, points, point_index, end_index)
    return tuple(sorted(selected))


def _push_simplification_candidate(
    candidates: list[tuple[float, int, int, int]],
    points: Sequence[QPointF],
    start_index: int,
    end_index: int,
) -> None:
    """把一个可继续细分的区间压入误差堆。注意：相邻端点之间没有候选点。"""

    if end_index - start_index <= 1:
        return
    point_index, error_squared = _farthest_point_from_segment(points, start_index, end_index)
    # 追加起止索引作为稳定次级排序键，确保相同误差下结果可重复。
    heappush(candidates, (-error_squared, start_index, end_index, point_index))


def _farthest_point_from_segment(
    points: Sequence[QPointF],
    start_index: int,
    end_index: int,
) -> tuple[int, float]:
    """返回区间内距首尾线段最远的点及平方距离。注意：只扫描当前局部块。"""

    farthest_index = start_index + 1
    farthest_error = -1.0
    start = points[start_index]
    end = points[end_index]
    for point_index in range(start_index + 1, end_index):
        error = _point_segment_distance_squared(points[point_index], start, end)
        if error > farthest_error:
            farthest_index = point_index
            farthest_error = error
    return farthest_index, max(0.0, farthest_error)


def _point_segment_distance_squared(point: QPointF, start: QPointF, end: QPointF) -> float:
    """计算点到有限线段的平方距离。注意：退化线段按到端点距离处理。"""

    delta_x = end.x() - start.x()
    delta_y = end.y() - start.y()
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared <= 1e-18:
        offset_x = point.x() - start.x()
        offset_y = point.y() - start.y()
        return offset_x * offset_x + offset_y * offset_y
    projection = (
        (point.x() - start.x()) * delta_x + (point.y() - start.y()) * delta_y
    ) / length_squared
    projection = min(1.0, max(0.0, projection))
    nearest_x = start.x() + projection * delta_x
    nearest_y = start.y() + projection * delta_y
    offset_x = point.x() - nearest_x
    offset_y = point.y() - nearest_y
    return offset_x * offset_x + offset_y * offset_y


def opacity_bucket(age: float, trail_seconds: float, bucket_count: int) -> int:
    """把尾迹年龄映射到透明度档。注意：越界年龄会夹到最旧或最新档。"""

    safe_bucket_count = max(1, int(bucket_count))
    if trail_seconds <= 0.0:
        return 0
    # freshness 为 1 表示最新，0 表示刚好达到尾迹窗口边界。
    freshness = min(1.0, max(0.0, 1.0 - max(0.0, age) / trail_seconds))
    return min(safe_bucket_count - 1, int(freshness * safe_bucket_count))


def stable_trail_point_key(point: Any) -> Hashable:
    """生成尾迹点稳定键。注意：无 point_id 的旧数据也能按值进行增量比对。"""

    # point_id 可区分同一时刻的重复坐标；其余字段则能识别普通列表中的中间改写。
    return (
        getattr(point, "point_id", None),
        float(getattr(point, "time")),
        float(getattr(point, "x")),
        float(getattr(point, "y")),
        float(getattr(point, "altitude")),
        float(getattr(point, "path_distance", 0.0)),
    )


class TrailPathCache:
    """按首删尾增规则维护 QPainterPath 块，并生成固定数量的透明度批次。"""

    def __init__(
        self,
        *,
        chunk_size: int = DEFAULT_TRAIL_PATH_CHUNK_SIZE,
        opacity_buckets: int = DEFAULT_TRAIL_OPACITY_BUCKETS,
        max_segments_per_chunk: int = DEFAULT_TRAIL_MAX_SEGMENTS_PER_CHUNK,
        simplify_tolerance: float = DEFAULT_TRAIL_SIMPLIFY_TOLERANCE,
    ) -> None:
        """初始化空缓存。注意：chunk_size 表示每块原始线段数，不是显示线段数。"""

        self.chunk_size = max(2, int(chunk_size))
        self.opacity_buckets = max(1, int(opacity_buckets))
        self.max_segments_per_chunk = max(1, int(max_segments_per_chunk))
        self.simplify_tolerance = max(0.0, float(simplify_tolerance))
        self._vertices: deque[_ProjectedVertex] = deque()
        self._chunks: deque[_PathChunk] = deque()
        self._projector: Callable[[Any], tuple[float, float]] | None = None
        self._semantic_key: Hashable | None = None
        self._next_chunk_uid = 0
        self._chunk_path_builds = 0
        self._batch_path_builds = 0
        self._full_rebuilds = 0
        self._last_appended_points = 0
        self._last_removed_points = 0
        self._last_reused_chunk_paths = 0
        # 每个透明度档或边界只保留最近一次合成结果，避免历史批次无限占用内存。
        self._batch_cache: dict[Hashable, tuple[Hashable, QPainterPath, int]] = {}
        # 带逻辑序号的 TrailBuffer/TrailSnapshot 可绕过全队列签名扫描。
        # generation 标识配置切换或时间回拨后的新一轮尾迹。
        # revision 标识容器内容是否发生过变化。
        # first_sequence 是当前窗口首点的稳定逻辑序号。
        # end_sequence 是尾后逻辑序号，可直接定位本帧新增区间。
        self._source_generation: Any = None
        self._source_revision: int | None = None
        self._source_first_sequence: int | None = None
        self._source_end_sequence: int | None = None
        # 无 revision 的兼容序列按“长度、首键、尾键”识别未变化帧，避免重复扫描数万点。
        self._plain_sequence_signature: tuple[int, Hashable | None, Hashable | None] | None = None

    @property
    def stats(self) -> TrailPathCacheStats:
        """返回当前缓存统计快照。注意：读取统计不会触发路径构建。"""

        return TrailPathCacheStats(
            point_count=len(self._vertices),
            chunk_count=len(self._chunks),
            chunk_path_builds=self._chunk_path_builds,
            batch_path_builds=self._batch_path_builds,
            full_rebuilds=self._full_rebuilds,
            last_appended_points=self._last_appended_points,
            last_removed_points=self._last_removed_points,
            last_reused_chunk_paths=self._last_reused_chunk_paths,
            max_draw_path_calls=self.opacity_buckets + 2,
            display_segment_count=sum(chunk.display_segment_count for chunk in self._chunks),
        )

    @property
    def chunk_paths(self) -> tuple[QPainterPath, ...]:
        """返回各块路径对象，供性能回归测试核对对象身份。注意：调用方不得修改路径。"""

        return tuple(chunk.path for chunk in self._chunks)

    @property
    def projected_bounds(self) -> tuple[float, float, float, float] | None:
        """返回投影点包围盒。注意：结果坐标仍是未平移、未缩放的投影世界坐标。"""

        if not self._vertices:
            return None
        if not self._chunks:
            point = self._vertices[0].projected
            return point.x(), point.x(), point.y(), point.y()
        # 块数远少于点数，按块归并不会恢复成逐点全表扫描。
        min_x = min(chunk.bounds[0] for chunk in self._chunks)
        max_x = max(chunk.bounds[1] for chunk in self._chunks)
        min_y = min(chunk.bounds[2] for chunk in self._chunks)
        max_y = max(chunk.bounds[3] for chunk in self._chunks)
        return min_x, max_x, min_y, max_y

    def synchronize(
        self,
        points: Sequence[Any],
        *,
        projector: Callable[[Any], tuple[float, float]],
        semantic_key: Hashable,
    ) -> None:
        """同步有序尾迹。注意：正常路径只允许删头、加尾；中间变化会安全地全量重建。"""

        before_path_ids = {id(chunk.path) for chunk in self._chunks}
        self._last_appended_points = 0
        self._last_removed_points = 0
        semantic_changed = self._projector is not None and semantic_key != self._semantic_key
        self._projector = projector
        self._semantic_key = semantic_key

        # 正式快照优先走逻辑序号快速路径。
        # 单元测试和旧调用方的 list 则退回稳定键比对。
        # 两条路径最终都只调用相同的首删、尾增原语。
        metadata = self._sequence_metadata(points)
        if metadata is None:
            did_full_rebuild = self._synchronize_plain_sequence(points)
            # 普通序列没有可靠 revision，清空元数据以免下一次误走快速路径。
            self._clear_source_metadata()
        else:
            self._plain_sequence_signature = None
            did_full_rebuild = self._synchronize_numbered_sequence(points, metadata)

        if semantic_changed and not did_full_rebuild:
            # 投影变化只重建已有块；平移缩放不进入 semantic_key，因而不会到达这里。
            self._reproject_all_vertices()
        self._last_reused_chunk_paths = sum(id(chunk.path) in before_path_ids for chunk in self._chunks)

    def render_batches(self, *, current_time: float, trail_seconds: float) -> tuple[TrailRenderBatch, ...]:
        """生成按透明度合并的绘制批次。注意：返回数量恒不超过透明度档数加二。"""

        if trail_seconds <= 0.0 or len(self._vertices) <= 1:
            return ()
        cutoff_time = current_time - trail_seconds
        visible_chunks: list[tuple[_PathChunk, int]] = []
        # 数据层通常已按时间窗弹头，这里的截止时间是显示端兜底。
        # 有旧点时只会切到第一个可见块内部，不会重采样历史点。
        for chunk in self._chunks:
            first_vertex_index = self._first_visible_vertex_index(chunk, cutoff_time)
            if first_vertex_index is not None:
                visible_chunks.append((chunk, first_vertex_index))
        if not visible_chunks:
            return ()

        batches: list[TrailRenderBatch] = []
        # 首块可能正按时间窗或队列头逐点裁剪，单独画可避免它牵连中间合成路径。
        first_chunk, first_index = visible_chunks[0]
        batches.append(
            self._boundary_batch(
                cache_label="首块",
                chunk=first_chunk,
                first_vertex_index=first_index,
                current_time=current_time,
                trail_seconds=trail_seconds,
            )
        )

        middle_chunks = [chunk for chunk, _ in visible_chunks[1:-1]]
        chunks_by_bucket: dict[int, list[_PathChunk]] = {}
        # 时间有序意味着同档块在历史轴上连续，可合成一条连续路径。
        # 分档只改变整块所属批次，不会改写任何块内顶点。
        for chunk in middle_chunks:
            bucket = opacity_bucket(current_time - chunk.vertices[-1].time, trail_seconds, self.opacity_buckets)
            chunks_by_bucket.setdefault(bucket, []).append(chunk)
        for bucket in sorted(chunks_by_bucket):
            chunks = chunks_by_bucket[bucket]
            # 合成缓存键只含块 uid/version，不含不断前进的 current_time。
            # 只要块没有跨档，连续帧就会复用同一个批次路径对象。
            path = self._combined_chunk_path(("透明度", bucket), chunks)
            batches.append(
                TrailRenderBatch(
                    path=path,
                    opacity_factor=self._opacity_factor(bucket),
                    start_path_distance=chunks[0].display_vertices[0].path_distance,
                    segment_count=sum(chunk.display_segment_count for chunk in chunks),
                )
            )

        # 末块持续接收新点，也独立绘制；只有一个可见块时首末块是同一对象，不重复画。
        if len(visible_chunks) > 1:
            last_chunk, last_index = visible_chunks[-1]
            # 末块相位从它自己的稳定累计里程开始，虚线不会因分批而跳变。
            batches.append(
                self._boundary_batch(
                    cache_label="末块",
                    chunk=last_chunk,
                    first_vertex_index=last_index,
                    current_time=current_time,
                    trail_seconds=trail_seconds,
                )
            )
        return tuple(batches)

    def _sequence_metadata(self, points: Sequence[Any]) -> tuple[Any, int, int, int] | None:
        """读取共享队列逻辑序号。注意：普通 list 没有这些字段时返回 None。"""

        names = ("generation", "revision", "first_sequence", "end_sequence")
        if not all(hasattr(points, name) for name in names):
            return None
        return (
            getattr(points, "generation"),
            int(getattr(points, "revision")),
            int(getattr(points, "first_sequence")),
            int(getattr(points, "end_sequence")),
        )

    def _synchronize_numbered_sequence(
        self,
        points: Sequence[Any],
        metadata: tuple[Any, int, int, int],
    ) -> bool:
        """按逻辑序号增量同步共享队列。注意：复杂度只与本帧首删尾增数量有关。"""

        generation, revision, first_sequence, end_sequence = metadata
        if (
            self._source_revision is None
            or generation != self._source_generation
            or self._source_first_sequence is None
            or self._source_end_sequence is None
            or first_sequence < self._source_first_sequence
            or end_sequence < self._source_end_sequence
        ):
            self._full_rebuild(points)
            self._set_source_metadata(metadata)
            return True
        if revision == self._source_revision:
            # 同一不可变快照可能先用于自适应包围盒、再用于实际绘制。
            # revision 未变时必须保持所有路径对象身份不变。
            return False

        old_first = self._source_first_sequence
        old_end = self._source_end_sequence
        # 新首序号与旧首序号之差就是窗口弹头数量。
        remove_count = min(len(self._vertices), max(0, first_sequence - old_first))
        # 新增区间从旧尾后与新首二者较大处开始，兼容一帧内整个旧窗口被淘汰。
        append_start_sequence = max(old_end, first_sequence)
        # 逻辑序号减去新首序号即可换算为当前快照切片索引。
        append_start_index = max(0, append_start_sequence - first_sequence)
        appended_points = points[append_start_index:]
        # 长度守卫能发现序号跳跃、错误切片或不兼容的第三方 Sequence。
        expected_length = len(self._vertices) - remove_count + len(appended_points)
        if expected_length != len(points):
            # 序号不连续或调用方换了不兼容实现时宁可重建，也不留下错接线段。
            self._full_rebuild(points)
            self._set_source_metadata(metadata)
            return True
        self._remove_head_vertices(remove_count)
        self._append_points(appended_points)
        # 先删除再追加与共享环形窗口的时间顺序一致。
        # 两个计数单独公开，方便性能测试发现意外全量重建。
        self._last_removed_points = remove_count
        self._last_appended_points = len(appended_points)
        self._set_source_metadata(metadata)
        return False

    def _synchronize_plain_sequence(self, points: Sequence[Any]) -> bool:
        """同步普通序列。注意：此兼容路径需扫描稳定键，正式共享队列不走这里。"""

        signature = self._plain_signature(points)
        if signature == self._plain_sequence_signature:
            # 任务约定普通序列也只会首删尾增，因此长度和稳定首尾均未变即可判定内容未变。
            return False
        self._plain_sequence_signature = signature
        new_points = list(points)
        if not self._vertices:
            self._full_rebuild(new_points)
            return True
        if not new_points:
            self._last_removed_points = len(self._vertices)
            self._full_rebuild(())
            return True

        old_keys = [vertex.key for vertex in self._vertices]
        new_keys = [stable_trail_point_key(point) for point in new_points]
        # 首个新键在旧序列中的位置就是普通列表的弹头数量。
        try:
            old_start = old_keys.index(new_keys[0])
        except ValueError:
            self._full_rebuild(new_points)
            return True
        overlap_length = min(len(old_keys) - old_start, len(new_keys))
        # 重叠区逐项相等才能证明中间历史点从未移动或被替换。
        if old_keys[old_start : old_start + overlap_length] != new_keys[:overlap_length]:
            self._full_rebuild(new_points)
            return True
        # 新序列必须完整包含旧序列尚未裁掉的后缀，否则属于中间删除而非滑动窗口。
        if overlap_length != len(old_keys) - old_start:
            self._full_rebuild(new_points)
            return True

        appended_points = new_points[overlap_length:]
        # 普通序列兼容路径同样只把差量交给块维护原语。
        self._remove_head_vertices(old_start)
        self._append_points(appended_points)
        self._last_removed_points = old_start
        self._last_appended_points = len(appended_points)
        return False

    @staticmethod
    def _plain_signature(points: Sequence[Any]) -> tuple[int, Hashable | None, Hashable | None]:
        """返回普通序列的 O(1) 首尾签名。注意：依赖历史点只删头、加尾且中间不改写。"""

        point_count = len(points)
        if point_count <= 0:
            return 0, None, None
        return point_count, stable_trail_point_key(points[0]), stable_trail_point_key(points[-1])

    def _full_rebuild(self, points: Sequence[Any]) -> None:
        """从输入序列重建全部块。注意：只用于首次同步、重置或规则被破坏时。"""

        self._vertices = deque(self._new_vertex(point) for point in points)
        self._chunks.clear()
        vertices = list(self._vertices)
        # 首次构建直接按块切分，每块只创建一次路径，避免逐点反复扩展末块。
        for start in range(0, max(0, len(vertices) - 1), self.chunk_size):
            # 每块含一个额外锚点，相邻块因此共享端点但各自只拥有自己的线段。
            chunk_vertices = vertices[start : min(len(vertices), start + self.chunk_size + 1)]
            self._chunks.append(self._new_chunk(chunk_vertices))
        self._batch_cache.clear()
        # 全量重建后旧批次含有失效块引用，必须一并清空。
        self._full_rebuilds += 1
        self._last_appended_points = len(vertices)
        self._last_removed_points = 0

    def _append_points(self, points: Sequence[Any]) -> None:
        """把新点追加到末块。注意：最多只重建一个不超过 chunk_size 的活动末块。"""

        for point in points:
            vertex = self._new_vertex(point)
            if self._vertices:
                previous = self._vertices[-1]
                if self._chunks and self._chunks[-1].segment_count < self.chunk_size:
                    # 活动末块尚未封口，只重建这一个受局部显示预算约束的路径。
                    self._chunks[-1].vertices.append(vertex)
                    self._rebuild_chunk(self._chunks[-1])
                else:
                    # 旧末块封口后保持不动，新块从共享边界点继续。
                    self._chunks.append(self._new_chunk([previous, vertex]))
            self._vertices.append(vertex)

    def _remove_head_vertices(self, count: int) -> None:
        """从缓存头删除指定数量顶点。注意：完整旧块直接丢弃，仅部分首块需要重建。"""

        remove_count = min(count, len(self._vertices))
        for _ in range(remove_count):
            self._vertices.popleft()
        if not self._vertices:
            # 空缓存不能遗留只含锚点的退化块。
            self._chunks.clear()
            return

        new_first = self._vertices[0]
        while self._chunks:
            first_chunk = self._chunks[0]
            first_index = next(
                (index for index, vertex in enumerate(first_chunk.vertices) if vertex is new_first),
                None,
            )
            if first_index is None:
                # 新首点不在该块中，说明整块及其全部线段都已过期，可直接释放。
                self._chunks.popleft()
                continue
            if first_index == len(first_chunk.vertices) - 1:
                # 新首点恰为旧块末锚点时，下一块也以它开头；丢弃退化旧块即可。
                self._chunks.popleft()
                continue
            if first_index > 0:
                # 最终部分首块只在所有批量删除完成后切片并重建一次。
                first_chunk.vertices = first_chunk.vertices[first_index:]
                self._rebuild_trimmed_head_chunk(first_chunk)
            break

    def _new_vertex(self, point: Any) -> _ProjectedVertex:
        """创建投影顶点。注意：projector 必须在 synchronize 开始时设置。"""

        if self._projector is None:
            raise RuntimeError("尾迹投影器尚未设置")
        x, y = self._projector(point)
        return _ProjectedVertex(
            point=point,
            key=stable_trail_point_key(point),
            projected=QPointF(float(x), float(y)),
            time=float(getattr(point, "time")),
            path_distance=float(getattr(point, "path_distance", 0.0)),
        )

    def _new_chunk(self, vertices: list[_ProjectedVertex]) -> _PathChunk:
        """创建新块并构建一次路径。注意：传入顶点至少应形成一条线段。"""

        chunk = _PathChunk(
            uid=self._next_chunk_uid,
            vertices=vertices,
            display_vertices=[],
            path=QPainterPath(),
            bounds=(0.0, 0.0, 0.0, 0.0),
        )
        self._next_chunk_uid += 1
        self._rebuild_chunk(chunk)
        return chunk

    def _rebuild_chunk(self, chunk: _PathChunk) -> None:
        """重建单块路径和包围盒。注意：调用者应只传活动首尾块或投影失效块。"""

        chunk.display_vertices = self._simplified_vertices(chunk.vertices)
        chunk.path = self._path_from_vertices(chunk.display_vertices)
        xs = [vertex.projected.x() for vertex in chunk.vertices]
        ys = [vertex.projected.y() for vertex in chunk.vertices]
        chunk.bounds = min(xs), max(xs), min(ys), max(ys)
        chunk.version += 1
        self._chunk_path_builds += 1

    def _rebuild_trimmed_head_chunk(self, chunk: _PathChunk) -> None:
        """重建裁剪后的首块。注意：保留尚未过期的既有显示锚点，不重新执行局部选点。"""

        remaining_vertex_ids = {id(vertex) for vertex in chunk.vertices}
        display_vertices = [
            vertex for vertex in chunk.display_vertices if id(vertex) in remaining_vertex_ids
        ]
        if not display_vertices or display_vertices[0] is not chunk.vertices[0]:
            # 新队首成为新的路径起点；其它历史显示锚点保持对象与坐标不变。
            display_vertices.insert(0, chunk.vertices[0])
        if display_vertices[-1] is not chunk.vertices[-1]:
            display_vertices.append(chunk.vertices[-1])
        chunk.display_vertices = display_vertices
        chunk.path = self._path_from_vertices(display_vertices)
        xs = [vertex.projected.x() for vertex in chunk.vertices]
        ys = [vertex.projected.y() for vertex in chunk.vertices]
        chunk.bounds = min(xs), max(xs), min(ys), max(ys)
        chunk.version += 1
        self._chunk_path_builds += 1

    def _reproject_all_vertices(self) -> None:
        """按新语义重投影全部点。注意：用于航段锁定或侧视角改变，不用于屏幕变换。"""

        if self._projector is None:
            return
        for vertex in self._vertices:
            x, y = self._projector(vertex.point)
            vertex.projected = QPointF(float(x), float(y))
        # 顶点对象本身保留，只有各块 QPainterPath 与包围盒换成新投影。
        for chunk in self._chunks:
            self._rebuild_chunk(chunk)
        self._batch_cache.clear()

    def _first_visible_vertex_index(self, chunk: _PathChunk, cutoff_time: float) -> int | None:
        """返回可见路径的锚点索引。注意：线段是否可见由其终点时刻判定。"""

        for endpoint_index in range(1, len(chunk.vertices)):
            if chunk.vertices[endpoint_index].time >= cutoff_time:
                return endpoint_index - 1
        return None

    def _boundary_batch(
        self,
        *,
        cache_label: Hashable,
        chunk: _PathChunk,
        first_vertex_index: int,
        current_time: float,
        trail_seconds: float,
    ) -> TrailRenderBatch:
        """构造活动边界批次。注意：按时间裁剪时只复制边界块的一小段路径。"""

        vertices = chunk.vertices[first_vertex_index:]
        key = (chunk.uid, chunk.version, first_vertex_index)
        if first_vertex_index == 0:
            path = chunk.path
            display_vertices = chunk.display_vertices
            display_segment_count = chunk.display_segment_count
        else:
            path, display_segment_count, display_vertices = self._cached_simplified_path(cache_label, key, vertices)
        bucket = opacity_bucket(current_time - vertices[-1].time, trail_seconds, self.opacity_buckets)
        return TrailRenderBatch(
            path=path,
            opacity_factor=self._opacity_factor(bucket),
            start_path_distance=display_vertices[0].path_distance,
            segment_count=display_segment_count,
        )

    def _combined_chunk_path(self, cache_label: Hashable, chunks: list[_PathChunk]) -> QPainterPath:
        """合并连续的稳定中间块。注意：块集合不变时直接复用上次 QPainterPath。"""

        key = tuple((chunk.uid, chunk.version) for chunk in chunks)
        cached = self._batch_cache.get(cache_label)
        if cached is not None and cached[0] == key:
            # Qt 路径采用隐式共享，直接复用比每帧 addPath 更稳定。
            return cached[1]
        vertices: list[_ProjectedVertex] = []
        for chunk in chunks:
            if not vertices:
                vertices.extend(chunk.display_vertices)
            elif vertices[-1] is chunk.display_vertices[0]:
                # 跳过共享锚点，保证合成后线段数与原块之和严格一致。
                vertices.extend(chunk.display_vertices[1:])
            else:
                # 正常有序队列不会断开；保留锚点可让异常输入至少不丢掉该块。
                vertices.extend(chunk.display_vertices)
        path = self._path_from_vertices(vertices)
        segment_count = sum(chunk.display_segment_count for chunk in chunks)
        self._batch_cache[cache_label] = (key, path, segment_count)
        self._batch_path_builds += 1
        return path

    def _cached_simplified_path(
        self,
        cache_label: Hashable,
        key: Hashable,
        vertices: list[_ProjectedVertex],
    ) -> tuple[QPainterPath, int, list[_ProjectedVertex]]:
        """按标签缓存简化边界路径。注意：时间窗切入块内部时才走该分支。"""

        cached = self._batch_cache.get(cache_label)
        if cached is not None and cached[0] == key:
            # 局部简化始终保留切片首点，虚线相位可直接从原始首点恢复。
            return cached[1], cached[2], [vertices[0]]
        display_vertices = self._simplified_vertices(vertices)
        path = self._path_from_vertices(display_vertices)
        segment_count = max(0, len(display_vertices) - 1)
        self._batch_cache[cache_label] = (key, path, segment_count)
        self._batch_path_builds += 1
        return path, segment_count, display_vertices

    def _simplified_vertices(self, vertices: Sequence[_ProjectedVertex]) -> list[_ProjectedVertex]:
        """按当前局部预算选择显示顶点。注意：选择过程不读取其它块或全局尾迹长度。"""

        projected_points = [vertex.projected for vertex in vertices]
        indices = simplify_polyline_indices(
            projected_points,
            tolerance=self.simplify_tolerance,
            max_segments=self.max_segments_per_chunk,
        )
        return [vertices[index] for index in indices]

    @staticmethod
    def _path_from_vertices(vertices: Sequence[_ProjectedVertex]) -> QPainterPath:
        """从连续顶点建立单个 QPainterPath。注意：不会创建逐段图元或画笔。"""

        path = QPainterPath()
        if not vertices:
            return path
        path.moveTo(vertices[0].projected)
        for vertex in vertices[1:]:
            path.lineTo(vertex.projected)
        return path

    def _opacity_factor(self, bucket: int) -> float:
        """返回透明度档代表值。注意：保留 0.08 下限避免旧尾迹突然完全消失。"""

        return max(0.08, (bucket + 1) / self.opacity_buckets)

    def _set_source_metadata(self, metadata: tuple[Any, int, int, int]) -> None:
        """记录共享队列同步位置。注意：四个字段必须来自同一份快照。"""

        (
            self._source_generation,
            self._source_revision,
            self._source_first_sequence,
            self._source_end_sequence,
        ) = metadata

    def _clear_source_metadata(self) -> None:
        """清除共享队列同步位置。注意：切换到普通序列时必须避免误用旧 revision。"""

        self._source_generation = None
        self._source_revision = None
        self._source_first_sequence = None
        self._source_end_sequence = None
