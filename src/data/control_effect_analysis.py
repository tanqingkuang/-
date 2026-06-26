"""控制效果离线分析内核。注意：本模块不依赖 PySide6，可被 GUI 和脚本共同调用。"""

from __future__ import annotations

import csv
import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class AnalysisChannel:
    """单个控制误差分析通道。注意：field_name 必须对应 snapshots.jsonl 的节点字段。"""

    # 内部稳定键，用于样本缓冲和 UI 控件映射。
    key: str
    # 中文展示名，进入 GUI 表格和 CSV。
    label: str
    # 通道单位，供图表标题或后续导出扩展使用。
    unit: str
    # snapshots.jsonl 节点对象内的字段名。
    field_name: str


@dataclass(frozen=True)
class MetricSummary:
    """某个样本集合的统计结果。注意：variance 使用总体方差。"""

    # 有效样本数量，空时间段会表现为没有 summary。
    count: int
    # 有符号均值，用于判断误差偏向。
    mean: float
    # 总体方差，不做 n-1 修正。
    variance: float
    # 标准差，与原始误差同量纲。
    std: float
    # 均方根误差，用于衡量综合误差能量。
    rms: float
    # 最大绝对误差。
    max_abs: float
    # 最大绝对误差首次出现的仿真时刻。
    max_abs_time_s: float


@dataclass(frozen=True)
class AnalysisSourceData:
    """单份 snapshots.jsonl 解析后的样本数据。"""

    label: str
    path: Path
    # node_id -> channel_key -> [(time_s, value)]。
    samples: dict[str, dict[str, list[tuple[float, float]]]] = field(default_factory=dict)
    # 文件中有效记录的最早仿真时刻。
    t_min: float = 0.0
    # 文件中有效记录的最晚仿真时刻。
    t_max: float = 0.0


@dataclass(frozen=True)
class AnalysisResult:
    """单份输入源在指定时间段内的完整指标结果。"""

    # 原始解析数据，保留 label、路径和节点样本供上层追溯。
    source: AnalysisSourceData
    # 规范化后的开始时间。
    start_s: float
    # 规范化后的结束时间。
    end_s: float
    # 本次结果覆盖的通道集合。
    channels: tuple[AnalysisChannel, ...]
    # 全机合并和逐飞机指标行，字段与 CSV 导出保持一致。
    metric_rows: tuple[dict[str, object], ...]

    def rows_for_scope(self, scope: str) -> tuple[dict[str, object], ...]:
        """按 scope 返回指标行，例如 all 或 node。"""
        return tuple(row for row in self.metric_rows if row["scope"] == scope)


DEFAULT_CHANNELS: tuple[AnalysisChannel, ...] = (
    # 位置误差三个轴按航迹坐标系 x/y/z 顺序展示。
    AnalysisChannel("pos_x", "前向位置误差 x", "m", "track_pos_err_x_m"),
    AnalysisChannel("pos_y", "垂向位置误差 y", "m", "track_pos_err_y_m"),
    AnalysisChannel("pos_z", "侧向位置误差 z", "m", "track_pos_err_z_m"),
    # 速度误差三个轴保持同样顺序，便于和位置误差对照。
    AnalysisChannel("vel_x", "前向速度误差 x", "m/s", "track_vel_err_x_mps"),
    AnalysisChannel("vel_y", "垂向速度误差 y", "m/s", "track_vel_err_y_mps"),
    AnalysisChannel("vel_z", "侧向速度误差 z", "m/s", "track_vel_err_z_mps"),
)
METRIC_FIELD_NAMES: tuple[str, ...] = (
    # 指标字段保持英文，便于脚本、CSV 和 GUI 共享同一套键名。
    "mean",
    "variance",
    "std",
    "rms",
    "max_abs",
    "max_abs_time_s",
)
CSV_FIELD_NAMES: tuple[str, ...] = (
    # CSV 首列保留输入标签，合并 A/B 导出后仍可追溯来源。
    "input_label",
    "source_path",
    "scope",
    "node_id",
    "channel",
    "channel_label",
    "count",
    *METRIC_FIELD_NAMES,
)


def load_snapshot_samples(
    path: str | Path,
    *,
    label: str,
    channels: Iterable[AnalysisChannel] = DEFAULT_CHANNELS,
) -> AnalysisSourceData:
    """读取 snapshots.jsonl 并提取默认控制误差通道。"""
    source_path = Path(path)
    # channels 可能来自 GUI 常量，先冻结为 tuple 便于多次遍历。
    channel_list = tuple(channels)
    samples: dict[str, dict[str, list[tuple[float, float]]]] = {}
    # times 单独记录，避免从各节点样本反推时遗漏无节点帧的时间边界。
    times: list[float] = []
    with source_path.open(encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            stripped = raw.strip()
            if not stripped:
                # 允许日志末尾或人工编辑后留下空行。
                continue
            record = _load_json_record(stripped, line_no)
            # time_s 是窗口和最大值定位的基础，缺失时不能兜底。
            t = _required_float(record, "time_s", line_no)
            nodes = record.get("nodes")
            if not isinstance(nodes, list):
                raise ValueError(f"第 {line_no} 行缺少 nodes 列表")
            # 时间范围按所有有效记录计算，不要求每帧节点集合完全一致。
            times.append(t)
            for node in nodes:
                _append_node_samples(samples, t, node, line_no, channel_list)
    if not times:
        # 全空文件没有任何可分析时间范围，调用方应显示加载失败。
        raise ValueError("文件为空")
    return AnalysisSourceData(
        label=label,
        path=source_path,
        samples=samples,
        t_min=min(times),
        t_max=max(times),
    )


def _load_json_record(raw: str, line_no: int) -> dict[str, object]:
    """解析单行 JSON 并校验其为对象。"""
    try:
        record = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"第 {line_no} 行不是合法 JSON：{exc.msg}") from exc
    if not isinstance(record, dict):
        # 顶层数组或数字即使 JSON 合法，也不符合 snapshots.jsonl 契约。
        raise ValueError(f"第 {line_no} 行不是 JSON 对象")
    return record


def _required_float(record: dict[str, object], field_name: str, line_no: int) -> float:
    """读取记录级必填数值字段。"""
    if field_name not in record:
        # 必填记录字段缺失要显式报错，避免生成看似正常的错误指标。
        raise ValueError(f"第 {line_no} 行缺少 {field_name}")
    try:
        return float(record[field_name])
    except (TypeError, ValueError):
        # 类型错误和格式错误统一成面向用户的中文 ValueError。
        raise ValueError(f"第 {line_no} 行 {field_name} 不是数值") from None


def _append_node_samples(
    samples: dict[str, dict[str, list[tuple[float, float]]]],
    time_s: float,
    node: object,
    line_no: int,
    channels: tuple[AnalysisChannel, ...],
) -> None:
    """把单个节点的一帧数据追加到样本缓冲。"""
    if not isinstance(node, dict):
        raise ValueError(f"第 {line_no} 行存在非对象节点")
    node_id = str(node.get("node_id", "")).strip()
    if not node_id:
        # node_id 是按飞机聚合的主键，不能用行号或序号代替。
        raise ValueError(f"第 {line_no} 行存在空 node_id")
    node_samples = samples.setdefault(node_id, {})
    for channel in channels:
        if channel.field_name not in node:
            # 通道缺失按错误处理，而不是按 0 兜底，避免掩盖日志契约破坏。
            raise ValueError(f"第 {line_no} 行 {node_id} 缺少 {channel.field_name}")
        try:
            value = float(node[channel.field_name])
        except (TypeError, ValueError):
            raise ValueError(f"第 {line_no} 行 {node_id}/{channel.label} 不是数值") from None
        node_samples.setdefault(channel.key, []).append((time_s, value))


def node_ids_from_sources(sources: Iterable[AnalysisSourceData]) -> list[str]:
    """返回多份输入源的 node_id 并集，按字典序排序。"""
    # 取并集可以让 B 未启用时也能预先看到已加载文件中的节点。
    return sorted({node_id for source in sources for node_id in source.samples})


def points_for(
    source: AnalysisSourceData,
    target: str,
    channel_key: str,
    start_s: float,
    end_s: float,
) -> list[tuple[float, float]]:
    """提取目标对象在指定时间段内的通道样本。"""
    start, end = normalized_time_range(start_s, end_s)
    result: list[tuple[float, float]] = []
    if target == "all":
        # all 合并所有飞机同一通道样本，再整体统计。
        node_ids: Iterable[str] = source.samples.keys()
    else:
        node_ids = (target,)
    for node_id in node_ids:
        # 缺少目标节点或通道时自然得到空列表，由上层显示为空位。
        channel_points = source.samples.get(node_id, {}).get(channel_key, [])
        result.extend((t, value) for t, value in channel_points if start <= t <= end)
    # 多机合并后按时间排序，保证窗口锚点顺序稳定。
    result.sort(key=lambda item: item[0])
    return result


def normalized_time_range(start_s: float, end_s: float) -> tuple[float, float]:
    """返回规范化后的时间段，用户输反时交换起止。"""
    # GUI 层不弹错，内核统一交换，脚本调用也能得到一致行为。
    return (end_s, start_s) if end_s < start_s else (start_s, end_s)


def summary_for(
    source: AnalysisSourceData,
    target: str,
    channel_key: str,
    start_s: float,
    end_s: float,
) -> MetricSummary | None:
    """计算某个输入源、对象和通道的时间段汇总指标。"""
    return calc_summary(points_for(source, target, channel_key, start_s, end_s))


def calc_summary(points: list[tuple[float, float]]) -> MetricSummary | None:
    """计算样本点的均值、方差、标准差、RMS 和最大绝对值。"""
    if not points:
        # 空时间段不返回零指标，防止用户误读为误差为 0。
        return None
    values = [value for _time_s, value in points]
    count = len(values)
    mean = sum(values) / count
    # 阶段二约定使用总体方差，不做 n-1 修正。
    variance = sum((value - mean) ** 2 for value in values) / count
    rms = math.sqrt(sum(value * value for value in values) / count)
    # max 保留首次达到最大绝对值的样本时刻。
    max_time, max_value = max(points, key=lambda item: abs(item[1]))
    return MetricSummary(
        count=count,
        mean=mean,
        variance=variance,
        std=math.sqrt(variance),
        rms=rms,
        max_abs=abs(max_value),
        max_abs_time_s=max_time,
    )


def sliding_window(
    points: list[tuple[float, float]],
    start_s: float,
    end_s: float,
    window_s: float,
) -> list[tuple[float, MetricSummary]]:
    """按样本时刻滑动窗口，返回每个窗口起点对应的统计结果。"""
    if not points or window_s <= 0.0:
        # 非正窗口不抛异常，GUI 已限制输入；脚本调用时返回空序列。
        return []
    start, end = normalized_time_range(start_s, end_s)
    relevant = [(t, value) for t, value in points if start <= t <= end]
    if not relevant:
        return []
    if any(relevant[index][0] < relevant[index - 1][0] for index in range(1, len(relevant))):
        # 公共函数允许脚本传入无序点；GUI 路径来自 points_for，通常已排序。
        relevant.sort(key=lambda item: item[0])
    # 只以真实样本时刻做锚点，且跳过超过 end 的尾部半窗口。
    anchors = sorted({t for t, _value in relevant if t + window_s <= end + 1e-12})
    if not anchors:
        return []
    result: list[tuple[float, MetricSummary]] = []
    # left/right 定义当前窗口在 relevant 中的半开区间 [left, right)。
    left = 0
    right = 0
    # total 和 square_total 支撑均值、方差、RMS 的 O(1) 更新。
    total = 0.0
    square_total = 0.0
    # 队列里保存 max_abs 候选点下标，队首始终是当前窗口最大绝对值。
    max_abs_indexes: deque[int] = deque()
    for anchor in anchors:
        window_end = anchor + window_s
        # 窗口左边界右移时同步扣除离开的样本。
        while left < len(relevant) and relevant[left][0] < anchor:
            _old_t, old_value = relevant[left]
            total -= old_value
            square_total -= old_value * old_value
            if max_abs_indexes and max_abs_indexes[0] == left:
                max_abs_indexes.popleft()
            left += 1
        # 窗口右边界右移时只追加新进入窗口的样本。
        while right < len(relevant) and relevant[right][0] < window_end:
            _new_t, new_value = relevant[right]
            total += new_value
            square_total += new_value * new_value
            # 单调队列按绝对值降序保留候选；相等时保留更早的样本时刻。
            while max_abs_indexes and abs(relevant[max_abs_indexes[-1]][1]) < abs(new_value):
                max_abs_indexes.pop()
            max_abs_indexes.append(right)
            right += 1
        # 清理已经在左边界之外、但不是刚好队首离开的旧候选。
        while max_abs_indexes and max_abs_indexes[0] < left:
            max_abs_indexes.popleft()
        count = right - left
        if count <= 0:
            continue
        mean = total / count
        # 浮点消差可能产生极小负数，用 max 保护 sqrt。
        variance = max(0.0, square_total / count - mean * mean)
        max_index = max_abs_indexes[0]
        max_time, max_value = relevant[max_index]
        result.append(
            (
                anchor,
                MetricSummary(
                    count=count,
                    mean=mean,
                    variance=variance,
                    std=math.sqrt(variance),
                    rms=math.sqrt(square_total / count),
                    max_abs=abs(max_value),
                    max_abs_time_s=max_time,
                ),
            )
        )
    return result


def metric_rows_for_source(
    source: AnalysisSourceData,
    start_s: float,
    end_s: float,
    *,
    channels: Iterable[AnalysisChannel] = DEFAULT_CHANNELS,
) -> list[dict[str, object]]:
    """生成单个输入源的全机和逐机指标行。"""
    # 兼容旧调用点：内部统一走 AnalysisResult，避免两套行生成逻辑分叉。
    return list(analyze_source(source, start_s, end_s, channels=channels).metric_rows)


def analyze_source(
    source: AnalysisSourceData,
    start_s: float,
    end_s: float,
    *,
    channels: Iterable[AnalysisChannel] = DEFAULT_CHANNELS,
) -> AnalysisResult:
    """生成单个输入源在指定时间段内的完整分析结果。"""
    start, end = normalized_time_range(start_s, end_s)
    channel_list = tuple(channels)
    rows: list[dict[str, object]] = []
    # 导出先写 all，再写逐机行，便于 Excel 中按 scope 筛选。
    targets = [("all", "all"), *[("node", node_id) for node_id in sorted(source.samples)]]
    for scope, node_id in targets:
        # scope=all 使用合并样本；scope=node 使用单机样本，字段结构保持一致。
        target = "all" if scope == "all" else node_id
        for channel in channel_list:
            # 导出覆盖全部通道，不受 GUI 绘图通道选择影响。
            summary = summary_for(source, target, channel.key, start, end)
            rows.append(_metric_row(source, scope, node_id, channel, summary))
    return AnalysisResult(
        source=source,
        start_s=start,
        end_s=end,
        channels=channel_list,
        metric_rows=tuple(rows),
    )


def _metric_row(
    source: AnalysisSourceData,
    scope: str,
    node_id: str,
    channel: AnalysisChannel,
    summary: MetricSummary | None,
) -> dict[str, object]:
    """把一个统计结果转换为 CSV 行。"""
    # 无样本也输出结构化行，便于外部脚本发现缺口。
    filled = summary or MetricSummary(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return {
        # 数值字段用紧凑字符串，避免 CSV 中出现过长浮点尾数。
        "input_label": source.label,
        "source_path": str(source.path),
        "scope": scope,
        "node_id": node_id,
        "channel": channel.field_name,
        "channel_label": channel.label,
        "count": filled.count,
        "mean": f"{filled.mean:.6g}",
        "variance": f"{filled.variance:.6g}",
        "std": f"{filled.std:.6g}",
        "rms": f"{filled.rms:.6g}",
        "max_abs": f"{filled.max_abs:.6g}",
        "max_abs_time_s": f"{filled.max_abs_time_s:.6g}",
    }


def write_metrics_csv(
    path: str | Path,
    sources: Iterable[AnalysisSourceData],
    start_s: float,
    end_s: float,
    *,
    channels: Iterable[AnalysisChannel] = DEFAULT_CHANNELS,
) -> None:
    """把多份输入源的全机和逐机指标写入 CSV。"""
    output_path = Path(path)
    # channels 先冻结，保证写入期间不会受外部可变迭代器影响。
    channel_list = tuple(channels)
    with output_path.open("w", encoding="utf-8-sig", newline="") as handle:
        # utf-8-sig 让 Excel 直接打开 CSV 时能正确识别中文表头。
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELD_NAMES)
        writer.writeheader()
        for source in sources:
            # A/B 顺序写入同一个 CSV，input_label 保留来源。
            for row in metric_rows_for_source(source, start_s, end_s, channels=channel_list):
                writer.writerow(row)
