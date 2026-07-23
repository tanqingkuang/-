"""控制效果离线分析内核。注意：本模块不依赖 PySide6，可被 GUI 和脚本共同调用。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterable

import numpy as np

from src.common.coordinates import enu_to_fur, fur_basis_from_angles


@dataclass(frozen=True)
class AnalysisChannel:
    """单个控制误差分析通道。注意：field_name 必须对应 snapshots.jsonl 的节点字段。"""

    # 内部稳定键，用于样本缓冲和 UI 控件映射。
    key: str
    # 中文展示名，进入 GUI 表格和 CSV。
    label: str
    # 通道单位，供图表标题或后续导出扩展使用。
    unit: str
    # snapshots.jsonl 节点对象内的字段名；派生通道为空字符串。
    field_name: str
    # 可选通道：旧日志缺字段或值为 null 时按"无样本"跳过，不判定日志契约破坏。
    optional: bool = False
    # 派生通道：不直接读节点字段，由每帧记录跨字段/跨节点计算（见 _append_derived_samples）。
    derived: bool = False
    # 积分只取正部：时间积分按 ∫max(v,0)dt 计算（如外甩面积），负值不得抵消正值。
    integral_positive_only: bool = False


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
    # 绝对值 95 分位，反映去除尖峰后的高分位误差水平；滑窗路径不计算，保持 0。
    p95_abs: float = 0.0
    # 总变差 Σ|Δv|，衡量信号抖动/频繁反向操纵；滑窗路径不计算，保持 0。
    tv: float = 0.0
    # 有符号时间积分（梯形法）；对超限量通道即"外甩面积/超载积分"；滑窗路径不计算。
    integral: float = 0.0


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
    # numpy 加速缓存：node_id/"all" -> channel_key -> (时间数组, 数值数组)，按时间升序。
    # 解析结束时逐节点构建，"all" 合并序列按需惰性生成；frozen 只约束字段绑定，字典内容可变。
    arrays: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]] = field(default_factory=dict)


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
# 扩展通道均为可选：旧日志缺字段时按"无样本"呈现，不阻断六个基础误差通道的分析。
EXTENDED_CHANNELS: tuple[AnalysisChannel, ...] = (
    # 公共几何裁判量（派生）：口径遵循 docs/codex指标.md 第 4.1 节。
    # e_perp 取航线左侧为正，与快照 cross_track_error_m（右正）符号相反。
    AnalysisChannel("e_perp", "航线横偏(左正)", "m", "", optional=True, derived=True),
    # e_out=-sgn(κ)·e_perp，弯道外侧统一为正；直线段（turn_sign=0）不产生样本。
    # 外甩面积口径为 ∫max(e_out,0)dt（docs/codex指标.md §4.3），积分只取正部。
    AnalysisChannel("e_out", "弯道外甩偏差", "m", "", optional=True, derived=True, integral_positive_only=True),
    # 刚性槽位误差 R_L^T(p_i-p_L)-(r_i*-r_L*)，长机 FUR 轴序前/上/右；长机自身不产生样本。
    AnalysisChannel("e_rigid_x", "刚性槽位误差 前向", "m", "", optional=True, derived=True),
    AnalysisChannel("e_rigid_y", "刚性槽位误差 上向", "m", "", optional=True, derived=True),
    AnalysisChannel("e_rigid_z", "刚性槽位误差 右向", "m", "", optional=True, derived=True),
    # 过载裁判量（派生）：n_tot 为三轴合过载，n_over 为 max(n_tot-1,0)^2，
    # 对 n_over 在窗口内取时间积分即 J_n（1g 以上合过载平方积分）。
    AnalysisChannel("n_tot", "三轴合过载", "g", "", optional=True, derived=True),
    AnalysisChannel("n_over", "超1g合过载平方", "g²", "", optional=True, derived=True),
    # 直读扩展通道：机动姿态、原始控制指令、饱和证据与计算代价。
    AnalysisChannel("phi", "滚转角", "deg", "phi_deg", optional=True),
    AnalysisChannel("n_normal", "法向合过载", "g", "n_normal", optional=True),
    AnalysisChannel("cmd_acc_e", "指令加速度 东", "m/s²", "cmd_acc_east_mps2", optional=True),
    AnalysisChannel("cmd_acc_n", "指令加速度 北", "m/s²", "cmd_acc_north_mps2", optional=True),
    AnalysisChannel("cmd_acc_u", "指令加速度 天", "m/s²", "cmd_acc_up_mps2", optional=True),
    # 饱和标志按 0/1 采样，窗口均值即饱和占空比。
    AnalysisChannel("acc_sat", "指令饱和(0/1)", "", "acc_saturated", optional=True),
    AnalysisChannel("algo_ms", "算法单步耗时", "ms", "algo_step_ms", optional=True),
)
# GUI 与批量导出使用的全量通道 = 基础误差通道 + 扩展评测通道。
GUI_CHANNELS: tuple[AnalysisChannel, ...] = DEFAULT_CHANNELS + EXTENDED_CHANNELS
METRIC_FIELD_NAMES: tuple[str, ...] = (
    # 指标字段保持英文，便于脚本、CSV 和 GUI 共享同一套键名。
    "mean",
    "variance",
    "std",
    "rms",
    "max_abs",
    "max_abs_time_s",
    "p95_abs",
    "tv",
    "integral",
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
# 滑窗曲线锚点上限：同时约束计算量与图表点数，超过时对锚点均匀抽稀。
MAX_WINDOW_ANCHORS = 2000
# 积分只取正部的通道键集合，供只持有 channel_key 的统计路径查询口径。
_INTEGRAL_POSITIVE_KEYS = frozenset(
    channel.key for channel in GUI_CHANNELS if channel.integral_positive_only
)


def load_snapshot_samples(
    path: str | Path,
    *,
    label: str,
    channels: Iterable[AnalysisChannel] = DEFAULT_CHANNELS,
) -> AnalysisSourceData:
    """读取 snapshots.jsonl 并提取控制误差与评测通道。"""
    source_path = Path(path)
    # channels 可能来自 GUI 常量，先冻结为 tuple 便于多次遍历。
    channel_list = tuple(channels)
    direct_channels = tuple(channel for channel in channel_list if not channel.derived)
    derived_keys = {channel.key for channel in channel_list if channel.derived}
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
                _append_node_samples(samples, t, node, line_no, direct_channels)
            if derived_keys:
                _append_derived_samples(samples, t, record, nodes, line_no, derived_keys)
    if not times:
        # 全空文件没有任何可分析时间范围，调用方应显示加载失败。
        raise ValueError("文件为空")
    source = AnalysisSourceData(
        label=label,
        path=source_path,
        samples=samples,
        t_min=min(times),
        t_max=max(times),
    )
    _build_array_cache(source)
    return source


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
        if channel.field_name not in node or node[channel.field_name] is None:
            if channel.optional:
                # 可选通道缺失/为 null 只表示旧日志或该帧无值，按"无样本"跳过。
                continue
            # 必填通道缺失按错误处理，而不是按 0 兜底，避免掩盖日志契约破坏。
            raise ValueError(f"第 {line_no} 行 {node_id} 缺少 {channel.field_name}")
        value = node[channel.field_name]
        if isinstance(value, bool):
            # 布尔型证据字段（如指令饱和）按 0/1 采样，窗口均值即占空比。
            value = 1.0 if value else 0.0
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"第 {line_no} 行 {node_id}/{channel.label} 不是数值") from None
        node_samples.setdefault(channel.key, []).append((time_s, value))


def _append_derived_samples(
    samples: dict[str, dict[str, list[tuple[float, float]]]],
    time_s: float,
    record: dict[str, object],
    nodes: list[object],
    line_no: int,
    derived_keys: set[str],
) -> None:
    """按帧计算派生裁判量并追加样本。注意：所需字段缺失时按"无样本"静默跳过。"""
    del line_no  # 派生通道全部可选，缺字段不报错，保留参数占位便于将来诊断。
    node_dicts = [node for node in nodes if isinstance(node, dict)]
    # 弯道符号来自当前航段几何；旧日志无 route 字段时 e_out 整体不可用。
    route = record.get("route")
    turn_sign = route.get("turn_sign") if isinstance(route, dict) else None
    # 刚性槽位误差以长机为参考帧；无长机角色（如单机场景）时 e_rigid 不可用。
    leader = next(
        (node for node in node_dicts if "leader" in str(node.get("role", ""))),
        None,
    )
    leader_basis = None
    leader_pos = None
    leader_slot = (0.0, 0.0, 0.0)
    if leader is not None and {"e_rigid_x", "e_rigid_y", "e_rigid_z"} & derived_keys:
        theta_deg = leader.get("theta_deg")
        psi_deg = leader.get("psi_v_deg")
        if isinstance(theta_deg, (int, float)) and isinstance(psi_deg, (int, float)):
            try:
                # 长机 FUR 基由地面航迹角建立，与快照 track_* 字段同一契约。
                leader_basis = fur_basis_from_angles(math.radians(theta_deg), math.radians(psi_deg))
            except ValueError:
                leader_basis = None
            leader_pos = (
                float(leader.get("x_m", 0.0)),
                float(leader.get("y_m", 0.0)),
                float(leader.get("altitude_m", 0.0)),
            )
            # 长机槽位通常为原点；非原点布局（如 A05 居中）用相对槽位差保持口径一致。
            if isinstance(leader.get("slot_x_m"), (int, float)):
                leader_slot = (
                    float(leader["slot_x_m"]),
                    float(leader.get("slot_y_m") or 0.0),
                    float(leader.get("slot_z_m") or 0.0),
                )
    for node in node_dicts:
        node_id = str(node.get("node_id", "")).strip()
        if not node_id:
            continue
        node_samples = samples.setdefault(node_id, {})
        # 过载合量：nx/ny/nz 均为 FUR 有符号轴分量，n_tot 恒非负。
        if {"n_tot", "n_over"} & derived_keys:
            nx, ny, nz = node.get("nx"), node.get("ny"), node.get("nz")
            if all(isinstance(v, (int, float)) for v in (nx, ny, nz)):
                n_tot = math.sqrt(nx * nx + ny * ny + nz * nz)
                if "n_tot" in derived_keys:
                    node_samples.setdefault("n_tot", []).append((time_s, n_tot))
                if "n_over" in derived_keys:
                    node_samples.setdefault("n_over", []).append((time_s, max(n_tot - 1.0, 0.0) ** 2))
        # 航线横偏：快照右侧为正，评测口径左侧为正，复用字段必须取反。
        cte = node.get("cross_track_error_m")
        if isinstance(cte, (int, float)):
            e_perp = -float(cte)
            if "e_perp" in derived_keys:
                node_samples.setdefault("e_perp", []).append((time_s, e_perp))
            if "e_out" in derived_keys and isinstance(turn_sign, (int, float)) and turn_sign != 0.0:
                # 本项目 turn_sign=+1 左转（κ>0），e_out=-sgn(κ)·e_perp 使弯道外侧为正。
                node_samples.setdefault("e_out", []).append(
                    (time_s, -math.copysign(1.0, turn_sign) * e_perp)
                )
        # 刚性槽位误差：长机自身不参与，僚机需具备槽位坐标。
        if leader_basis is not None and leader_pos is not None and node is not leader:
            slot_x = node.get("slot_x_m")
            if isinstance(slot_x, (int, float)):
                delta = (
                    float(node.get("x_m", 0.0)) - leader_pos[0],
                    float(node.get("y_m", 0.0)) - leader_pos[1],
                    float(node.get("altitude_m", 0.0)) - leader_pos[2],
                )
                rel = enu_to_fur(delta, leader_basis)
                slot = (
                    float(slot_x) - leader_slot[0],
                    float(node.get("slot_y_m") or 0.0) - leader_slot[1],
                    float(node.get("slot_z_m") or 0.0) - leader_slot[2],
                )
                for axis, key in enumerate(("e_rigid_x", "e_rigid_y", "e_rigid_z")):
                    if key in derived_keys:
                        node_samples.setdefault(key, []).append((time_s, rel[axis] - slot[axis]))


def _build_array_cache(source: AnalysisSourceData) -> None:
    """把逐节点样本列表转换为按时间升序的 numpy 数组缓存。"""
    for node_id, node_samples in source.samples.items():
        node_arrays: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for key, points in node_samples.items():
            t = np.fromiter((p[0] for p in points), dtype=np.float64, count=len(points))
            v = np.fromiter((p[1] for p in points), dtype=np.float64, count=len(points))
            if len(t) > 1 and np.any(np.diff(t) < 0.0):
                # 解析顺序通常即时间顺序；个别乱序文件只排一次序，后续全走二分。
                order = np.argsort(t, kind="stable")
                t, v = t[order], v[order]
            node_arrays[key] = (t, v)
        source.arrays[node_id] = node_arrays


def _merged_arrays(source: AnalysisSourceData, channel_key: str) -> tuple[np.ndarray, np.ndarray]:
    """返回全机合并后的通道数组，惰性构建并缓存到 arrays['all']。"""
    all_cache = source.arrays.setdefault("all", {})
    cached = all_cache.get(channel_key)
    if cached is not None:
        return cached
    parts = [
        arrays[channel_key]
        for node_id, arrays in source.arrays.items()
        if node_id != "all" and channel_key in arrays
    ]
    if not parts:
        empty = (np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64))
        all_cache[channel_key] = empty
        return empty
    t = np.concatenate([part[0] for part in parts])
    v = np.concatenate([part[1] for part in parts])
    # 多机合并后按时间排序，保证窗口锚点顺序稳定；缓存后重复查询为 O(1)。
    order = np.argsort(t, kind="stable")
    merged = (t[order], v[order])
    all_cache[channel_key] = merged
    return merged


def series_for(
    source: AnalysisSourceData,
    target: str,
    channel_key: str,
    start_s: float,
    end_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """提取目标对象在闭区间时间段内的通道数组视图。注意：这是所有统计的公共快路径。"""
    start, end = normalized_time_range(start_s, end_s)
    if target == "all":
        t, v = _merged_arrays(source, channel_key)
    else:
        t, v = source.arrays.get(target, {}).get(
            channel_key, (np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64))
        )
    # 数组已按时间升序，闭区间裁剪用二分定位，避免整列布尔扫描。
    left = int(np.searchsorted(t, start, side="left"))
    right = int(np.searchsorted(t, end, side="right"))
    return t[left:right], v[left:right]


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
    t, v = series_for(source, target, channel_key, start_s, end_s)
    # tolist 一次性转回内建 float，保持既有"(time, value) 列表"公共契约。
    return list(zip(t.tolist(), v.tolist()))


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
    """计算某个输入源、对象和通道的时间段汇总指标。

    分布类指标（均值/方差/RMS/P95/最大值）在 all 目标下按合并样本统计；
    时序类指标（总变差/时间积分）跨机拼接会把机间差当成时间变化，
    因此 all 目标必须逐机计算后求和（全队总控制活动量/总面积口径）。
    """
    positive_integral = channel_key in _INTEGRAL_POSITIVE_KEYS
    t, v = series_for(source, target, channel_key, start_s, end_s)
    summary = _summary_from_arrays(t, v, positive_integral=positive_integral)
    if summary is None or target != "all":
        return summary
    start, end = normalized_time_range(start_s, end_s)
    tv_total = 0.0
    integral_total = 0.0
    for node_id in source.arrays:
        if node_id == "all":
            continue
        node_t, node_v = series_for(source, node_id, channel_key, start, end)
        if len(node_t) > 1:
            tv_total += float(np.sum(np.abs(np.diff(node_v))))
            integral_total += _integral_from_arrays(node_t, node_v, positive_integral)
    return replace(summary, tv=tv_total, integral=integral_total)


def calc_summary(points: list[tuple[float, float]]) -> MetricSummary | None:
    """计算样本点的均值、方差、标准差、RMS、最大绝对值及扩展指标。"""
    if not points:
        # 空时间段不返回零指标，防止用户误读为误差为 0。
        return None
    t = np.fromiter((p[0] for p in points), dtype=np.float64, count=len(points))
    v = np.fromiter((p[1] for p in points), dtype=np.float64, count=len(points))
    return _summary_from_arrays(t, v)


def _integral_from_arrays(t: np.ndarray, v: np.ndarray, positive_only: bool) -> float:
    """按梯形法计算时间积分；positive_only 时先截取正部（如外甩面积口径）。"""
    if len(v) < 2:
        return 0.0
    values = np.maximum(v, 0.0) if positive_only else v
    return float(np.trapezoid(values, t))


def _summary_from_arrays(
    t: np.ndarray,
    v: np.ndarray,
    *,
    positive_integral: bool = False,
) -> MetricSummary | None:
    """按 numpy 数组计算全套汇总指标。注意：t 必须升序，扩展指标依赖时间顺序。"""
    count = int(len(v))
    if count == 0:
        return None
    abs_v = np.abs(v)
    mean = float(np.mean(v))
    # 阶段二约定使用总体方差，不做 n-1 修正。
    variance = float(np.mean((v - mean) ** 2))
    rms = float(np.sqrt(np.mean(v * v)))
    # argmax 返回首个最大值下标，保留首次达到最大绝对值的样本时刻。
    max_index = int(np.argmax(abs_v))
    # 总变差衡量抖动；单样本无差分，积分同理为 0。
    tv = float(np.sum(np.abs(np.diff(v)))) if count > 1 else 0.0
    integral = _integral_from_arrays(t, v, positive_integral)
    return MetricSummary(
        count=count,
        mean=mean,
        variance=variance,
        std=math.sqrt(variance),
        rms=rms,
        max_abs=float(abs_v[max_index]),
        max_abs_time_s=float(t[max_index]),
        p95_abs=float(np.percentile(abs_v, 95.0)),
        tv=tv,
        integral=integral,
    )


def convergence_time_s(
    points: list[tuple[float, float]] | tuple[np.ndarray, np.ndarray],
    band: float,
    hold_s: float,
) -> float | None:
    """按误差带 band 和保持时长 hold_s 计算收敛时刻。

    口径遵循 docs/codex指标.md：进入 |v|<=band 并连续保持 hold_s 后的最早时刻
    （即入带时刻 + hold_s），不能使用第一次穿越阈值的时刻；此后不要求继续保持。
    样本覆盖不足 hold_s 或从未满足时返回 None。
    """
    if isinstance(points, tuple) and len(points) == 2 and isinstance(points[0], np.ndarray):
        t, v = points
    else:
        t = np.fromiter((p[0] for p in points), dtype=np.float64, count=len(points))
        v = np.fromiter((p[1] for p in points), dtype=np.float64, count=len(points))
    if len(t) == 0 or band < 0.0 or hold_s < 0.0:
        return None
    inside = np.abs(v) <= band
    # 向量化定位连续入带区段边界：首尾补 False 后差分，+1 为区段起点，-1 为区段终点后一位。
    padded = np.concatenate(([False], inside, [False]))
    edges = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1) - 1
    # 第一个持续时长达到 hold_s 的区段即收敛，返回其入带时刻 + hold_s。
    for start, end in zip(starts, ends):
        if t[end] - t[start] >= hold_s:
            return float(t[start] + hold_s)
    return None


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
    t = np.fromiter((p[0] for p in relevant), dtype=np.float64, count=len(relevant))
    v = np.fromiter((p[1] for p in relevant), dtype=np.float64, count=len(relevant))
    # 只以真实样本时刻做锚点，且跳过超过 end 的尾部半窗口；锚点数受上限约束，
    # 超限时均匀抽稀——同时压住统计计算量和图表点数，避免大数据量卡顿。
    anchors = np.unique(t[t + window_s <= end + 1e-12])
    if len(anchors) == 0:
        return []
    if len(anchors) > MAX_WINDOW_ANCHORS:
        keep = np.linspace(0, len(anchors) - 1, MAX_WINDOW_ANCHORS).round().astype(np.int64)
        anchors = anchors[np.unique(keep)]
    # 窗口区间 [anchor, anchor+window) 的左右边界用二分批量定位。
    lefts = np.searchsorted(t, anchors, side="left")
    rights = np.searchsorted(t, anchors + window_s, side="left")
    # 前缀和支撑均值/方差/RMS 的 O(1) 查询；首位补零便于差分。
    prefix = np.concatenate(([0.0], np.cumsum(v)))
    prefix_sq = np.concatenate(([0.0], np.cumsum(v * v)))
    abs_v = np.abs(v)
    result: list[tuple[float, MetricSummary]] = []
    # 最大绝对值用单调队列维护：锚点窗口左右边界均单调右移，整体 O(n + 锚点数)。
    max_candidates: deque[int] = deque()
    cursor = 0
    for anchor_index in range(len(anchors)):
        left = int(lefts[anchor_index])
        right = int(rights[anchor_index])
        count = right - left
        if count <= 0:
            continue
        # 右边界推进：新样本入队时弹出绝对值更小的候选，相等保留更早时刻。
        while cursor < right:
            while max_candidates and abs_v[max_candidates[-1]] < abs_v[cursor]:
                max_candidates.pop()
            max_candidates.append(cursor)
            cursor += 1
        # 左边界推进：弹出已滑出窗口的队首候选。
        while max_candidates and max_candidates[0] < left:
            max_candidates.popleft()
        total = prefix[right] - prefix[left]
        square_total = prefix_sq[right] - prefix_sq[left]
        mean = total / count
        # 浮点消差可能产生极小负数，用 max 保护 sqrt。
        variance = max(0.0, square_total / count - mean * mean)
        mean_square = max(0.0, square_total / count)
        max_index = max_candidates[0]
        result.append(
            (
                float(anchors[anchor_index]),
                MetricSummary(
                    count=count,
                    mean=float(mean),
                    variance=float(variance),
                    std=math.sqrt(variance),
                    rms=math.sqrt(mean_square),
                    max_abs=float(abs_v[max_index]),
                    max_abs_time_s=float(t[max_index]),
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
        "channel": channel.field_name or channel.key,
        "channel_label": channel.label,
        "count": filled.count,
        "mean": f"{filled.mean:.6g}",
        "variance": f"{filled.variance:.6g}",
        "std": f"{filled.std:.6g}",
        "rms": f"{filled.rms:.6g}",
        "max_abs": f"{filled.max_abs:.6g}",
        "max_abs_time_s": f"{filled.max_abs_time_s:.6g}",
        "p95_abs": f"{filled.p95_abs:.6g}",
        "tv": f"{filled.tv:.6g}",
        "integral": f"{filled.integral:.6g}",
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


def _choose_snapshot_file() -> Path | None:
    """弹出文件选择框并返回用户选择的仿真快照。"""

    # tkinter 只在省略命令行路径时按需加载，无界面批处理不引入 GUI 依赖。
    from tkinter import Tk, filedialog

    project_root = Path(__file__).resolve().parents[2]
    root = Tk()
    root.withdraw()
    try:
        selected = filedialog.askopenfilename(
            title="选择要分析的仿真快照",
            initialdir=project_root / "result" / "simulation_data" / "logs",
            filetypes=(("仿真快照", "snapshots_seed_*.jsonl"),),
        )
    finally:
        root.destroy()
    return Path(selected) if selected else None


def _parse_cli_args(argv: list[str]) -> argparse.Namespace:
    """解析控制效果分析命令行参数。"""

    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="生成控制效果指标报告")
    parser.add_argument(
        "snapshot_file",
        type=Path,
        nargs="?",
        help="要分析的 snapshots_seed_<seed>.jsonl；省略时弹出文件选择框",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_root / "result" / "analysis",
        help="分析报告根目录",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """使用通用控制效果分析口径导出单次仿真的全量指标。"""

    args = _parse_cli_args(list(sys.argv[1:] if argv is None else argv))
    try:
        selected_file = args.snapshot_file or _choose_snapshot_file()
        if selected_file is None:
            print("已取消控制效果分析。")
            return 0
        snapshot_file = selected_file.resolve()
        if not snapshot_file.is_file():
            raise ValueError(f"请选择有效的 snapshots_seed_<seed>.jsonl：{snapshot_file}")
        source = load_snapshot_samples(
            snapshot_file,
            label=snapshot_file.parent.name,
            channels=GUI_CHANNELS,
        )
        output_dir = args.output_root.resolve() / source.label
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "control_effect_metrics.csv"
        write_metrics_csv(
            output_path,
            [source],
            source.t_min,
            source.t_max,
            channels=GUI_CHANNELS,
        )
    except (OSError, ValueError) as exc:
        print(f"控制效果分析失败：{exc}")
        return 1
    print(f"控制效果分析完成：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
