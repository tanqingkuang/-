"""编队控制精度离线分析。注意：只读取仿真结果，不参与控制闭环。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from src.data.control_effect_analysis import (
    GUI_CHANNELS,
    AnalysisSourceData,
    MetricSummary,
    calc_summary,
    convergence_time_s,
    load_snapshot_samples,
)


# 精度报告只加载位置和刚性槽位通道，避免把速度、姿态等无关列驻留内存。
# 三轴模长在本模块按同一时刻派生，基础分析内核仍保持单通道统计职责。
_ACCURACY_CHANNEL_KEYS = frozenset(
    {
        "pos_x",
        "pos_y",
        "pos_z",
        "e_rigid_x",
        "e_rigid_y",
        "e_rigid_z",
    }
)
ACCURACY_CHANNELS = tuple(channel for channel in GUI_CHANNELS if channel.key in _ACCURACY_CHANNEL_KEYS)
# 每架僚机只占一行，两类三维误差各自展开为固定统计列。
DETAIL_FIELD_NAMES = (
    "运行编号",
    "飞机编号",
    "样本数",
    "编队三维位置误差均值(米)",
    "编队三维位置误差方差(米²)",
    "编队三维位置误差标准差(米)",
    "编队三维位置误差均方根(米)",
    "编队三维位置误差绝对值95分位(米)",
    "编队三维位置误差最大值(米)",
    "编队三维位置误差最大值时刻(秒)",
    "跟踪三维位置误差均值(米)",
    "跟踪三维位置误差方差(米²)",
    "跟踪三维位置误差标准差(米)",
    "跟踪三维位置误差均方根(米)",
    "跟踪三维位置误差绝对值95分位(米)",
    "跟踪三维位置误差最大值(米)",
    "跟踪三维位置误差最大值时刻(秒)",
)


@dataclass(frozen=True)
class FormationAccuracyReport:
    """单次仿真的逐僚机编队控制精度报告。"""

    # run_id 作为报告子目录名。
    run_id: str
    # 未完成集结时保持 None，不用仿真结束时刻伪装为 HOLD 起点。
    hold_start_s: float | None
    # 稳定起点是全队进入 HOLD 后连续满足误差门限的完成时刻。
    stable_start_s: float | None
    status: str
    metric_rows: tuple[dict[str, object], ...]


def _read_thresholds(run_dir: Path) -> tuple[float, float]:
    """从运行配置读取最终精度门限与稳定保持时间。"""

    # 日志目录中的配置是运行时快照，避免用户事后修改 configs/ 影响历史口径。
    config_path = run_dir / "config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("config.json 根节点必须是对象")
    # 普通保持场景可能没有 rally_cfg，使用与设计文档一致的保守默认值。
    rally_cfg = config.get("rally_cfg")
    cfg = rally_cfg if isinstance(rally_cfg, dict) else {}
    return float(cfg.get("tight_radius_m", 5.0)), float(cfg.get("stable_hold_s", 5.0))


def _aligned_norm_series(
    source: AnalysisSourceData,
    node_id: str,
    channel_keys: tuple[str, str, str],
) -> tuple[np.ndarray, np.ndarray]:
    """对齐三轴样本并返回向量模长序列。"""

    # 三轴必须同时存在才计算模长，缺轴不能按 0 补齐，否则会虚假改善精度。
    arrays = source.arrays.get(node_id, {})
    parts = [arrays.get(key) for key in channel_keys]
    if any(part is None for part in parts):
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)
    typed_parts = [part for part in parts if part is not None]
    times = typed_parts[0][0]
    # 同一节点的三轴由同一快照生成，时间不一致说明日志已损坏或被错误裁剪。
    if not all(np.array_equal(times, part[0]) for part in typed_parts[1:]):
        raise ValueError(f"节点 {node_id} 的三轴数据不同步")
    return times, np.sqrt(sum(part[1] * part[1] for part in typed_parts))


def _slice_series(
    series: tuple[np.ndarray, np.ndarray],
    start_s: float,
    end_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """按闭区间裁剪已按时间排序的序列。"""

    times, values = series
    # 闭区间与公共 analysis 内核一致，HOLD 边界帧归入两段便于连续核对。
    left = int(np.searchsorted(times, start_s, side="left"))
    right = int(np.searchsorted(times, end_s, side="right"))
    return times[left:right], values[left:right]


def _summary_from_series(series: tuple[np.ndarray, np.ndarray]) -> MetricSummary:
    """复用公共统计口径计算 numpy 序列指标。"""

    times, values = series
    # 指标公式全部复用公共函数，方差、P95 和最大值口径不会产生第二套实现。
    points = list(zip(times.tolist(), values.tolist()))
    summary = calc_summary(points)
    if summary is None:
        raise ValueError("稳定保持阶段没有可统计的数据")
    return summary


def _team_max_series(
    node_series: dict[str, tuple[np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, np.ndarray]:
    """生成每个时刻全队最差的三维刚性槽位误差。"""

    if not node_series:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)
    first_times = next(iter(node_series.values()))[0]
    # 所有节点来自同一快照序列，时间不一致时不能继续计算全队最大值。
    if not all(np.array_equal(first_times, times) for times, _values in node_series.values()):
        raise ValueError("僚机编队误差数据不同步")
    # 行最大值回答“该时刻最差的一架偏离槽位多少”，用于全队收敛判定。
    return first_times, np.max(np.vstack([values for _times, values in node_series.values()]), axis=0)


def _detect_hold_start(snapshot_path: Path) -> float | None:
    """返回全队首次同拍处于 HOLD 的仿真时刻。"""

    with snapshot_path.open(encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            if not raw.strip():
                continue
            record = json.loads(raw)
            if not isinstance(record, dict):
                raise ValueError(f"snapshots.jsonl 第 {line_no} 行不是对象")
            time_s = float(record["time_s"])
            nodes = record.get("nodes")
            if not isinstance(nodes, list):
                raise ValueError(f"snapshots.jsonl 第 {line_no} 行缺少 nodes")
            node_dicts = [node for node in nodes if isinstance(node, dict)]
            if len(node_dicts) != len(nodes) or not node_dicts:
                raise ValueError(f"snapshots.jsonl 第 {line_no} 行节点数据无效")
            task_stages = [node.get("task_stage") for node in node_dicts]
            if any(stage in {None, ""} for stage in task_stages):
                raise ValueError(f"snapshots.jsonl 第 {line_no} 行缺少 task_stage")
            # 必须全员同拍进入 HOLD，不能用第一架进入时间代替。
            if all(str(stage) == "HOLD" for stage in task_stages):
                return time_s
    return None


def _follower_metric_row(
    run_id: str,
    node_id: str,
    rigid_summary: MetricSummary,
    track_summary: MetricSummary,
) -> dict[str, object]:
    """把单架僚机的两类三维误差合并为一行中文指标。"""

    return {
        "运行编号": run_id,
        "飞机编号": node_id,
        "样本数": min(rigid_summary.count, track_summary.count),
        "编队三维位置误差均值(米)": rigid_summary.mean,
        "编队三维位置误差方差(米²)": rigid_summary.variance,
        "编队三维位置误差标准差(米)": rigid_summary.std,
        "编队三维位置误差均方根(米)": rigid_summary.rms,
        "编队三维位置误差绝对值95分位(米)": rigid_summary.p95_abs,
        "编队三维位置误差最大值(米)": rigid_summary.max_abs,
        "编队三维位置误差最大值时刻(秒)": rigid_summary.max_abs_time_s,
        "跟踪三维位置误差均值(米)": track_summary.mean,
        "跟踪三维位置误差方差(米²)": track_summary.variance,
        "跟踪三维位置误差标准差(米)": track_summary.std,
        "跟踪三维位置误差均方根(米)": track_summary.rms,
        "跟踪三维位置误差绝对值95分位(米)": track_summary.p95_abs,
        "跟踪三维位置误差最大值(米)": track_summary.max_abs,
        "跟踪三维位置误差最大值时刻(秒)": track_summary.max_abs_time_s,
    }


def analyze_formation_accuracy(run_dir: str | Path) -> FormationAccuracyReport:
    """分析单个 run 目录中稳定保持阶段的编队和跟踪性能。"""

    resolved_run_dir = Path(run_dir).resolve()
    snapshot_path = resolved_run_dir / "snapshots.jsonl"
    if not snapshot_path.is_file():
        raise FileNotFoundError(f"未找到仿真快照：{snapshot_path}")
    tight_radius_m, stable_hold_s = _read_thresholds(resolved_run_dir)
    source = load_snapshot_samples(
        snapshot_path,
        label=resolved_run_dir.name,
        channels=ACCURACY_CHANNELS,
    )
    # 能产生刚性槽位误差的节点天然是具有理论槽位的僚机。
    rigid_norm_by_node = {
        node_id: _aligned_norm_series(
            source, node_id, ("e_rigid_x", "e_rigid_y", "e_rigid_z")
        )
        for node_id in sorted(source.samples)
    }
    rigid_norm_by_node = {
        node_id: series for node_id, series in rigid_norm_by_node.items() if len(series[0])
    }
    follower_ids = tuple(rigid_norm_by_node)
    if not follower_ids:
        raise ValueError("日志中没有可计算编队位置误差的僚机")
    hold_start_s = _detect_hold_start(snapshot_path)
    team_max = _team_max_series(rigid_norm_by_node)
    # 稳定判定只查看 HOLD 之后，避免集结过程中的偶然入带导致提前统计。
    stable_start_s = None
    if hold_start_s is not None:
        stable_start_s = convergence_time_s(
            _slice_series(team_max, hold_start_s, source.t_max),
            tight_radius_m,
            stable_hold_s,
        )
    status = (
        "未进入保持"
        if hold_start_s is None
        else "保持未稳定"
        if stable_start_s is None
        else "稳定保持"
    )
    # 未稳定时不生成任何指标行，不能用零伪装成优秀结果。
    rows: list[dict[str, object]] = []
    if stable_start_s is not None:
        track_norm_by_node = {
            node_id: _aligned_norm_series(source, node_id, ("pos_x", "pos_y", "pos_z"))
            for node_id in follower_ids
        }
        # 每架僚机只输出一行，三轴只用于内部计算模长，不再展开到 Excel。
        for node_id in follower_ids:
            rows.append(
                _follower_metric_row(
                    resolved_run_dir.name,
                    node_id,
                    _summary_from_series(
                        _slice_series(rigid_norm_by_node[node_id], stable_start_s, source.t_max)
                    ),
                    _summary_from_series(
                        _slice_series(track_norm_by_node[node_id], stable_start_s, source.t_max)
                    ),
                )
            )
    return FormationAccuracyReport(
        run_id=resolved_run_dir.name,
        hold_start_s=hold_start_s,
        stable_start_s=stable_start_s,
        status=status,
        metric_rows=tuple(rows),
    )


def write_accuracy_report(report: FormationAccuracyReport, output_root: str | Path) -> Path:
    """只写出每架僚机一行的中文指标 CSV。"""

    # 每次运行使用独立子目录，重复分析同一 run 时原位刷新而不制造重复版本。
    output_dir = Path(output_root) / report.run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    # 清理旧版本附加报告，保证目录中只留下用户需要的指标表。
    for obsolete_name in ("formation_accuracy_summary.csv", "formation_accuracy.json"):
        (output_dir / obsolete_name).unlink(missing_ok=True)
    # 指标表每架僚机一行，Excel 打开后无需再按范围和指标筛选。
    with (output_dir / "formation_accuracy_detail.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=DETAIL_FIELD_NAMES)
        writer.writeheader()
        writer.writerows(report.metric_rows)
    return output_dir
