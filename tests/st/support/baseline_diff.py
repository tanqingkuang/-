"""T3 紧凑基线比对。注意：只输出 top-10 差异用于定位。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from tests.st.support import thresholds

UPDATE_BASELINE_HINT = "若本次改动涉及算法属预期失败，运行 `python scripts/run_st.py --update-baseline` 刷新"


@dataclass(frozen=True)
class TrajectoryDiff:
    """一条紧凑轨迹差异。注意：字段足够生成一行定位报告。"""

    time_s: float
    node_id: str
    field: str
    baseline: Any
    current: Any
    delta: float | None


def diff_trajectory(baseline: dict[str, Any], current: dict[str, Any], *, top_n: int = 10) -> list[TrajectoryDiff]:
    """逐点比对紧凑轨迹。注意：结构缺失也会转成定位差异。"""

    diffs: list[TrajectoryDiff] = []
    base_samples = baseline.get("samples", []) if isinstance(baseline, dict) else []
    curr_samples = current.get("samples", []) if isinstance(current, dict) else []
    max_len = max(len(base_samples), len(curr_samples))
    for index in range(max_len):
        if index >= len(base_samples):
            sample = curr_samples[index]
            diffs.append(TrajectoryDiff(float(sample.get("time_s", -1.0)), "*", "sample", None, "extra", None))
            continue
        if index >= len(curr_samples):
            sample = base_samples[index]
            diffs.append(TrajectoryDiff(float(sample.get("time_s", -1.0)), "*", "sample", "missing", None, None))
            continue
        _diff_sample(base_samples[index], curr_samples[index], diffs)
    return sorted(diffs, key=_diff_sort_key)[:top_n]


def _diff_sample(baseline: dict[str, Any], current: dict[str, Any], diffs: list[TrajectoryDiff]) -> None:
    """比对单个采样点。注意：节点按 node_id 对齐。"""

    time_s = float(baseline.get("time_s", current.get("time_s", -1.0)))
    if abs(float(baseline.get("time_s", -1.0)) - float(current.get("time_s", -1.0))) > thresholds.T3_COMPARE_EPS:
        diffs.append(TrajectoryDiff(time_s, "*", "time_s", baseline.get("time_s"), current.get("time_s"), None))
    base_nodes = {str(node.get("node_id", "")): node for node in baseline.get("nodes", []) if isinstance(node, dict)}
    curr_nodes = {str(node.get("node_id", "")): node for node in current.get("nodes", []) if isinstance(node, dict)}
    for node_id in sorted(set(base_nodes) | set(curr_nodes)):
        if node_id not in base_nodes:
            diffs.append(TrajectoryDiff(time_s, node_id, "node", None, "extra", None))
            continue
        if node_id not in curr_nodes:
            diffs.append(TrajectoryDiff(time_s, node_id, "node", "missing", None, None))
            continue
        for field in thresholds.T3_FIELDS:
            base_value = base_nodes[node_id].get(field)
            curr_value = curr_nodes[node_id].get(field)
            delta = _numeric_delta(base_value, curr_value)
            if delta is None:
                if base_value != curr_value:
                    diffs.append(TrajectoryDiff(time_s, node_id, field, base_value, curr_value, None))
            elif abs(delta) > thresholds.T3_COMPARE_EPS:
                diffs.append(TrajectoryDiff(time_s, node_id, field, base_value, curr_value, delta))


def _numeric_delta(left: Any, right: Any) -> float | None:
    """计算数值差。注意：非数值字段返回 None。"""

    try:
        return float(right) - float(left)
    except (TypeError, ValueError):
        return None


def _diff_sort_key(diff: TrajectoryDiff) -> tuple[float, str, str, float]:
    """生成差异排序键。注意：优先按时间和差值大小定位。"""

    magnitude = abs(diff.delta) if diff.delta is not None else 0.0
    return (diff.time_s, diff.node_id, diff.field, -magnitude)


def format_diff(diff: TrajectoryDiff) -> str:
    """格式化一条轨迹差异。注意：供入口脚本追加到失败报告。"""

    delta = "" if diff.delta is None else f" diff={diff.delta:.3f}"
    return (
        f"@t={diff.time_s:.3f}s node={diff.node_id} field={diff.field} "
        f"baseline={diff.baseline} current={diff.current}{delta}"
    )
