"""T3 紧凑轨迹提取。注意：输出不包含 run_dir、时间戳等易变信息。"""

from __future__ import annotations

import json
from typing import Any

from tests.st.support import thresholds


def extract_trajectory(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    """从快照列表提取紧凑轨迹。注意：同一输入两次提取应逐字节一致。"""

    fields = list(thresholds.T3_FIELDS)
    # 每个整秒桶先选出与桶时刻最接近的帧：日志采样快于 10Hz 时容差窗口内会有多帧，
    # 若按"先到先占"取帧，0.95s 的状态会被标成 1.0s，制造约一个采样周期的假差异。
    best_by_bucket: dict[float, tuple[float, dict[str, Any]]] = {}
    for snapshot in snapshots:
        time_s = float(snapshot.get("time_s", 0.0))
        bucket = round(time_s / thresholds.T3_SAMPLE_PERIOD_S)
        sample_time = round(bucket * thresholds.T3_SAMPLE_PERIOD_S, 3)
        if sample_time <= 0.0:
            continue
        distance = abs(time_s - sample_time)
        if distance > thresholds.LOG_SAMPLE_PERIOD_S / 2.0 + 1e-9:
            continue
        current_best = best_by_bucket.get(sample_time)
        # 距离相等时保留先出现的帧，与旧行为在 10Hz 网格下逐字节一致。
        if current_best is None or distance < current_best[0] - 1e-12:
            best_by_bucket[sample_time] = (distance, snapshot)
    samples: list[dict[str, Any]] = []
    for sample_time in sorted(best_by_bucket):
        snapshot = best_by_bucket[sample_time][1]
        nodes = sorted(
            (node for node in snapshot.get("nodes", []) if isinstance(node, dict)),
            key=lambda item: str(item.get("node_id", "")),
        )
        compact_nodes: list[dict[str, Any]] = []
        for node in nodes:
            compact = {"node_id": str(node.get("node_id", ""))}
            for field in fields:
                compact[field] = _round_field(field, node.get(field, 0.0))
            compact_nodes.append(compact)
        samples.append({"time_s": sample_time, "nodes": compact_nodes})
    return {
        "sample_period_s": thresholds.T3_SAMPLE_PERIOD_S,
        "fields": fields,
        "samples": samples,
    }


def trajectory_json_bytes(trajectory: dict[str, Any]) -> bytes:
    """序列化紧凑轨迹。注意：固定分隔符以保证逐字节稳定。"""

    return json.dumps(trajectory, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _round_field(field: str, value: Any) -> float:
    """按字段语义舍入。注意：位置、角度和速度使用不同阈值表位数。"""

    number = float(value)
    if field in {"x_m", "y_m", "altitude_m"}:
        return round(number, thresholds.T3_POSITION_DECIMALS)
    if field.endswith("_deg"):
        return round(number, thresholds.T3_ANGLE_DECIMALS)
    if field.endswith("_mps"):
        return round(number, thresholds.T3_SPEED_DECIMALS)
    return round(number, 6)
