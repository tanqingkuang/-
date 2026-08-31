"""运行三类非惯性补偿消融试验并生成精简指标与对比图。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.algorithm.context.leaf_types import PosTrackStrategyE
from src.algorithm.entity.leader_follower import FOLLOWER_PROFILE
from src.algorithm.entity.leader_follower.follower import FollowerEntity
from src.algorithm.entity.types import EntityProfileS
from src.data.config_loader import resolve_config_references
from src.runner.sim_controller import SimulationController


Row = dict[str, Any]
Selector = Callable[[list[Row]], list[Row]]


def _node(row: Row, node_id: str) -> Row:
    """按节点 ID 读取单帧状态。"""
    return next(node for node in row["nodes"] if node["node_id"] == node_id)


def _followers(row: Row) -> list[Row]:
    """取得单帧全部僚机。"""
    return [node for node in row["nodes"] if node["role"] == "wingman"]


def _rms(values: list[float]) -> float:
    """计算均方根。"""
    return math.sqrt(statistics.fmean(value * value for value in values))


def _position_error(node: Row) -> float:
    """计算三维槽位位置误差。"""
    return math.sqrt(
        float(node["pos_err_east_m"]) ** 2
        + float(node["pos_err_north_m"]) ** 2
        + float(node["pos_err_h_m"]) ** 2
    )


def _profile_for_mode(mode: str) -> EntityProfileS:
    """按消融模式复制僚机 Profile，并只替换编队飞行阶段的位置跟踪策略。"""
    strategies = {
        "none": PosTrackStrategyE.PID_POSITION_NO_INERTIAL,
        "rotation": PosTrackStrategyE.PID_POSITION,
        "full": PosTrackStrategyE.PID_POSITION_FULL_INERTIAL,
    }
    try:
        target = strategies[mode]
    except KeyError as exc:
        raise ValueError(f"不支持的非惯性补偿模式: {mode!r}") from exc
    return replace(
        FOLLOWER_PROFILE,
        route_changes=tuple(
            replace(change, strategies=replace(change.strategies, pos_track=target))
            if change.strategies.pos_track == PosTrackStrategyE.PID_POSITION
            else change
            for change in FOLLOWER_PROFILE.route_changes
        ),
    )


def _run(config_path: Path, mode: str) -> list[Row]:
    """以内存日志运行一个补偿模式，返回可序列化快照。"""
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    config = resolve_config_references(raw, config_path)
    profile = _profile_for_mode(mode)
    original_profile = FollowerEntity.PROFILE
    controller = SimulationController()
    controller.set_file_log_enabled(False)
    try:
        FollowerEntity.PROFILE = profile
        result = controller.run_until_complete(config, seed=int(raw.get("seed", 0)))
        if result.code != "OK":
            raise RuntimeError(f"{config_path.name}/{mode}: {result.code} {result.message}")
        _, snapshots = controller.read_timed_snapshots(None)
        return [asdict(snapshot) for snapshot in snapshots]
    finally:
        controller.close()
        FollowerEntity.PROFILE = original_profile


def _turn_rows(rows: list[Row]) -> list[Row]:
    """选择稳定圆弧帧。"""
    selected = [row for row in rows if row.get("route") and float(row["route"]["turn_sign"]) != 0.0]
    if not selected:
        raise ValueError("匀速转弯场景未产生圆弧帧")
    return selected


def _leader_acceleration_rows(rows: list[Row]) -> list[Row]:
    """选择长机明显加减速帧并扩展两秒响应窗口。"""
    active: set[int] = set()
    if len(rows) < 2:
        return rows
    dt_s = float(rows[1]["time_s"]) - float(rows[0]["time_s"])
    margin = max(1, round(2.0 / dt_s))
    for index, row in enumerate(rows):
        leader = next(node for node in row["nodes"] if node["role"] == "leader")
        acceleration = math.hypot(
            float(leader["cmd_acc_east_mps2"]),
            float(leader["cmd_acc_north_mps2"]),
        )
        if acceleration >= 0.5 and float(row["time_s"]) > 2.0:
            active.update(range(max(0, index - margin), min(len(rows), index + margin + 1)))
    selected = [rows[index] for index in sorted(active)]
    if not selected:
        raise ValueError("直线场景未识别到长机加减速窗口")
    return selected


def _yaw_rate_transition_rows(rows: list[Row]) -> list[Row]:
    """选择航段转向符号变化前后两秒，覆盖角速度建立和反向过程。"""
    if len(rows) < 2:
        return rows
    dt_s = float(rows[1]["time_s"]) - float(rows[0]["time_s"])
    margin = max(1, round(2.0 / dt_s))
    active: set[int] = set()
    previous_sign = float(rows[0]["route"]["turn_sign"]) if rows[0].get("route") else 0.0
    for index, row in enumerate(rows[1:], start=1):
        sign = float(row["route"]["turn_sign"]) if row.get("route") else 0.0
        if sign != previous_sign:
            active.update(range(max(0, index - margin), min(len(rows), index + margin + 1)))
        previous_sign = sign
    selected = [rows[index] for index in sorted(active)]
    if not selected:
        raise ValueError("S弯场景未识别到变转率窗口")
    return selected


def _metrics(rows: list[Row]) -> dict[str, float | int]:
    """汇总所选窗口内的槽位误差与控制代价。"""
    followers = [node for row in rows for node in _followers(row)]
    errors = [_position_error(node) for node in followers]
    accelerations = [
        math.hypot(float(node["cmd_acc_east_mps2"]), float(node["cmd_acc_north_mps2"]))
        for node in followers
    ]
    return {
        "frames": len(rows),
        "samples": len(followers),
        "position_error_rms_m": _rms(errors),
        "position_error_peak_m": max(errors),
        "horizontal_acceleration_rms_mps2": _rms(accelerations),
        "horizontal_acceleration_peak_mps2": max(accelerations),
    }


def _leader_max_difference(left: list[Row], right: list[Row]) -> float:
    """核对两组试验长机轨迹一致性。"""
    if len(left) != len(right):
        raise ValueError("对照组快照帧数不一致")
    fields = ("x_m", "y_m", "altitude_m", "ground_speed_mps", "psi_dot_deg_s")
    maximum = 0.0
    for left_row, right_row in zip(left, right):
        left_leader = next(node for node in left_row["nodes"] if node["role"] == "leader")
        right_leader = next(node for node in right_row["nodes"] if node["role"] == "leader")
        maximum = max(
            maximum,
            *(abs(float(left_leader[field]) - float(right_leader[field])) for field in fields),
        )
    return maximum


def _experiment(
    name: str,
    config_path: Path,
    baseline_mode: str,
    compensated_mode: str,
    selector: Selector,
    *,
    capture_rows: bool = False,
) -> dict[str, Any]:
    """运行单项成对消融并计算改善比例。"""
    baseline_all = _run(config_path, baseline_mode)
    compensated_all = _run(config_path, compensated_mode)
    baseline_rows = selector(baseline_all)
    compensated_rows = selector(compensated_all)
    baseline = _metrics(baseline_rows)
    compensated = _metrics(compensated_rows)
    before = float(baseline["position_error_rms_m"])
    after = float(compensated["position_error_rms_m"])
    result: dict[str, Any] = {
        "name": name,
        "config": str(config_path.relative_to(ROOT)),
        "baseline_mode": baseline_mode,
        "compensated_mode": compensated_mode,
        "leader_max_state_difference": _leader_max_difference(baseline_all, compensated_all),
        "baseline": baseline,
        "compensated": compensated,
        "error_reduction_percent": 100.0 * (before - after) / before if before > 0.0 else 0.0,
    }
    if capture_rows:
        # 轨迹图使用完整仿真过程；指标仍只统计 selector 选出的特征窗口。
        result["_baseline_rows"] = baseline_all
        result["_compensated_rows"] = compensated_all
    return result


def _write_svg(path: Path, experiments: list[dict[str, Any]]) -> None:
    """绘制三项试验槽位误差 RMS 对比柱状图。"""
    width, height = 960, 520
    left, top, plot_w, plot_h = 90, 80, 800, 330
    values = [
        float(experiment[group]["position_error_rms_m"])
        for experiment in experiments
        for group in ("baseline", "compensated")
    ]
    maximum = max(values) * 1.15 or 1.0
    group_w = plot_w / len(experiments)
    bar_w = 72.0
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>.title{font:700 25px "Microsoft YaHei";fill:#172033}.label{font:16px "Microsoft YaHei";fill:#334155}.value{font:14px "Microsoft YaHei";fill:#334155}.grid{stroke:#e2e8f0;stroke-width:1}</style>',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="480" y="42" text-anchor="middle" class="title">三类补偿试验槽位误差 RMS</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="white" stroke="#94a3b8"/>',
    ]
    for tick in range(5):
        value = maximum * tick / 4.0
        y = top + plot_h - value / maximum * plot_h
        svg.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" class="grid"/>')
        svg.append(f'<text x="{left - 10}" y="{y + 5:.1f}" text-anchor="end" class="value">{value:.1f}</text>')
    for index, experiment in enumerate(experiments):
        center = left + (index + 0.5) * group_w
        for offset, key, color in ((-0.58, "baseline", "#ea580c"), (0.08, "compensated", "#2563eb")):
            value = float(experiment[key]["position_error_rms_m"])
            x = center + offset * bar_w
            y = top + plot_h - value / maximum * plot_h
            svg.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{top + plot_h - y:.1f}" fill="{color}"/>')
            svg.append(f'<text x="{x + bar_w / 2:.1f}" y="{y - 7:.1f}" text-anchor="middle" class="value">{value:.2f}</text>')
        svg.append(f'<text x="{center:.1f}" y="{top + plot_h + 30}" text-anchor="middle" class="label">{experiment["name"]}</text>')
    svg.extend([
        '<rect x="310" y="470" width="32" height="12" fill="#ea580c"/><text x="352" y="482" class="label">消融基线</text>',
        '<rect x="520" y="470" width="32" height="12" fill="#2563eb"/><text x="562" y="482" class="label">补偿组</text>',
        '</svg>\n',
    ])
    path.write_text("".join(svg), encoding="utf-8", newline="\n")


def _write_trajectory_svg(
    path: Path,
    baseline_rows: list[Row],
    compensated_rows: list[Row],
    *,
    title: str,
    baseline_title: str,
    compensated_title: str,
) -> None:
    """用相同坐标比例并排绘制两组真实飞行轨迹。"""
    width, height = 1120, 560
    panel_top, panel_w, panel_h = 85, 470, 390
    panel_lefts = (80, 630)
    node_ids = ("A01", "A02", "A03")
    colors = {"A01": "#2563eb", "A02": "#16a34a", "A03": "#ea580c"}
    all_rows = baseline_rows + compensated_rows
    all_nodes = [node for row in all_rows for node in row["nodes"] if node["node_id"] in node_ids]
    min_east = min(float(node["x_m"]) for node in all_nodes)
    max_east = max(float(node["x_m"]) for node in all_nodes)
    min_north = min(float(node["y_m"]) for node in all_nodes)
    max_north = max(float(node["y_m"]) for node in all_nodes)
    east_span = max(max_east - min_east, 1.0)
    north_span = max(max_north - min_north, 1.0)
    padding_m = 0.08 * max(east_span, north_span)
    min_east -= padding_m
    max_east += padding_m
    min_north -= padding_m
    max_north += padding_m
    east_span = max_east - min_east
    north_span = max_north - min_north
    scale = min(panel_w / east_span, panel_h / north_span)
    draw_w = east_span * scale
    draw_h = north_span * scale

    def map_point(east_m: float, north_m: float, panel_left: float) -> tuple[float, float]:
        """以全局等比例坐标映射真实东北位置。"""
        offset_x = panel_left + (panel_w - draw_w) / 2.0
        offset_y = panel_top + (panel_h - draw_h) / 2.0
        return (
            offset_x + (east_m - min_east) * scale,
            offset_y + (max_north - north_m) * scale,
        )

    def trajectory_points(rows: list[Row], node_id: str, panel_left: float) -> str:
        """提取指定飞机实际位置并生成 SVG 折线点。"""
        points = []
        for row in rows:
            node = _node(row, node_id)
            x, y = map_point(float(node["x_m"]), float(node["y_m"]), panel_left)
            points.append(f"{x:.1f},{y:.1f}")
        return " ".join(points)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>.title{font:700 25px "Microsoft YaHei";fill:#172033}.panel{font:700 19px "Microsoft YaHei";fill:#334155}.label{font:15px "Microsoft YaHei";fill:#475569}.node{font:700 14px "Microsoft YaHei"}</style>',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="560" y="42" text-anchor="middle" class="title">{title}</text>',
    ]
    for panel_left, panel_title, rows in (
        (panel_lefts[0], baseline_title, baseline_rows),
        (panel_lefts[1], compensated_title, compensated_rows),
    ):
        svg.append(f'<text x="{panel_left + panel_w / 2:.1f}" y="70" text-anchor="middle" class="panel">{panel_title}</text>')
        svg.append(f'<rect x="{panel_left}" y="{panel_top}" width="{panel_w}" height="{panel_h}" fill="white" stroke="#94a3b8"/>')
        for node_id in node_ids:
            points = trajectory_points(rows, node_id, panel_left)
            svg.append(
                f'<polyline points="{points}" fill="none" stroke="{colors[node_id]}" '
                f'stroke-width="{3.0 if node_id == "A01" else 2.4}"/>'
            )
            last = _node(rows[-1], node_id)
            label_x, label_y = map_point(float(last["x_m"]), float(last["y_m"]), panel_left)
            svg.append(
                f'<text x="{label_x + 7:.1f}" y="{label_y - 7:.1f}" '
                f'class="node" fill="{colors[node_id]}">{node_id}</text>'
            )
            marker_step = max(1, len(rows) // 10)
            for row in rows[::marker_step]:
                marker = _node(row, node_id)
                marker_x, marker_y = map_point(
                    float(marker["x_m"]),
                    float(marker["y_m"]),
                    panel_left,
                )
                svg.append(
                    f'<circle cx="{marker_x:.1f}" cy="{marker_y:.1f}" r="2.4" '
                    f'fill="{colors[node_id]}" opacity="0.65"/>'
                )
    svg.extend(
        [
            '<text x="560" y="515" text-anchor="middle" class="label">实际东北位置使用相同米制比例；圆点为等时间位置标记</text>',
            '<line x1="405" y1="540" x2="435" y2="540" stroke="#2563eb" stroke-width="3"/><text x="445" y="545" class="label">A01 长机</text>',
            '<line x1="535" y1="540" x2="565" y2="540" stroke="#16a34a" stroke-width="3"/><text x="575" y="545" class="label">A02 僚机</text>',
            '<line x1="665" y1="540" x2="695" y2="540" stroke="#ea580c" stroke-width="3"/><text x="705" y="545" class="label">A03 僚机</text>',
            '</svg>\n',
        ]
    )
    path.write_text("".join(svg), encoding="utf-8", newline="\n")


def _write_straight_overlay_svg(
    path: Path,
    baseline_rows: list[Row],
    compensated_rows: list[Row],
) -> None:
    """在同一坐标图叠加直线加减速差异最大窗口的真实轨迹。"""
    width, height = 1120, 500
    left, top, plot_w, plot_h = 70, 105, 980, 285
    node_ids = ("A01", "A02", "A03")
    colors = {"A01": "#2563eb", "A02": "#16a34a", "A03": "#ea580c"}
    pair_count = min(len(baseline_rows), len(compensated_rows))
    peak_index = max(
        range(pair_count),
        key=lambda index: max(
            math.hypot(
                float(_node(baseline_rows[index], node_id)["x_m"])
                - float(_node(compensated_rows[index], node_id)["x_m"]),
                float(_node(baseline_rows[index], node_id)["y_m"])
                - float(_node(compensated_rows[index], node_id)["y_m"]),
            )
            for node_id in ("A02", "A03")
        ),
    )
    dt_s = (
        float(baseline_rows[1]["time_s"]) - float(baseline_rows[0]["time_s"])
        if len(baseline_rows) > 1
        else 0.02
    )
    margin_frames = max(1, round(3.0 / dt_s))
    start_index = max(0, peak_index - margin_frames)
    end_index = min(pair_count, peak_index + margin_frames + 1)
    baseline_rows = baseline_rows[start_index:end_index]
    compensated_rows = compensated_rows[start_index:end_index]
    window_start_s = float(baseline_rows[0]["time_s"])
    window_end_s = float(baseline_rows[-1]["time_s"])
    all_rows = baseline_rows + compensated_rows
    all_nodes = [node for row in all_rows for node in row["nodes"] if node["node_id"] in node_ids]
    min_east = min(float(node["x_m"]) for node in all_nodes)
    max_east = max(float(node["x_m"]) for node in all_nodes)
    min_north = min(float(node["y_m"]) for node in all_nodes)
    max_north = max(float(node["y_m"]) for node in all_nodes)
    padding_m = 0.06 * max(max_east - min_east, max_north - min_north, 1.0)
    min_east -= padding_m
    max_east += padding_m
    min_north -= padding_m
    max_north += padding_m
    east_span = max(max_east - min_east, 1.0)
    north_span = max(max_north - min_north, 1.0)
    scale = min(plot_w / east_span, plot_h / north_span)
    draw_w = east_span * scale
    draw_h = north_span * scale
    offset_x = left + (plot_w - draw_w) / 2.0
    offset_y = top + (plot_h - draw_h) / 2.0

    def map_point(east_m: float, north_m: float) -> tuple[float, float]:
        """把真实东北位置等比例映射到共用画布。"""
        return (
            offset_x + (east_m - min_east) * scale,
            offset_y + (max_north - north_m) * scale,
        )

    def trajectory_points(rows: list[Row], node_id: str) -> str:
        """生成指定飞机的实际轨迹折线坐标。"""
        points = []
        for row in rows:
            node = _node(row, node_id)
            x, y = map_point(float(node["x_m"]), float(node["y_m"]))
            points.append(f"{x:.1f},{y:.1f}")
        return " ".join(points)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>.title{font:700 25px "Microsoft YaHei";fill:#172033}.label{font:15px "Microsoft YaHei";fill:#475569}</style>',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="560" y="40" text-anchor="middle" class="title">直线加减速真实飞行轨迹局部叠加对比</text>',
        f'<text x="560" y="72" text-anchor="middle" class="label">差异最大时刻前后 3 s：t={window_start_s:.1f}~{window_end_s:.1f} s</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="white" stroke="#94a3b8"/>',
    ]
    for node_id in node_ids:
        color = colors[node_id]
        svg.append(
            f'<polyline points="{trajectory_points(baseline_rows, node_id)}" fill="none" '
            f'stroke="{color}" stroke-width="3.2" opacity="0.42"/>'
        )
        svg.append(
            f'<polyline points="{trajectory_points(compensated_rows, node_id)}" fill="none" '
            f'stroke="{color}" stroke-width="2.2" stroke-dasharray="8 5"/>'
        )

    marker_count = 10
    for marker_index in range(marker_count + 1):
        baseline_index = round(marker_index * (len(baseline_rows) - 1) / marker_count)
        compensated_index = round(marker_index * (len(compensated_rows) - 1) / marker_count)
        for node_id in node_ids:
            color = colors[node_id]
            baseline_node = _node(baseline_rows[baseline_index], node_id)
            bx, by = map_point(float(baseline_node["x_m"]), float(baseline_node["y_m"]))
            svg.append(
                f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="4.0" fill="white" '
                f'stroke="{color}" stroke-width="2"/>'
            )
            compensated_node = _node(compensated_rows[compensated_index], node_id)
            cx, cy = map_point(float(compensated_node["x_m"]), float(compensated_node["y_m"]))
            svg.append(
                f'<line x1="{bx:.1f}" y1="{by:.1f}" x2="{cx:.1f}" y2="{cy:.1f}" '
                f'stroke="{color}" stroke-width="1.2" opacity="0.45"/>'
            )
            svg.append(
                f'<rect x="{cx - 3.5:.1f}" y="{cy - 3.5:.1f}" width="7" height="7" '
                f'fill="{color}" stroke="white" stroke-width="0.8"/>'
            )

    svg.extend(
        [
            '<text x="560" y="420" text-anchor="middle" class="label">真实位置等比例局部放大；空心圆为原控制，实心方块为增加长机平动补偿</text>',
            '<line x1="250" y1="455" x2="280" y2="455" stroke="#2563eb" stroke-width="3"/><text x="290" y="460" class="label">A01 长机</text>',
            '<line x1="380" y1="455" x2="410" y2="455" stroke="#16a34a" stroke-width="3"/><text x="420" y="460" class="label">A02 僚机</text>',
            '<line x1="510" y1="455" x2="540" y2="455" stroke="#ea580c" stroke-width="3"/><text x="550" y="460" class="label">A03 僚机</text>',
            '<circle cx="690" cy="455" r="4" fill="white" stroke="#475569" stroke-width="2"/><text x="705" y="460" class="label">原控制</text>',
            '<rect x="795" y="451" width="8" height="8" fill="#475569"/><text x="812" y="460" class="label">增加补偿</text>',
            '</svg>\n',
        ]
    )
    path.write_text("".join(svg), encoding="utf-8", newline="\n")


def main() -> int:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "docs" / "非惯性补偿对比" / "assets",
        help="指标和图表输出目录",
    )
    args = parser.parse_args()
    data_dir = ROOT / "docs" / "非惯性补偿对比" / "data"
    turn_experiment = _experiment(
        "匀速转弯",
        data_dir / "匀速转弯.json",
        "none",
        "rotation",
        _turn_rows,
        capture_rows=True,
    )
    turn_baseline_rows = turn_experiment.pop("_baseline_rows")
    turn_compensated_rows = turn_experiment.pop("_compensated_rows")
    straight_experiment = _experiment(
        "直线加减速",
        data_dir / "直线加减速.json",
        "rotation",
        "full",
        _leader_acceleration_rows,
        capture_rows=True,
    )
    straight_baseline_rows = straight_experiment.pop("_baseline_rows")
    straight_compensated_rows = straight_experiment.pop("_compensated_rows")
    s_turn_experiment = _experiment(
        "S弯变转率",
        data_dir / "S弯变转率.json",
        "rotation",
        "full",
        _yaw_rate_transition_rows,
        capture_rows=True,
    )
    s_turn_baseline_rows = s_turn_experiment.pop("_baseline_rows")
    s_turn_compensated_rows = s_turn_experiment.pop("_compensated_rows")
    experiments = [
        turn_experiment,
        straight_experiment,
        s_turn_experiment,
    ]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    payload = {"experiments": experiments}
    (args.output_dir / "metrics.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    _write_svg(args.output_dir / "三类试验误差对比.svg", experiments)
    _write_trajectory_svg(
        args.output_dir / "匀速转弯真实轨迹对比.svg",
        turn_baseline_rows,
        turn_compensated_rows,
        title="匀速转弯真实飞行轨迹对比",
        baseline_title="未加非惯性补偿",
        compensated_title="加入非惯性补偿",
    )
    _write_straight_overlay_svg(
        args.output_dir / "直线加减速真实轨迹对比.svg",
        straight_baseline_rows,
        straight_compensated_rows,
    )
    _write_trajectory_svg(
        args.output_dir / "S弯变转率真实轨迹对比.svg",
        s_turn_baseline_rows,
        s_turn_compensated_rows,
        title="S弯变转率真实飞行轨迹对比",
        baseline_title="现有旋转补偿",
        compensated_title="完整非惯性补偿",
    )
    for experiment in experiments:
        print(
            f'{experiment["name"]}: '
            f'{experiment["baseline"]["position_error_rms_m"]:.3f} -> '
            f'{experiment["compensated"]["position_error_rms_m"]:.3f} m '
            f'({experiment["error_reduction_percent"]:.1f}%)'
        )
    print(f"analysis=PASS output={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
