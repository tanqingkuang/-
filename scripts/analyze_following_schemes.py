"""生成空间队形跟随与航线里程跟随的对比指标和 SVG 图。"""

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

from src.algorithm.context.leaf_types import PosCalcStrategyE
from src.algorithm.entity.leader_follower import FOLLOWER_PROFILE
from src.algorithm.entity.leader_follower.follower import FollowerEntity
from src.algorithm.entity.types import EntityProfileS
from src.runner.sim_controller import SimulationController

_CENTER_EAST_M = 800.0
_CENTER_NORTH_M = 200.0
_TURN_RADIUS_M = 200.0
_SLOT_OFFSET_M = 100.0
_GRAVITY_MPS2 = 9.80665
_COLORS = {"空间队形跟随": "#ea580c", "航线里程跟随": "#2563eb"}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    """读取快照日志。"""
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _profile_for_pos_calc(strategy: PosCalcStrategyE) -> EntityProfileS:
    """复制僚机 Profile，并只替换编队飞行阶段的位置解算策略。"""

    route_changes = tuple(
        replace(
            change,
            strategies=replace(change.strategies, pos_calc=strategy),
        )
        if change.strategies.pos_calc == PosCalcStrategyE.ROUTE_FORMATION
        else change
        for change in FOLLOWER_PROFILE.route_changes
    )
    return replace(FOLLOWER_PROFILE, route_changes=route_changes)


def _run_strategy(config_path: Path, strategy: PosCalcStrategyE) -> list[dict[str, Any]]:
    """使用同一配置运行指定僚机策略，并从公开定时快照接口返回分析数据。"""

    profile = _profile_for_pos_calc(strategy)
    original_profile = FollowerEntity.PROFILE
    controller = SimulationController()
    try:
        # 离线对比只读取内存快照，不生成依赖时间戳的临时日志目录。
        controller.set_file_log_enabled(False)
        FollowerEntity.PROFILE = profile
        result = controller.run_until_complete(str(config_path), seed=0)
        if result.code != "OK":
            raise RuntimeError(f"{strategy.name} 仿真失败: {result.code} {result.message}")
        _cursor, snapshots = controller.read_timed_snapshots(None)
        return [asdict(snapshot) for snapshot in snapshots]
    finally:
        controller.close()
        FollowerEntity.PROFILE = original_profile


def _node(snapshot: dict[str, Any], node_id: str) -> dict[str, Any]:
    """按节点 ID 读取单帧状态。"""
    return next(node for node in snapshot["nodes"] if node["node_id"] == node_id)


def _rms(values: list[float]) -> float:
    """计算均方根。"""
    return math.sqrt(statistics.fmean(value * value for value in values))


def _arc_progress_m(node: dict[str, Any]) -> float:
    """把本试验左转圆弧上的点按径向投影为全局航线里程。"""
    angle = math.atan2(
        float(node["y_m"]) - _CENTER_NORTH_M,
        float(node["x_m"]) - _CENTER_EAST_M,
    )
    return 800.0 + _TURN_RADIUS_M * (angle + math.pi / 2.0)


def _scheme_metrics(rows: list[dict[str, Any]]) -> dict[str, float]:
    """计算单方案指标；机动窗口由实际航迹角速率超过 1 deg/s 自动识别。"""
    active_rows = [
        row for row in rows if abs(float(_node(row, "F01")["psi_dot_deg_s"])) > 1.0
    ]
    active_nodes = [_node(row, "F01") for row in active_rows]
    common_rows = [row for row in rows if 40.0 <= float(row["time_s"]) <= 50.7]
    common_followers = [_node(row, "F01") for row in common_rows]
    common_leaders = [_node(row, "L01") for row in common_rows]

    active_speeds = [float(node["ground_speed_mps"]) for node in active_nodes]
    active_accels = [
        abs(float(node["ground_speed_mps"]) * math.radians(float(node["psi_dot_deg_s"])))
        for node in active_nodes
    ]
    active_rolls = [abs(float(node["phi_deg"])) for node in active_nodes]
    active_track_errors = [
        math.hypot(float(node["track_pos_err_x_m"]), float(node["track_pos_err_z_m"]))
        for node in active_nodes
    ]
    radii = [
        math.hypot(
            float(node["x_m"]) - _CENTER_EAST_M,
            float(node["y_m"]) - _CENTER_NORTH_M,
        )
        for node in common_followers
    ]
    separations = [
        math.hypot(
            float(follower["x_m"]) - float(leader["x_m"]),
            float(follower["y_m"]) - float(leader["y_m"]),
        )
        for follower, leader in zip(common_followers, common_leaders)
    ]
    mileage_gaps = [
        _arc_progress_m(follower) - _arc_progress_m(leader)
        for follower, leader in zip(common_followers, common_leaders)
    ]
    rigid_errors: list[float] = []
    for follower, leader in zip(common_followers, common_leaders):
        speed = math.hypot(float(leader["vx_mps"]), float(leader["vy_mps"]))
        tangent_east = float(leader["vx_mps"]) / speed
        tangent_north = float(leader["vy_mps"]) / speed
        rigid_errors.append(
            math.hypot(
                float(follower["x_m"])
                - (float(leader["x_m"]) + _SLOT_OFFSET_M * tangent_east),
                float(follower["y_m"])
                - (float(leader["y_m"]) + _SLOT_OFFSET_M * tangent_north),
            )
        )
    minimum_separation = min(
        math.hypot(
            float(_node(row, "F01")["x_m"]) - float(_node(row, "L01")["x_m"]),
            float(_node(row, "F01")["y_m"]) - float(_node(row, "L01")["y_m"]),
        )
        for row in rows
    )
    return {
        "maneuver_start_s": float(active_rows[0]["time_s"]),
        "maneuver_end_s": float(active_rows[-1]["time_s"]),
        "maneuver_duration_s": float(active_rows[-1]["time_s"])
        - float(active_rows[0]["time_s"]),
        "maneuver_speed_mean_mps": statistics.fmean(active_speeds),
        "maneuver_speed_peak_mps": max(active_speeds),
        "maneuver_centripetal_accel_mean_mps2": statistics.fmean(active_accels),
        "maneuver_centripetal_accel_peak_mps2": max(active_accels),
        "maneuver_abs_roll_mean_deg": statistics.fmean(active_rolls),
        "maneuver_abs_roll_peak_deg": max(active_rolls),
        "maneuver_tracking_error_rms_m": _rms(active_track_errors),
        "maneuver_tracking_error_peak_m": max(active_track_errors),
        "common_turn_radius_mean_m": statistics.fmean(radii),
        "common_turn_route_deviation_abs_mean_m": statistics.fmean(
            abs(radius - _TURN_RADIUS_M) for radius in radii
        ),
        "common_turn_spatial_separation_mean_m": statistics.fmean(separations),
        "common_turn_route_mileage_gap_mean_m": statistics.fmean(mileage_gaps),
        "common_turn_rigid_slot_error_mean_m": statistics.fmean(rigid_errors),
        "whole_run_minimum_separation_m": minimum_separation,
    }


def _leader_max_difference(
    space_rows: list[dict[str, Any]], route_rows: list[dict[str, Any]]
) -> float:
    """核对两次试验的长机轨迹是否逐帧一致。"""
    fields = ("x_m", "y_m", "altitude_m", "vx_mps", "vy_mps", "vz_mps")
    return max(
        abs(float(_node(left, "L01")[field]) - float(_node(right, "L01")[field]))
        for left, right in zip(space_rows, route_rows)
        for field in fields
    )


def _polyline(
    points: list[tuple[float, float]],
    x_map: Callable[[float], float],
    y_map: Callable[[float], float],
) -> str:
    """把数据点转换为 SVG polyline 坐标串。"""
    return " ".join(f"{x_map(x):.1f},{y_map(y):.1f}" for x, y in points)


def _write_trajectory_svg(
    output: Path,
    space_rows: list[dict[str, Any]],
    route_rows: list[dict[str, Any]],
) -> None:
    """绘制转弯区域实际轨迹。"""
    width, height = 1000, 650
    left, top, plot_w, plot_h = 95, 80, 820, 490
    x_min, x_max, y_min, y_max = 620.0, 1060.0, -40.0, 400.0
    x_map = lambda value: left + (value - x_min) / (x_max - x_min) * plot_w
    y_map = lambda value: top + plot_h - (value - y_min) / (y_max - y_min) * plot_h
    leader = [
        (float(_node(row, "L01")["x_m"]), float(_node(row, "L01")["y_m"]))
        for row in space_rows
        if 28.0 <= float(row["time_s"]) <= 62.0
    ]
    schemes = {
        "空间队形跟随": [
            (float(_node(row, "F01")["x_m"]), float(_node(row, "F01")["y_m"]))
            for row in space_rows
            if 28.0 <= float(row["time_s"]) <= 62.0
        ],
        "航线里程跟随": [
            (float(_node(row, "F01")["x_m"]), float(_node(row, "F01")["y_m"]))
            for row in route_rows
            if 28.0 <= float(row["time_s"]) <= 62.0
        ],
    }
    planned: list[tuple[float, float]] = [(620.0, 0.0), (800.0, 0.0)]
    planned.extend(
        (
            _CENTER_EAST_M + _TURN_RADIUS_M * math.cos(-math.pi / 2.0 + index * math.pi / 80.0),
            _CENTER_NORTH_M + _TURN_RADIUS_M * math.sin(-math.pi / 2.0 + index * math.pi / 80.0),
        )
        for index in range(41)
    )
    planned.append((1000.0, 400.0))
    grid = []
    for tick in range(700, 1100, 100):
        grid.append(
            f'<line x1="{x_map(tick):.1f}" y1="{top}" x2="{x_map(tick):.1f}" y2="{top + plot_h}" class="grid"/>'
        )
        grid.append(f'<text x="{x_map(tick):.1f}" y="{top + plot_h + 28}" class="tick" text-anchor="middle">{tick}</text>')
    for tick in range(0, 401, 100):
        grid.append(
            f'<line x1="{left}" y1="{y_map(tick):.1f}" x2="{left + plot_w}" y2="{y_map(tick):.1f}" class="grid"/>'
        )
        grid.append(f'<text x="{left - 14}" y="{y_map(tick) + 5:.1f}" class="tick" text-anchor="end">{tick}</text>')
    lines = [
        f'<polyline points="{_polyline(planned, x_map, y_map)}" fill="none" stroke="#64748b" stroke-width="3" stroke-dasharray="9 7"/>',
        f'<polyline points="{_polyline(leader, x_map, y_map)}" fill="none" stroke="#111827" stroke-width="3"/>',
    ]
    for name, points in schemes.items():
        lines.append(
            f'<polyline points="{_polyline(points, x_map, y_map)}" fill="none" stroke="{_COLORS[name]}" stroke-width="4"/>'
        )
    output.write_text(
        f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>.title{{font:700 26px 'Microsoft YaHei';fill:#172033}}.label{{font:16px 'Microsoft YaHei';fill:#334155}}.tick{{font:14px 'Microsoft YaHei';fill:#475569}}.grid{{stroke:#e2e8f0;stroke-width:1}}</style>
<rect width="100%" height="100%" fill="#f8fafc"/><text x="500" y="42" text-anchor="middle" class="title">90° 转弯区域实际轨迹</text>
<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="white" stroke="#94a3b8"/>{''.join(grid)}{''.join(lines)}
<text x="505" y="628" text-anchor="middle" class="label">东向位置 / m</text><text x="24" y="325" text-anchor="middle" class="label" transform="rotate(-90 24 325)">北向位置 / m</text>
<line x1="170" y1="605" x2="205" y2="605" stroke="#111827" stroke-width="3"/><text x="212" y="611" class="label">长机（两组重合）</text>
<line x1="385" y1="605" x2="420" y2="605" stroke="#ea580c" stroke-width="4"/><text x="427" y="611" class="label">空间队形僚机</text>
<line x1="625" y1="605" x2="660" y2="605" stroke="#2563eb" stroke-width="4"/><text x="667" y="611" class="label">航线里程僚机</text>
</svg>\n''',
        encoding="utf-8",
        newline="\n",
    )


def _write_timeseries_svg(
    output: Path,
    space_rows: list[dict[str, Any]],
    route_rows: list[dict[str, Any]],
) -> None:
    """绘制僚机速度、滚转角和向心加速度时序。"""
    width, height = 1100, 860
    left, plot_w = 100, 900
    top_values = (90, 335, 580)
    plot_h = 180
    panels: tuple[tuple[str, str, float, float, Callable[[dict[str, Any]], float]], ...] = (
        ("地速", "m/s", 18.0, 25.0, lambda node: float(node["ground_speed_mps"])),
        ("滚转角绝对值", "deg", 0.0, 50.0, lambda node: abs(float(node["phi_deg"]))),
        (
            "航迹向心加速度",
            "m/s²",
            0.0,
            11.0,
            lambda node: abs(
                float(node["ground_speed_mps"]) * math.radians(float(node["psi_dot_deg_s"]))
            ),
        ),
    )
    x_map = lambda value: left + (value - 32.0) / 30.0 * plot_w
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>.title{font:700 26px "Microsoft YaHei";fill:#172033}.label{font:16px "Microsoft YaHei";fill:#334155}.tick{font:14px "Microsoft YaHei";fill:#475569}.grid{stroke:#e2e8f0;stroke-width:1}</style>',
        '<rect width="100%" height="100%" fill="#f8fafc"/><text x="550" y="42" text-anchor="middle" class="title">僚机转弯机动时序对比</text>',
    ]
    for panel_index, (title, unit, y_min, y_max, getter) in enumerate(panels):
        top = top_values[panel_index]
        y_map = lambda value, top=top, y_min=y_min, y_max=y_max: top + plot_h - (value - y_min) / (y_max - y_min) * plot_h
        svg.append(f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="white" stroke="#94a3b8"/>')
        svg.append(f'<text x="{left}" y="{top - 16}" class="label">{title} / {unit}</text>')
        for index in range(5):
            value = y_min + (y_max - y_min) * index / 4.0
            y = y_map(value)
            svg.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" class="grid"/>')
            svg.append(f'<text x="{left - 12}" y="{y + 5:.1f}" text-anchor="end" class="tick">{value:.1f}</text>')
        for name, rows in (("空间队形跟随", space_rows), ("航线里程跟随", route_rows)):
            points = [
                (float(row["time_s"]), getter(_node(row, "F01")))
                for row in rows
                if 32.0 <= float(row["time_s"]) <= 62.0
            ]
            svg.append(
                f'<polyline points="{_polyline(points, x_map, y_map)}" fill="none" stroke="{_COLORS[name]}" stroke-width="3"/>'
            )
        if panel_index == len(panels) - 1:
            for tick in range(32, 63, 5):
                svg.append(f'<text x="{x_map(tick):.1f}" y="{top + plot_h + 26}" text-anchor="middle" class="tick">{tick}</text>')
            svg.append(f'<text x="{left + plot_w / 2}" y="{top + plot_h + 55}" text-anchor="middle" class="label">仿真时间 / s</text>')
    svg.extend(
        [
            '<line x1="330" y1="825" x2="370" y2="825" stroke="#ea580c" stroke-width="3"/><text x="380" y="831" class="label">空间队形跟随</text>',
            '<line x1="590" y1="825" x2="630" y2="825" stroke="#2563eb" stroke-width="3"/><text x="640" y="831" class="label">航线里程跟随</text>',
            '</svg>\n',
        ]
    )
    output.write_text("".join(svg), encoding="utf-8", newline="\n")


def main() -> int:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", type=Path, help="同一配置自动运行两种跟随策略")
    source.add_argument("--space", type=Path, help="空间队形方案 snapshots.jsonl")
    parser.add_argument("--route", type=Path, help="航线里程方案 snapshots.jsonl")
    parser.add_argument("--output-dir", type=Path, required=True, help="指标和 SVG 输出目录")
    args = parser.parse_args()
    if args.config is not None:
        if args.route is not None:
            parser.error("--config 不能与 --route 同时使用")
        space_rows = _run_strategy(args.config, PosCalcStrategyE.SLOT_GEOMETRY)
        route_rows = _run_strategy(args.config, PosCalcStrategyE.ROUTE_FORMATION)
        space_source = f"{args.config}#SLOT_GEOMETRY"
        route_source = f"{args.config}#ROUTE_FORMATION"
    else:
        if args.route is None:
            parser.error("使用 --space 时必须同时提供 --route")
        space_rows = _read_jsonl(args.space)
        route_rows = _read_jsonl(args.route)
        space_source = str(args.space)
        route_source = str(args.route)
    if len(space_rows) != len(route_rows):
        raise ValueError("两组日志帧数不一致")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    omega = 20.0 / _TURN_RADIUS_M
    space_radius = math.hypot(_TURN_RADIUS_M, _SLOT_OFFSET_M)
    payload = {
        "experiment": {
            "space_source": space_source,
            "route_source": route_source,
            "frames_per_run": len(space_rows),
            "common_turn_window_s": [40.0, 50.7],
            "leader_max_frame_difference": _leader_max_difference(space_rows, route_rows),
        },
        "theory": {
            "turn_radius_m": _TURN_RADIUS_M,
            "slot_offset_m": _SLOT_OFFSET_M,
            "leader_speed_mps": 20.0,
            "yaw_rate_rad_s": omega,
            "space_target_radius_m": space_radius,
            "space_target_speed_mps": omega * space_radius,
            "space_target_centripetal_accel_mps2": omega * omega * space_radius,
            "space_target_bank_deg": math.degrees(
                math.atan(omega * omega * space_radius / _GRAVITY_MPS2)
            ),
            "route_target_radius_m": _TURN_RADIUS_M,
            "route_target_speed_mps": 20.0,
            "route_target_centripetal_accel_mps2": 20.0 * 20.0 / _TURN_RADIUS_M,
            "route_target_bank_deg": math.degrees(
                math.atan((20.0 * 20.0 / _TURN_RADIUS_M) / _GRAVITY_MPS2)
            ),
            "route_chord_separation_m": 2.0
            * _TURN_RADIUS_M
            * math.sin(_SLOT_OFFSET_M / (2.0 * _TURN_RADIUS_M)),
        },
        "measured": {
            "空间队形跟随": _scheme_metrics(space_rows),
            "航线里程跟随": _scheme_metrics(route_rows),
        },
    }
    (args.output_dir / "metrics.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    _write_trajectory_svg(args.output_dir / "实际轨迹对比.svg", space_rows, route_rows)
    _write_timeseries_svg(args.output_dir / "转弯机动时序.svg", space_rows, route_rows)
    print(f"analysis=PASS output={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
