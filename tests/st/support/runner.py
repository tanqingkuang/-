"""ST 场景运行辅助。注意：唯一封装 SimulationController 私有 logger 访问。"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.algorithm.units.process.tra_plan.avoidance.obstacle import make_circle, make_polygon, make_rect
from src.algorithm.units.process.tra_plan.avoidance.planner import plan_avoidance_route
from src.data.config_loader import resolve_config_references
from src.runner.sim_controller import SimulationController
from src.runner.sim_control_types import CommandResult


@dataclass(frozen=True)
class ScenarioRun:
    """单次场景运行结果。注意：run_dir 可能在配置加载失败时为空。"""

    scenario: str
    config_path: Path
    result: CommandResult
    run_dir: Path | None
    snapshots: list[dict[str, Any]]
    events: list[dict[str, Any]]
    config: dict[str, Any]
    wall_time_s: float


def _controller_run_dir(controller: SimulationController) -> Path | None:
    """读取控制器本次日志目录。注意：这是 support 内唯一允许触碰私有 logger 的位置。"""

    logger = getattr(controller, "_logger")
    run_dir = getattr(logger, "run_dir", None)
    return Path(run_dir) if run_dir is not None else None


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    """读取 JSONL 文件。注意：空文件或不存在时返回空列表。"""

    if not path.exists():
        return []
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            data = json.loads(line)
            if isinstance(data, dict):
                records.append(data)
    return records


def run_scenario(config_path: str | Path, *, scenario: str | None = None, seed: int | None = None) -> ScenarioRun:
    """运行一个 ST 场景并解析日志。注意：兼容层把旧场景 seed 转成显式运行入参。"""

    path = Path(config_path)
    scenario_name = scenario or path.stem
    # ST 基线历史上把 seed 写在场景文件中；这里只负责读取并显式传给控制器，生产入口不再读取。
    scenario_config = json.loads(path.read_text(encoding="utf-8"))
    run_seed = int(scenario_config.get("seed", 0)) if seed is None else seed
    started = time.perf_counter()
    controller = SimulationController()
    # 文件日志默认关闭（见 log_enabled 开关），但 ST 三层检查依赖 snapshots/events 落盘，
    # 因此在 ST 入口无条件强制开启，场景配置无需各自声明 log_enabled。
    controller.set_file_log_enabled(True)
    try:
        if _needs_planned_avoidance(path):
            result = _run_with_planned_avoidance(controller, path, run_seed)
        else:
            result = controller.run_until_complete(str(path), seed=run_seed)
        run_dir = _controller_run_dir(controller)
        controller.close()
        wall_time_s = time.perf_counter() - started
    finally:
        try:
            controller.close()
        except Exception:
            pass
    if result.code == "OK" and run_dir is not None:
        config = json.loads((run_dir / "config.json").read_text(encoding="utf-8"))
        snapshots = read_jsonl(run_dir / f"snapshots_seed_{int(config.get('seed', 0))}.jsonl")
        events = read_jsonl(run_dir / "events.jsonl")
    else:
        snapshots = []
        events = []
        config = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    return ScenarioRun(scenario_name, path, result, run_dir, snapshots, events, config, wall_time_s)


def _needs_planned_avoidance(path: Path) -> bool:
    """判断场景是否需要先生成避障航线。注意：当前只对 ST 避障配置启用。"""

    data = json.loads(path.read_text(encoding="utf-8"))
    avoidance = data.get("avoidance")
    return isinstance(avoidance, dict) and bool(avoidance.get("enabled"))


def _run_with_planned_avoidance(
    controller: SimulationController,
    path: Path,
    seed: int,
) -> CommandResult:
    """规划并采用避障航线后运行到结束。注意：全程使用公开控制器接口推进。"""

    raw = json.loads(path.read_text(encoding="utf-8"))
    resolved = resolve_config_references(raw, path)
    plan = _build_avoidance_plan(resolved)
    if plan.code != "OK":
        return CommandResult("ERR_CONFIG_INVALID", f"avoidance plan failed: {plan.code} {plan.detail}")
    load = controller.load_config(str(path), seed=seed)
    if load.code != "OK":
        return load
    apply = controller.apply_avoidance_route(plan.route or [])
    if apply.code != "OK":
        return apply
    max_steps = int(float(resolved.get("duration_s", 0.0)) / float(resolved.get("step_s", 0.02))) + 5
    for _ in range(max_steps):
        snapshot = controller.get_snapshot()
        if snapshot.run_state == "FINISHED":
            return CommandResult("OK", "finished")
        step = controller.step(1)
        if step.code != "OK":
            return step
    return CommandResult("ERR_TICK_FAILED", "avoidance scenario did not finish within max_steps")


def _build_avoidance_plan(config: dict[str, Any]):
    """从已解析配置生成避障规划。注意：只消费 ST 需要的圆/多边形障碍字段。"""

    route = config.get("route", {}) if isinstance(config.get("route"), dict) else {}
    avoidance = config.get("avoidance", {}) if isinstance(config.get("avoidance"), dict) else {}
    waypoints = []
    for point in route.get("waypoints", []):
        if isinstance(point, dict):
            waypoints.append((float(point.get("x_m", 0.0)), float(point.get("y_m", 0.0)), float(point.get("altitude_m", 0.0))))
    obstacles = [_to_backend_obstacle(item) for item in avoidance.get("obstacles", []) if isinstance(item, dict) and item.get("enabled", True)]
    return plan_avoidance_route(
        waypoints,
        [item for item in obstacles if item is not None],
        turn_radius_m=float(avoidance.get("turn_radius_m", 0.0)),
        leg_margin_m=float(avoidance.get("leg_length_margin_m", 0.0)),
        clearance_m=float(avoidance.get("clearance_m", 0.0)),
        speed_mps=float(route.get("speed_mps", 0.0)),
        resolution_m=float((avoidance.get("grid") or {}).get("resolution_m", 50.0)) if isinstance(avoidance.get("grid"), dict) else 50.0,
        simplify_clearance_m=float(avoidance.get("simplify_clearance_m", avoidance.get("clearance_m", 0.0))),
        turn_switch_penalty_m=float(avoidance.get("turn_switch_penalty_m", 0.0)),
        turn_angle_weight_m=float(avoidance.get("turn_angle_weight_m", 0.0)),
        margin_m=float((avoidance.get("grid") or {}).get("margin_m", 0.0)) if isinstance(avoidance.get("grid"), dict) else 0.0,
        allow_arc=bool(avoidance.get("allow_arc", True)),
    )


def _to_backend_obstacle(raw: dict[str, Any]):
    """转换避障障碍为后端对象。注意：未知形状返回 None。"""

    obstacle_id = str(raw.get("id", "OB"))
    kind = str(raw.get("type", raw.get("kind", "circle")))
    if kind == "circle":
        center = raw.get("center", {}) if isinstance(raw.get("center"), dict) else {}
        return make_circle(obstacle_id, float(center.get("east_m", 0.0)), float(center.get("north_m", 0.0)), float(raw.get("radius_m", raw.get("radius", 0.0))))
    if kind == "rect":
        lo = raw.get("min", {}) if isinstance(raw.get("min"), dict) else {}
        hi = raw.get("max", {}) if isinstance(raw.get("max"), dict) else {}
        return make_rect(obstacle_id, float(lo.get("east_m", 0.0)), float(lo.get("north_m", 0.0)), float(hi.get("east_m", 0.0)), float(hi.get("north_m", 0.0)))
    if kind == "polygon" and isinstance(raw.get("vertices"), list):
        vertices = [(float(point.get("east_m", 0.0)), float(point.get("north_m", 0.0))) for point in raw["vertices"] if isinstance(point, dict)]
        return make_polygon(obstacle_id, vertices) if len(vertices) >= 3 else None
    return None
