"""ST 支撑工具 LLT。注意：只使用微型夹具，不运行真实仿真。"""

from __future__ import annotations

import io
import json
from pathlib import Path

from scripts.run_st import main as run_st_main
from src.runner.sim_control_types import CommandResult
from tests.st.support.baseline_diff import diff_trajectory
from tests.st.support.invariants import check_dynamic_limits
from tests.st.support.runner import ScenarioRun, read_jsonl
from tests.st.support.trajectory_extract import extract_trajectory, trajectory_json_bytes

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests" / "llt" / "fixtures" / "st_snapshots.jsonl"


def _config() -> dict:
    """构造微型 ST 配置。注意：字段覆盖入口脚本会执行的不变量。"""

    return {
        "duration_s": 1.0,
        "step_s": 0.1,
        "route": {"speed_mps": 10.0, "waypoints": [{"x_m": 0.0, "y_m": 0.0}, {"x_m": 10.0, "y_m": 0.0}]},
        "model": {
            "gravity_mps2": 9.80665,
            "min_speed_mps": 1.0,
            "limits": {
                "acceleration_command_mps2": 20.0,
                "max_climb_rate_mps": 5.0,
                "max_descent_rate_mps": 5.0,
                "nx_min": -1.0,
                "nx_max": 1.0,
                "n_normal_min": 0.0,
                "n_normal_max": 4.0,
                "phi_min_deg": -70.0,
                "phi_max_deg": 70.0,
            },
        },
        "control": {"velocity_command_limits": {"forward_min_mps": 1.0, "forward_max_mps": 20.0}},
        "nodes": [{"node_id": "S01", "role": "leader"}],
        "links": [],
    }


def _make_root(tmp_path: Path) -> Path:
    """创建脚本契约测试用根目录。注意：只包含 ST 需要的最小目录。"""

    root = tmp_path
    scenarios = root / "tests" / "st" / "scenarios"
    scenarios.mkdir(parents=True)
    (scenarios / "st_fake.json").write_text("{}\n", encoding="utf-8")
    (root / "tests" / "st" / "baselines").mkdir(parents=True)
    return root


def _runner(config_path: Path, *, scenario: str):
    """入口脚本注入 runner。注意：返回固定夹具，不跑控制器。"""

    snapshots = read_jsonl(FIXTURE)
    return ScenarioRun(
        scenario=scenario,
        config_path=Path(config_path),
        result=CommandResult("OK", "finished"),
        run_dir=None,
        snapshots=snapshots,
        events=[],
        config=_config(),
        wall_time_s=0.01,
    )


def test_trajectory_extract_is_deterministic_and_normalized() -> None:
    """UT-11：提取器应降采样、舍入并剔除易变字段。"""

    snapshots = read_jsonl(FIXTURE)
    first = extract_trajectory(snapshots)
    second = extract_trajectory(snapshots)

    assert trajectory_json_bytes(first) == trajectory_json_bytes(second)
    assert first["fields"] == ["x_m", "y_m", "altitude_m", "psi_v_deg", "speed_mps"]
    assert first["samples"] == [
        {
            "time_s": 1.0,
            "nodes": [
                {"node_id": "S01", "x_m": 10.0, "y_m": 0.01, "altitude_m": 1000.0, "psi_v_deg": 0.0, "speed_mps": 10.0}
            ],
        }
    ]
    assert b"run-x" not in trajectory_json_bytes(first)
    assert b"drop-me" not in trajectory_json_bytes(first)


def test_baseline_diff_locates_point_and_ignores_tolerance() -> None:
    """UT-12：比对器应定位单点差异，并忽略容差内差异。"""

    baseline = extract_trajectory(read_jsonl(FIXTURE))
    changed = json.loads(json.dumps(baseline))
    changed["samples"][0]["nodes"][0]["x_m"] += 0.5

    diffs = diff_trajectory(baseline, changed)

    assert len(diffs) == 1
    assert diffs[0].time_s == 1.0
    assert diffs[0].node_id == "S01"
    assert diffs[0].field == "x_m"
    assert diffs[0].delta == 0.5

    tolerated = json.loads(json.dumps(baseline))
    tolerated["samples"][0]["nodes"][0]["x_m"] += 0.004
    assert diff_trajectory(baseline, tolerated) == []


def test_dynamic_load_limit_checks_normal_magnitude_not_signed_right_axis() -> None:
    """UT-03 应允许负的右向载荷分量，并只对法向合过载执行包线检查。"""

    snapshots = read_jsonl(FIXTURE)
    node = snapshots[-1]["nodes"][0]
    node.update({"ny": 0.0, "nz": -3.0, "n_normal": 3.0})
    run = ScenarioRun(
        scenario="st_fake",
        config_path=Path("st_fake.json"),
        result=CommandResult("OK", "finished"),
        run_dir=None,
        snapshots=[snapshots[-1]],
        events=[],
        config=_config(),
        wall_time_s=0.01,
    )

    assert check_dynamic_limits(run) == []

    node["n_normal"] = 4.5
    issues = check_dynamic_limits(run)
    assert len(issues) == 1
    assert issues[0].field == "n_normal"


def test_dynamic_yaw_limit_uses_air_rate_not_wind_affected_ground_rate() -> None:
    """横风可改变地面航迹率；UT-03 的滚转包线只能裁决空速航向率。"""

    snapshots = read_jsonl(FIXTURE)
    node = snapshots[-1]["nodes"][0]
    node.update({"psi_dot_deg_s": 9999.0, "air_psi_dot_deg_s": 0.0})
    run = ScenarioRun(
        scenario="st_fake",
        config_path=Path("st_fake.json"),
        result=CommandResult("OK", "finished"),
        run_dir=None,
        snapshots=[snapshots[-1]],
        events=[],
        config=_config(),
        wall_time_s=0.01,
    )

    assert check_dynamic_limits(run) == []

    node["air_psi_dot_deg_s"] = 9999.0
    issues = check_dynamic_limits(run)
    assert len(issues) == 1
    assert issues[0].field == "air_psi_dot_deg_s"


def test_run_st_contract_ok_failure_and_update_baseline(tmp_path: Path) -> None:
    """UT-13：入口脚本应覆盖 OK、失败和刷新基线后三种路径。"""

    root = _make_root(tmp_path)
    out = io.StringIO()
    assert run_st_main(["--update-baseline"], root=root, runner_func=_runner, stdout=out) == 0
    assert out.getvalue().strip() == "BASELINE UPDATED"

    out = io.StringIO()
    assert run_st_main([], root=root, runner_func=_runner, stdout=out) == 0
    assert out.getvalue().strip() == "ST OK"

    baseline_path = root / "tests" / "st" / "baselines" / "st_fake.json"
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    baseline["trajectory"]["samples"][0]["nodes"][0]["x_m"] += 0.5
    baseline_path.write_text(json.dumps(baseline, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    out = io.StringIO()
    assert run_st_main([], root=root, runner_func=_runner, stdout=out) == 1
    text = out.getvalue()
    assert "[st_fake][UT-10]" in text
    assert "@t=1.000s node=S01 field=x_m" in text
    assert "ST FAILED" in text

    out = io.StringIO()
    assert run_st_main(["--update-baseline"], root=root, runner_func=_runner, stdout=out) == 0
    out = io.StringIO()
    assert run_st_main([], root=root, runner_func=_runner, stdout=out) == 0
    assert out.getvalue().strip() == "ST OK"
