"""pytest 形式的 ST 场景检查。注意：同一场景在本模块内只运行一次。"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import pytest

from tests.st.support.baseline_diff import diff_trajectory
from tests.st.support.invariants import run_invariants
from tests.st.support.metrics import compare_metrics, extract_metrics
from tests.st.support.runner import run_scenario
from tests.st.support.trajectory_extract import extract_trajectory

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = sorted((ROOT / "tests" / "st" / "scenarios").glob("st_*.json"))


@lru_cache(maxsize=None)
def _run(path_text: str):
    """缓存单场景仿真结果。注意：多个 pytest 用例共享同一次运行产物。"""

    path = Path(path_text)
    return run_scenario(path, scenario=path.stem)


@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda path: path.stem)
def test_t1_invariants(scenario_path: Path) -> None:
    """T1：不变量应全部通过。注意：失败明细由 support 返回。"""

    issues = run_invariants(_run(str(scenario_path)))
    assert issues == []


@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda path: path.stem)
def test_t2_metrics_regression(scenario_path: Path) -> None:
    """T2：指标只允许在容差内波动。注意：变好提示不构成失败。"""

    run = _run(str(scenario_path))
    baseline = json.loads((ROOT / "tests" / "st" / "baselines" / f"{scenario_path.stem}.json").read_text(encoding="utf-8"))
    comparisons = compare_metrics(scenario_path.stem, baseline["metrics"], extract_metrics(scenario_path.stem, run.snapshots, run.config))
    assert [item.issue for item in comparisons if item.issue is not None] == []


@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda path: path.stem)
def test_t3_compact_trajectory_baseline(scenario_path: Path) -> None:
    """T3：紧凑轨迹应与基线一致。注意：算法预期改动需刷新基线。"""

    run = _run(str(scenario_path))
    baseline = json.loads((ROOT / "tests" / "st" / "baselines" / f"{scenario_path.stem}.json").read_text(encoding="utf-8"))
    assert diff_trajectory(baseline["trajectory"], extract_trajectory(run.snapshots)) == []
