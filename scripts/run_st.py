"""ST 一键入口。注意：默认跑全部场景的 T1/T2/T3 三层检查。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, TextIO

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.st.support.baseline_diff import UPDATE_BASELINE_HINT, diff_trajectory, format_diff
from tests.st.support.invariants import run_invariants
from tests.st.support.metrics import compare_metrics, extract_metrics
from tests.st.support.runner import ScenarioRun, run_scenario
from tests.st.support.trajectory_extract import extract_trajectory
from tests.st.support.types import CheckIssue, format_issue

RunnerFunc = Callable[..., ScenarioRun]


def main(
    argv: list[str] | None = None,
    *,
    root: Path | None = None,
    runner_func: RunnerFunc | None = None,
    stdout: TextIO | None = None,
) -> int:
    """运行 ST 入口。注意：测试可注入 root、runner_func 和 stdout。"""

    out = stdout or sys.stdout
    # Windows 重定向管道默认走本地编码(GBK)，与 PowerShell 7 的 UTF-8 解码不一致会乱码，统一为 UTF-8。
    if out is sys.stdout and hasattr(out, "reconfigure"):
        out.reconfigure(encoding="utf-8")
    repo_root = root or ROOT
    args = _parse_args(argv)
    scenarios_dir = repo_root / "tests" / "st" / "scenarios"
    baselines_dir = repo_root / "tests" / "st" / "baselines"
    runner = runner_func or run_scenario
    scenarios = _select_scenarios(scenarios_dir, args.scenario)
    failures: list[str] = []
    hints: list[str] = []

    for name, config_path in scenarios:
        run = runner(config_path, scenario=name)
        metrics = extract_metrics(name, run.snapshots, run.config)
        trajectory = extract_trajectory(run.snapshots)
        baseline_path = baselines_dir / f"{name}.json"
        if args.update_baseline:
            _write_baseline(baseline_path, metrics, trajectory)
            continue
        failures.extend(format_issue(issue) for issue in run_invariants(run))
        baseline = _read_baseline(baseline_path)
        if baseline is None:
            failures.append(format_issue(CheckIssue(name, "UT-08", "缺少基线文件", field=str(baseline_path))))
            continue
        for item in compare_metrics(name, baseline.get("metrics", {}), metrics):
            if item.issue is not None:
                failures.append(format_issue(item.issue))
            if item.hint is not None:
                hints.append(item.hint)
        diffs = diff_trajectory(baseline.get("trajectory", {}), trajectory)
        if diffs:
            for diff in diffs:
                failures.append(f"[{name}][UT-10] 轨迹基线不一致 {format_diff(diff)}")
            failures.append(f"[{name}][UT-10] {UPDATE_BASELINE_HINT}")

    if args.update_baseline:
        print("BASELINE UPDATED", file=out)
        return 0
    for hint in hints:
        print(hint, file=out)
    if failures:
        for failure in failures:
            print(failure, file=out)
        print(f"ST FAILED: {len(failures)} 项", file=out)
        return 1
    print("ST OK", file=out)
    return 0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """解析命令行参数。注意：保持脚本契约精简。"""

    parser = argparse.ArgumentParser(description="运行自动化 ST 场景")
    parser.add_argument("--scenario", help="只运行指定场景名，例如 st_line")
    parser.add_argument("--update-baseline", action="store_true", help="重算并写入 T2/T3 基线")
    return parser.parse_args(argv)


def _select_scenarios(scenarios_dir: Path, scenario: str | None) -> list[tuple[str, Path]]:
    """选择待运行场景。注意：默认按文件名排序保证输出稳定。"""

    if scenario:
        path = scenarios_dir / f"{scenario}.json"
        if not path.exists():
            raise SystemExit(f"unknown scenario: {scenario}")
        return [(scenario, path)]
    return [(path.stem, path) for path in sorted(scenarios_dir.glob("st_*.json"))]


def _read_baseline(path: Path) -> dict[str, Any] | None:
    """读取基线文件。注意：缺失时返回 None 交给调用方报告。"""

    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else None


def _write_baseline(path: Path, metrics: dict[str, Any], trajectory: dict[str, Any]) -> None:
    """写入基线文件。注意：只包含 metrics 与 trajectory 两段。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"metrics": metrics, "trajectory": trajectory}
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
