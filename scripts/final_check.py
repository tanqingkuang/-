"""最终收口一键检查（自判断版）。

借鉴 FCC 仓库 ``run-xxx-if-needed`` 模式：把"这个门禁要不要跑"的判断从
agent 提示词下沉到脚本。脚本收集本次改动文件（工作区未提交改动 +
相对 ``origin/main`` 已提交改动），逐个门禁匹配触发模式：

- 命中 → 真实执行，失败时只回放输出尾部关键行；
- 未命中 → 标记 ``SKIP(no matching changes)``，等价于 FCC 的
  ``don't need xxx``，视为该门禁已完成。

用法（项目根目录）::

    python -X utf8 scripts/final_check.py            # 按改动自动裁剪
    python -X utf8 scripts/final_check.py --all      # 强制全量执行
    python -X utf8 scripts/final_check.py --base X   # 指定比对基准

输出契约：
- 每个门禁一行 ``<gate>=PASS|FAIL|SKIP(原因)``；
- 末行 ``final_check=PASS|FAIL``；全部非 FAIL 时退出码 0，否则 1。
"""

from __future__ import annotations

import argparse
import fnmatch
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# 项目根目录：所有 git / 门禁命令都以它为工作目录，保证任意 cwd 可执行。
ROOT = Path(__file__).resolve().parents[1]

# 失败时最多回放的输出行数，避免把全量日志灌进对话。
FAIL_TAIL_LINES = 30


@dataclass
class Gate:
    """一个收口门禁：名字、触发模式与待执行命令序列。"""

    name: str
    # 触发改动模式（fnmatch 语法）；None 表示无条件执行。
    patterns: list[str] | None
    # 排除模式：命中 patterns 但同时命中 excludes 的文件不算触发。
    excludes: list[str]
    # 依次执行的命令，任一非零即判 FAIL。
    commands: list[list[str]]


# 门禁清单与 .agents/test.md 一一对应；触发模式即"改了什么才需要跑"。
GATES: list[Gate] = [
    Gate(
        name="compile",
        patterns=["src/*.py"],
        excludes=[],
        commands=[[sys.executable, "-m", "compileall", "-q", "src"]],
    ),
    Gate(
        name="lint",
        patterns=["src/*.py", "tests/*.py", "scripts/*.py", "pyproject.toml"],
        excludes=[],
        commands=[
            [sys.executable, "-m", "ruff", "check", "src", "tests", "scripts"],
            [sys.executable, "-m", "mypy"],
        ],
    ),
    Gate(
        name="llt",
        patterns=["src/*.py", "tests/llt/*", "pyproject.toml"],
        excludes=[],
        commands=[[sys.executable, "-m", "pytest", "tests/llt", "-q"]],
    ),
    Gate(
        name="comment_coverage",
        patterns=["src/*.py"],
        excludes=[],
        commands=[
            [
                sys.executable,
                "-X",
                "utf8",
                "scripts/comment_coverage.py",
                "--fail-under-module", "100",
                "--fail-under-class", "100",
                "--fail-under-func", "100",
                "--fail-under-inline", "15",
                "--worst", "12",
            ]
        ],
    ),
    Gate(
        # ST 只关心仿真结果：GUI 层改动不会影响黑盒轨迹，予以排除。
        name="st",
        patterns=["src/*.py", "tests/st/*", "scripts/run_st.py"],
        excludes=["src/ui/*"],
        commands=[[sys.executable, "scripts/run_st.py"]],
    ),
    Gate(
        name="gui_offscreen",
        patterns=["src/ui/*"],
        excludes=[],
        commands=[[sys.executable, "-X", "utf8", "scripts/check_gui_offscreen.py"]],
    ),
    Gate(
        name="demo_html",
        patterns=["docs/demo.html", "scripts/check_demo_html.js"],
        excludes=[],
        commands=[["node", "scripts/check_demo_html.js"]],
    ),
    Gate(
        name="git_diff_check",
        patterns=None,
        excludes=[],
        commands=[["git", "diff", "--check"]],
    ),
]


def _git_lines(args: list[str]) -> list[str]:
    """执行 git 命令并返回非空输出行；失败时返回空列表而不是抛错。"""
    try:
        out = subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout
    except (subprocess.CalledProcessError, OSError):
        return []
    return [line for line in out.splitlines() if line.strip()]


def collect_changed_files(base: str) -> list[str]:
    """收集改动文件：工作区/暂存区未提交改动 ∪ 相对基准分支的已提交改动。"""
    files: set[str] = set()

    # 工作区与暂存区（含未跟踪文件）；重命名行形如 "R  old -> new"。
    for line in _git_lines(["status", "--porcelain"]):
        path = line[3:]
        if " -> " in path:
            path = path.split(" -> ", 1)[1]
        files.add(path.strip().strip('"'))

    # 相对基准分支已提交但未合入的改动；基准不存在时静默跳过。
    merge_base = _git_lines(["merge-base", base, "HEAD"])
    if merge_base:
        files.update(_git_lines(["diff", "--name-only", merge_base[0], "HEAD"]))

    return sorted(f.replace("\\", "/") for f in files if f)


def gate_needed(gate: Gate, changed: list[str]) -> bool:
    """按触发/排除模式判断门禁是否需要真实执行。"""
    if gate.patterns is None:
        return True
    for path in changed:
        if any(fnmatch.fnmatch(path, pat) for pat in gate.excludes):
            continue
        if any(fnmatch.fnmatch(path, pat) for pat in gate.patterns):
            return True
    return False


def run_gate(gate: Gate) -> tuple[bool, str]:
    """依次执行门禁命令，返回 (是否通过, 失败时的尾部输出)。"""
    for cmd in gate.commands:
        proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        if proc.returncode != 0:
            merged = (proc.stdout + "\n" + proc.stderr).strip()
            tail = "\n".join(merged.splitlines()[-FAIL_TAIL_LINES:])
            return False, f"$ {' '.join(cmd)}\n{tail}"
    return True, ""


def main() -> int:
    """解析参数、裁剪并执行各门禁，输出结构化摘要。"""
    parser = argparse.ArgumentParser(description="最终收口一键检查")
    parser.add_argument("--all", action="store_true", help="忽略改动裁剪，强制执行全部门禁")
    parser.add_argument("--base", default="origin/main", help="已提交改动的比对基准（默认 origin/main）")
    args = parser.parse_args()

    changed = collect_changed_files(args.base)
    print(f"changed_files={len(changed)} base={args.base}")

    failed = False
    for gate in GATES:
        if not args.all and not gate_needed(gate, changed):
            print(f"{gate.name}=SKIP(no matching changes)")
            continue
        start = time.monotonic()
        ok, detail = run_gate(gate)
        elapsed = time.monotonic() - start
        if ok:
            print(f"{gate.name}=PASS ({elapsed:.1f}s)")
        else:
            failed = True
            print(f"{gate.name}=FAIL ({elapsed:.1f}s)")
            print(detail, file=sys.stderr)

    print(f"final_check={'FAIL' if failed else 'PASS'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
