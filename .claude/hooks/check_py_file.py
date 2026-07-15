"""PostToolUse hook：Edit/Write 落盘 .py 文件后即时做语法 + ruff 单文件检查。

借鉴 FCC 仓库 ensure-bom hook 的思路：把"改完必须自检"从提示词下沉为机制。
从 stdin 读取 Claude Code 的 hook JSON，提取被写入的文件路径：

- 非 .py 文件、项目外文件、已被删除的文件 → 静默退出 0；
- ``py_compile`` 语法检查 + ``ruff check`` 单文件检查（毫秒级）；
- 任一失败 → 关键错误行写 stderr 并退出 2，Claude 会立即收到反馈修复。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

# 项目根目录 = 本文件上溯两级（.claude/hooks/ → 仓库根）。
ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    """解析 hook 输入并对目标 .py 文件做快速检查，返回退出码。"""
    try:
        payload = json.load(sys.stdin)
    except json.JSONDecodeError:
        return 0
    file_path = payload.get("tool_input", {}).get("file_path", "")
    if not file_path.endswith(".py"):
        return 0

    target = Path(file_path)
    if not target.exists():
        return 0
    try:
        # 项目外的 .py（如用户主目录下的脚本）不做检查，避免误用本项目 ruff 配置。
        target.resolve().relative_to(ROOT)
    except ValueError:
        return 0

    errors: list[str] = []

    # 语法检查：比 compileall 全量扫描快，且只针对本次改动文件。
    compile_proc = subprocess.run(
        [sys.executable, "-X", "utf8", "-m", "py_compile", str(target)],
        capture_output=True, text=True, cwd=ROOT,
    )
    if compile_proc.returncode != 0:
        errors.append(compile_proc.stderr.strip())

    # ruff 单文件检查：与 CI 门禁同一套 pyproject 配置。
    ruff_proc = subprocess.run(
        [sys.executable, "-m", "ruff", "check", str(target)],
        capture_output=True, text=True, cwd=ROOT,
    )
    if ruff_proc.returncode != 0:
        errors.append(ruff_proc.stdout.strip())

    if errors:
        print("\n".join(err for err in errors if err), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
