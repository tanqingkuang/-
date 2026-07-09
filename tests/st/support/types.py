from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CheckIssue:
    """描述一条 ST 检查问题。注意：字段保持扁平，便于脚本输出一行定位。"""

    scenario: str
    ut: str
    message: str
    time_s: float | None = None
    node: str | None = None
    field: str | None = None
    actual: Any | None = None
    limit: Any | None = None


def format_issue(issue: CheckIssue) -> str:
    """把检查问题格式化为单行报告。注意：输出给 AI/人工一屏定位使用。"""

    parts = [f"[{issue.scenario}][{issue.ut}] {issue.message}"]
    if issue.time_s is not None:
        parts.append(f"@t={issue.time_s:.3f}s")
    if issue.node is not None:
        parts.append(f"node={issue.node}")
    if issue.field is not None:
        parts.append(f"field={issue.field}")
    if issue.actual is not None:
        parts.append(f"actual={issue.actual}")
    if issue.limit is not None:
        parts.append(f"limit={issue.limit}")
    return " ".join(parts)
