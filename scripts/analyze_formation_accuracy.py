"""根据用户选择的仿真快照生成编队控制精度报告。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from tkinter import Tk, filedialog


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.data.formation_accuracy_analysis import (  # noqa: E402
    analyze_formation_accuracy,
    write_accuracy_report,
)


def _choose_snapshot_file() -> Path | None:
    """弹出文件选择框并返回用户选择的仿真快照。"""

    root = Tk()
    root.withdraw()
    try:
        selected = filedialog.askopenfilename(
            title="选择要分析的仿真快照",
            initialdir=PROJECT_ROOT / "result" / "simulation_data" / "logs",
            filetypes=(("仿真快照", "snapshots.jsonl"), ("JSONL 文件", "*.jsonl")),
        )
    finally:
        root.destroy()
    return Path(selected) if selected else None


def _parse_args(argv: list[str]) -> argparse.Namespace:
    """解析快照文件和报告输出目录。"""

    parser = argparse.ArgumentParser(description="生成编队控制精度报告")
    parser.add_argument(
        "snapshot_file",
        type=Path,
        nargs="?",
        help="要分析的 snapshots.jsonl；自动读取同目录的 config.json",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "result" / "analysis",
        help="分析报告根目录",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """分析指定快照并输出报告路径。"""

    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    try:
        selected_file = args.snapshot_file or _choose_snapshot_file()
        if selected_file is None:
            print("已取消编队精度分析。")
            return 0
        snapshot_file = selected_file.resolve()
        if snapshot_file.name.lower() != "snapshots.jsonl" or not snapshot_file.is_file():
            raise ValueError(f"请选择有效的 snapshots.jsonl：{snapshot_file}")
        report = analyze_formation_accuracy(snapshot_file.parent)
        output_dir = write_accuracy_report(report, args.output_root.resolve())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"编队精度分析失败：{exc}")
        return 1
    print(f"编队精度分析完成：{output_dir}")
    print(f"分析状态：{report.status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
