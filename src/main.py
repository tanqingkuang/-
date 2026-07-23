"""一键仿真无界面入口。注意：GUI 入口仍由 ``main_window.py`` 独立负责。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def _ensure_project_root_on_path() -> None:
    """确保直接执行本文件时能够导入项目包。注意：包导入场景不修改路径。"""

    if __package__:
        return
    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))


_ensure_project_root_on_path()

from src.runner.sim_control import RunState  # noqa: E402
from src.runner.sim_controller import SimulationController  # noqa: E402


def _parse_args(argv: list[str]) -> argparse.Namespace:
    """解析一键仿真的配置路径和墙钟倍率。注意：默认倍率为 10x。"""

    parser = argparse.ArgumentParser(description="无界面运行一份编队仿真配置")
    parser.add_argument("--config", required=True, type=Path, help="仿真 JSON 配置文件路径")
    parser.add_argument("--rate", default=10.0, type=float, help="无界面运行倍率，默认 10 倍速")
    # seed 是本次进程的运行参数，不从场景 JSON 推导，便于 BAT 批量替换。
    parser.add_argument("--seed", default=0, type=int, help="不确定性算例及随机过程种子，默认 0")
    return parser.parse_args(argv)


def _print_status(message: str) -> None:
    """输出一键仿真状态。注意：调用方依赖控制台判断当前进度。"""

    if sys.stdout is not None:
        print(message)


def run_simulation(
    config_path: Path,
    *,
    playback_rate: float = 10.0,
    seed: int = 0,
) -> int:
    """按指定墙钟倍率无界面运行一份仿真，成功返回 0，失败返回 1。"""

    controller = SimulationController()
    try:
        # 无界面运行没有 GUI 内存回放入口，必须强制落盘供后续分析使用。
        controller.set_file_log_enabled(True)
        # 显式传入控制器，确保配置文件中的历史 seed 字段不会覆盖 BAT 选择。
        result = controller.load_config(str(config_path), seed=seed)
        if result.code != "OK":
            _print_status(f"仿真失败 [{result.code}]: {result.message}")
            return 1
        result = controller.set_playback_rate(playback_rate)
        if result.code != "OK":
            _print_status(f"仿真失败 [{result.code}]: {result.message}")
            return 1
        has_rally_nodes = any(
            node.role in {"rally_leader", "rally_follower"}
            for node in controller.get_snapshot().nodes
        )
        result = controller.start(auto_rally=has_rally_nodes)
        if result.code != "OK":
            _print_status(f"仿真失败 [{result.code}]: {result.message}")
            return 1
        _print_status(f"仿真开始: {config_path}，倍率 {playback_rate:g}x，seed={seed}")
        while True:
            snapshot = controller.get_snapshot()
            if snapshot.run_state == RunState.FINISHED:
                log_result = controller.validate_file_log()
                if log_result.code != "OK":
                    _print_status(f"仿真失败 [{log_result.code}]: {log_result.message}")
                    return 1
                _print_status(f"仿真完成: {config_path}")
                return 0
            if snapshot.run_state != RunState.RUNNING:
                _print_status(f"仿真异常停止: state={snapshot.run_state}")
                return 1
            # 后台控制器负责按墙钟倍率推进；入口仅低频轮询终态，避免无意义忙等。
            time.sleep(0.05)
    except Exception as exc:  # noqa: BLE001
        # 进程边界必须把未预期异常转换成非零退出码，供 BAT 判断失败。
        _print_status(f"仿真异常: {exc}")
        return 1
    finally:
        controller.close()


def main(argv: list[str] | None = None) -> int:
    """执行一键无界面仿真入口。"""

    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    return run_simulation(args.config, playback_rate=args.rate, seed=args.seed)


if __name__ == "__main__":
    raise SystemExit(main())
