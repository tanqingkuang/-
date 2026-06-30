"""SimulationController 后台调度循环。注意：作为 mixin 降低控制器主体长度。"""

from __future__ import annotations

import threading
import time

from src.runner.sim_control_constants import (
    _CPU_UTILIZATION_SAMPLE_PERIOD_S,
    _MAX_RUN_LOOP_BATCH_TICKS,
    _RUN_LOOP_SLEEP_SLICE_S,
)
from src.runner.sim_control_types import SimulationSnapshot


def _cpu_utilization_sample_period_s() -> float:
    """读取 CPU 统计周期。注意：兼容旧入口 sim_control 上的测试 patch。"""
    from src.runner import sim_control

    return float(getattr(sim_control, "_CPU_UTILIZATION_SAMPLE_PERIOD_S", _CPU_UTILIZATION_SAMPLE_PERIOD_S))


class SimulationControllerLoopMixin:
    """拆分后台线程调度逻辑。注意：依赖主控制器实例状态。"""

    def _run_loop(self) -> None:
        """后台线程主循环。注意：所有共享状态访问必须受锁保护。"""
        current = threading.current_thread()
        last_wall_s = time.perf_counter()
        stats_start_wall_s = last_wall_s
        stats_busy_s = 0.0
        sim_budget_s = 0.0
        force_first_tick = True
        try:
            # 直到收到停止请求；按累计墙钟时间批量补拍，避免高倍率下依赖亚毫秒 sleep。
            while not self._stop_requested.is_set():
                now_wall_s = time.perf_counter()
                wall_delta_s = max(0.0, now_wall_s - last_wall_s)
                last_wall_s = now_wall_s
                snapshots_to_notify: list[SimulationSnapshot] = []
                cpu_snapshot: SimulationSnapshot | None = None
                should_sleep = False
                with self._lock:
                    # 运行态被外部改为非 RUNNING（暂停/结束）时退出循环。
                    if self._run_state != "RUNNING":
                        break
                    step_s = self._step_s
                    playback_rate = self._playback_rate
                    sim_budget_s += wall_delta_s * playback_rate
                    if force_first_tick and sim_budget_s < step_s:
                        sim_budget_s = step_s
                    force_first_tick = False

                    ticks_due = min(int(sim_budget_s / step_s), _MAX_RUN_LOOP_BATCH_TICKS)
                    if ticks_due <= 0:
                        should_sleep = True
                    for _ in range(ticks_due):
                        try:
                            snapshot = self._tick_unlocked()
                        except Exception as exc:  # noqa: BLE001
                            # tick 出错不崩线程：记录错误、转入暂停并产出一帧快照便于排查。
                            self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                            self._run_state = "PAUSED"
                            snapshot = self._make_snapshot_unlocked()
                        sim_budget_s = max(0.0, sim_budget_s - step_s)
                        if snapshot is not None:
                            snapshots_to_notify.append(snapshot)
                        if self._run_state != "RUNNING":
                            break
                    if self._run_state == "RUNNING" and sim_budget_s < step_s:
                        should_sleep = True
                # 在锁外通知订阅者，避免回调阻塞持锁路径。
                for snapshot in snapshots_to_notify:
                    self._notify_subscribers(snapshot)
                busy_end_s = time.perf_counter()
                stats_busy_s += max(0.0, busy_end_s - now_wall_s)
                stats_wall_s = max(0.0, busy_end_s - stats_start_wall_s)
                if stats_wall_s >= _cpu_utilization_sample_period_s():
                    with self._lock:
                        self._cpu_utilization = min(1.0, max(0.0, stats_busy_s / stats_wall_s))
                        self._latest_snapshot = self._make_snapshot_unlocked()
                        cpu_snapshot = self._latest_snapshot
                    stats_start_wall_s = busy_end_s
                    stats_busy_s = 0.0
                if cpu_snapshot is not None:
                    self._notify_subscribers(cpu_snapshot)
                if should_sleep:
                    time.sleep(_RUN_LOOP_SLEEP_SLICE_S)
        finally:
            with self._lock:
                # 仅当自己仍是登记的工作线程时才清空引用，避免误清新线程。
                if self._worker is current:
                    self._worker = None

    def _stop_worker(self) -> None:
        """停止后台工作线程。注意：调用后需要等待线程退出。"""
        # 置停止标志，循环下一圈即退出。
        self._stop_requested.set()
        worker = self._worker
        # 等待线程真正退出（最多 2s）；但不能 join 自身，否则死锁。
        if worker is not None and worker.is_alive() and worker is not threading.current_thread():
            worker.join(timeout=2.0)
        self._worker = None
        # 清标志，为下次启动复位。
        self._stop_requested.clear()

    def _start_worker_unlocked(self) -> None:
        """在已持锁状态下启动工作线程。注意：调用方必须先持有控制器锁。"""
        # 已有存活线程则不重复创建。
        if self._worker is not None and self._worker.is_alive():
            return
        # 守护线程：主程序退出时不被其阻塞。
        self._worker = threading.Thread(target=self._run_loop, name="SimulationController", daemon=True)
        self._worker.start()
