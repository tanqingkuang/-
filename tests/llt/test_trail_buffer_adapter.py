"""控制器 GUI 适配层共享尾迹队列测试。注意：不构造 Qt 窗口。"""

from __future__ import annotations

import unittest
from dataclasses import replace

from src.runner.sim_control import NodeState as ControllerNodeState
from src.runner.sim_control import SimulationSnapshot as ControllerSnapshot
from src.ui.gui.simulation_adapter import ControllerSimulationAdapter
from src.ui.gui.trail_path_cache import TrailPathCache
from src.ui.gui.trail_view_model import TrailBuffer, TrailSnapshot


def _controller_node(x_m: float) -> ControllerNodeState:
    """构造适配器测试节点。注意：只让东向位置随帧变化。"""

    return ControllerNodeState(
        node_id="A01",
        role="leader",
        health="normal",
        x_m=x_m,
        y_m=20.0,
        altitude_m=1200.0,
        psi_v_deg=0.0,
        theta_deg=0.0,
        speed_mps=10.0,
        vx_mps=10.0,
        vy_mps=0.0,
        vz_mps=0.0,
        nx=0.0,
        nz=1.0,
        phi_deg=0.0,
        psi_dot_deg_s=0.0,
    )


def _controller_snapshot(time_s: float, x_m: float) -> ControllerSnapshot:
    """构造单机控制器快照。注意：测试只关心时间与 ENU 位置。"""

    return ControllerSnapshot(
        time_s=time_s,
        duration_s=100.0,
        step_s=0.1,
        run_state="RUNNING",
        control_report="保持",
        nodes=[_controller_node(x_m)],
        links=[],
    )


class TrailBufferAdapterTests(unittest.TestCase):
    """验证适配器只增量维护队列，并向视图暴露稳定快照。"""

    def test_adapter_reuses_buffer_and_old_snapshot_stays_unchanged(self) -> None:
        """连续帧复用同一队列，旧快照不变且新快照复用历史点对象。"""

        adapter = ControllerSimulationAdapter()
        self.addCleanup(adapter.close)
        adapter.set_trail_seconds(10.0)

        first_snapshot = adapter._convert_snapshot(_controller_snapshot(1.0, 10.0))
        first_trail = first_snapshot.nodes[0].trail
        second_snapshot = adapter._convert_snapshot(_controller_snapshot(2.0, 20.0))
        second_trail = second_snapshot.nodes[0].trail

        self.assertIsInstance(adapter._trail_by_node["A01"], TrailBuffer)
        self.assertIsInstance(first_trail, TrailSnapshot)
        self.assertIsInstance(second_trail, TrailSnapshot)
        self.assertEqual(len(first_trail), 1)
        self.assertEqual(len(second_trail), 2)
        self.assertIs(first_trail[0], second_trail[0])
        self.assertEqual(first_trail.generation, second_trail.generation)
        self.assertEqual((second_trail.first_sequence, second_trail.end_sequence), (0, 2))

    def test_adapter_time_window_pops_head_without_renumbering_history(self) -> None:
        """适配器缩短时间窗后只弹头，保留点逻辑序号和累计路程不重算。"""

        adapter = ControllerSimulationAdapter()
        self.addCleanup(adapter.close)
        adapter.set_trail_seconds(10.0)
        first = _controller_snapshot(1.0, 10.0)
        adapter._convert_snapshot(first)
        adapter._convert_snapshot(replace(first, time_s=2.0, nodes=[_controller_node(22.0)]))
        adapter._convert_snapshot(replace(first, time_s=3.0, nodes=[_controller_node(34.0)]))

        adapter.set_trail_seconds(1.5)
        converted = adapter._convert_snapshot(replace(first, time_s=3.0, nodes=[_controller_node(34.0)]))

        trail = converted.nodes[0].trail
        self.assertEqual([point.x for point in trail], [22.0, 34.0])
        self.assertEqual([point.point_id for point in trail], [1, 2])
        self.assertEqual([point.path_distance for point in trail], [12.0, 24.0])

    def test_adapter_close_clears_trail_buffers(self) -> None:
        """关闭适配器时清空全部飞机尾迹队列。"""

        adapter = ControllerSimulationAdapter()
        adapter.set_trail_seconds(10.0)
        adapter._convert_snapshot(_controller_snapshot(1.0, 10.0))

        adapter.close()

        self.assertEqual(adapter._trail_by_node, {})

    def test_numbered_snapshot_drives_2d_cache_by_head_remove_and_tail_append(self) -> None:
        """共享快照游标使二维缓存只处理本帧弹头与新增点。"""

        trail = TrailBuffer(capacity=4)
        for index in range(4):
            trail.append_position(float(index), 0.0, 100.0, float(index))
        cache = TrailPathCache(chunk_size=2)
        cache.synchronize(
            trail.snapshot(),
            projector=lambda point: (point.x, point.y),
            semantic_key="俯视",
        )

        trail.append_position(4.0, 0.0, 100.0, 4.0)
        cache.synchronize(
            trail.snapshot(),
            projector=lambda point: (point.x, point.y),
            semantic_key="俯视",
        )

        self.assertEqual(cache.stats.point_count, 4)
        self.assertEqual(cache.stats.last_removed_points, 1)
        self.assertEqual(cache.stats.last_appended_points, 1)


if __name__ == "__main__":
    unittest.main()
