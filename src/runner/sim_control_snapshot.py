"""SimulationController 快照生成辅助。注意：作为 mixin 降低控制器主体长度。"""

from __future__ import annotations

import math
import time

from src.algorithm.context.leaf_types import FormStageE, PosTrackDiagS
from src.environment.model import AircraftState
from src.runner.sim_control_routes import _route_state_from_wayline
from src.runner.sim_control_types import (
    ControlReport,
    LinkState,
    NodeState,
    RouteState,
    RunState,
    SimulationSnapshot,
    _ConfiguredLink,
)


class SimulationControllerSnapshotMixin:
    """拆分快照构造与航线几何计算。注意：依赖主控制器实例状态。"""

    def _make_snapshot_unlocked(self) -> SimulationSnapshot:
        """在已持锁状态下生成完整快照。注意：不得把内部可变对象直接暴露出去。"""
        # 汇总各子系统当前态：健康表、当前航段、全部航段，再逐节点组装显示状态。
        health_map = self._disturbance.read_health()
        route = self._make_route_snapshot()
        route_segments = self._make_route_segment_snapshots()
        blocked_route_segments = self._make_blocked_route_segment_snapshots()
        nodes: list[NodeState] = []
        rally_phases = (
            {}
            if self._run_state == RunState.READY
            else {nid: alg.current_rally_phase_str() for nid, alg in self._node_algorithms.items()}
        )
        # 评测补充字段的公共上下文：原始指令饱和阈值与当前队形槽位表。
        acc_limit = self._model.acceleration_command_limit_mps2
        slot_rows = self._formation_slots
        slot_row = (
            slot_rows[self._formation_index]
            if 0 <= self._formation_index < len(slot_rows)
            else []
        )
        slot_by_id = {slot.id: slot for slot in slot_row}
        for state in self._model.read_states().values():
            algorithm = self._node_algorithms.get(state.node_id)
            diag = self._control_diagnostics.get(state.node_id, PosTrackDiagS())
            control = self._current_controls.get(state.node_id)
            # 原始控制指令取限幅前缓存值；任一 ENU 轴触达模型幅值上限即记饱和。
            cmd_acc = control.as_vector() if control is not None else (0.0, 0.0, 0.0)
            acc_saturated = any(abs(a) >= acc_limit - 1e-9 for a in cmd_acc)
            slot = slot_by_id.get(state.node_id)
            cmd_pos_e = diag.cmd_pos_east_m
            cmd_pos_n = diag.cmd_pos_north_m
            # 快照是环境模型到 UI/日志的裁决边界，字段参考系在此一次性固定。
            # 位置、速度分量与控制诊断统一使用东北天 ENU，便于直接叠加与作图。
            # psi_v_deg/theta_deg 反映地面轨迹，横风下应与 air_* 角度明确分离。
            # speed_mps 保留为空速兼容字段，ground_speed_mps 才描述三维地速大小。
            # nx/ny/nz 按前上右 FUR 原样输出，n_normal 仅作为法向合量的派生指标。
            # psi_dot_deg_s 对外表示地面转率，air_psi_dot_deg_s 保留动力学转率证据。
            nodes.append(
                NodeState(
                    node_id=state.node_id,
                    role=self._node_roles.get(state.node_id, "unknown"),
                    health=health_map.get(state.node_id, "normal"),
                    x_m=state.x_m,
                    y_m=state.y_m,
                    altitude_m=state.altitude_m,
                    psi_v_deg=math.degrees(state.ground_psi_rad),
                    theta_deg=math.degrees(state.ground_theta_rad),
                    speed_mps=state.speed_mps,
                    airspeed_mps=state.speed_mps,
                    air_psi_v_deg=state.psi_v_deg,
                    air_theta_deg=state.theta_deg,
                    air_psi_dot_deg_s=state.psi_dot_deg_s,
                    wind_east_mps=state.wind_east_mps,
                    wind_north_mps=state.wind_north_mps,
                    wind_up_mps=state.wind_up_mps,
                    ground_speed_mps=state.ground_speed_mps,
                    vx_mps=state.ground_vx_mps,
                    vy_mps=state.ground_vy_mps,
                    vz_mps=state.ground_vz_mps,
                    nx=state.nx,
                    ny=state.ny,
                    nz=state.nz,
                    n_normal=state.n_normal,
                    phi_deg=state.phi_deg,
                    psi_dot_deg_s=state.ground_psi_dot_deg_s,
                    # 控制指令是 ENU 绝对量，直接保留其东、北、天三个物理分量。
                    cmd_pos_east_m=cmd_pos_e,
                    cmd_pos_north_m=cmd_pos_n,
                    cmd_pos_h_m=diag.cmd_pos_h_m,
                    # 速度指令同样保持 ENU，不能套用航迹系的前上右轴序。
                    cmd_vel_east_mps=diag.cmd_vel_east_mps,
                    cmd_vel_north_mps=diag.cmd_vel_north_mps,
                    cmd_vel_up_mps=diag.cmd_vel_up_mps,
                    # 世界系位置误差用于复现导航解算，正负号遵循 ENU 各轴方向。
                    pos_err_east_m=diag.pos_err_east_m,
                    pos_err_north_m=diag.pos_err_north_m,
                    pos_err_h_m=diag.pos_err_h_m,
                    # 世界系速度误差与上面的地速分量同源，可直接做逐轴闭环分析。
                    vel_err_east_mps=diag.vel_err_east_mps,
                    vel_err_north_mps=diag.vel_err_north_mps,
                    vel_err_up_mps=diag.vel_err_up_mps,
                    # 航迹系位置误差按前上右输出，z 正值始终代表右侧误差。
                    track_pos_err_x_m=diag.track_pos_err_x_m,
                    track_pos_err_y_m=diag.track_pos_err_y_m,
                    track_pos_err_z_m=diag.track_pos_err_z_m,
                    # 航迹系速度误差与槽位坐标共用 FUR，便于验证控制器轴向响应。
                    track_vel_err_x_mps=diag.track_vel_err_x_mps,
                    track_vel_err_y_mps=diag.track_vel_err_y_mps,
                    track_vel_err_z_mps=diag.track_vel_err_z_mps,
                    # 侧偏与待飞距相对"当前航段"计算，供 UI 显示跟踪误差。
                    # 侧偏取右侧为正，与 FUR z 轴一致；待飞距则沿航段前向投影。
                    cross_track_error_m=self._cross_track_error(state, route),
                    distance_to_go_m=self._distance_to_go(state, route),
                    rally_phase=rally_phases.get(state.node_id, ""),
                    # 直接记录算法任务阶段，离线分析据此严格识别全队 HOLD。
                    task_stage=algorithm.current_stage().name if algorithm is not None else "",
                    # 评测补充：原始 ENU 加速度指令与饱和证据、算法耗时、标称槽位。
                    cmd_acc_east_mps2=cmd_acc[0],
                    cmd_acc_north_mps2=cmd_acc[1],
                    cmd_acc_up_mps2=cmd_acc[2],
                    acc_saturated=acc_saturated,
                    lateral_saturated=diag.lateral_saturated,
                    algo_step_ms=self._algo_step_ms.get(state.node_id, 0.0),
                    slot_x_m=slot.x if slot is not None else None,
                    slot_y_m=slot.y if slot is not None else None,
                    slot_z_m=slot.z if slot is not None else None,
                )
            )
        # 链路快照已折叠双向状态。
        links = self._make_configured_link_snapshots()
        return SimulationSnapshot(
            time_s=self._time_s,
            duration_s=self._duration_s,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=nodes,
            links=links,
            active_disturbances=self._disturbance.active_types(),
            route=route,
            route_segments=route_segments,
            blocked_route_segments=blocked_route_segments,
            cpu_utilization=self._cpu_utilization,
            rally_geometry=self._rally_geometry,
        )

    def _parse_configured_links(self, raw_links: list[object]) -> list[_ConfiguredLink]:
        """解析配置中的通信链路。注意：链路 ID 需能反向映射双向状态。"""
        configured: list[_ConfiguredLink] = []
        for link in raw_links:
            # 跳过缺 link_id 的非法条目。
            if not isinstance(link, dict) or not link.get("link_id"):
                continue
            configured.append(
                _ConfiguredLink(
                    link_id=str(link["link_id"]),
                    direction=str(link.get("direction") or "duplex"),
                )
            )
        return configured

    def _make_configured_link_snapshots(self) -> list[LinkState]:
        """生成配置链路快照。注意：需要合并正反向通信状态。"""
        # 把通信模块返回的有向状态建索引，便于按 link_id 查找。
        states = {state.link_id: state for state in self._comm.read_link_states()}
        links: list[LinkState] = []
        for configured in self._configured_links:
            # 双工链路需同时取正反两个方向，合并为面向 UI 的一条。
            ids = [configured.link_id]
            if configured.direction == "duplex":
                ids.append(self._reverse_link_id(configured.link_id))
            directional_states = [states[link_id] for link_id in ids if link_id in states]
            if not directional_states:
                continue
            # 折叠取最坏值：任一方向中断即显示 lost，时延/丢包取两向最大（保守显示）。
            status = "lost" if any(state.status == "lost" for state in directional_states) else directional_states[0].status
            links.append(
                LinkState(
                    link_id=configured.link_id,
                    direction=configured.direction,
                    latency_ms=max(state.latency_ms for state in directional_states),
                    loss_rate=max(state.loss_rate for state in directional_states),
                    status=status,
                    frame_rate_hz=directional_states[0].frame_rate_hz,
                )
            )
        return links

    def _make_route_snapshot(self) -> RouteState | None:
        """生成当前航线快照。注意：无航线时返回空状态。"""
        # 取第一个能给出当前航段的算法（通常是长机）作为显示航线。
        for algorithm in self._node_algorithms.values():
            route = algorithm.current_route()
            if route is None:
                continue
            return _route_state_from_wayline(route)
        return None

    def _make_route_segment_snapshots(self) -> list[RouteState]:
        """生成全部航段快照。注意：用于 GUI 绘制多航段轨迹。"""
        if not self._node_algorithms or self._leader_route is None:
            return []
        display_route = self._display_route
        if not display_route:
            return []
        return [_route_state_from_wayline(line) for line in display_route]

    def _make_blocked_route_segment_snapshots(self) -> list[RouteState]:
        """生成被封锁的原始航线快照。注意：仅在避障覆盖航线生效时非空。"""

        # getattr 兜底只是双重保险：__init__/_init_modules_unlocked 已保证该属性总被赋值，
        # 这里不应该真的触发默认值分支，出现了反而说明初始化顺序被破坏。
        blocked_route = getattr(self, "_blocked_display_route", None)
        if not blocked_route:
            return []
        return [_route_state_from_wayline(line) for line in blocked_route]

    @staticmethod
    def _cross_track_error(state: AircraftState, route: RouteState | None) -> float | None:
        """计算节点相对当前航段的侧偏。注意：退化航段返回零偏差。"""
        if route is None:
            return None
        if route.radius_m > 0.0:
            # 圆弧段侧偏应取径向误差；按转向符号定号，保持左右偏差语义稳定。
            radial_distance = math.hypot(state.x_m - route.center_x_m, state.y_m - route.center_y_m)
            turn_sign = 1.0 if route.turn_sign >= 0.0 else -1.0
            return (radial_distance - route.radius_m) * turn_sign
        # 航段方向向量（ENU 平面，x 东 y 北）。
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        # 退化航段（首尾重合）无法定义法向，返回 None。
        if length <= 1e-9:
            return None
        # 单位右法向量（航段方向顺时针旋转 90°），与航迹系 z 右侧向为正保持一致。
        normal_x = dy / length
        normal_y = -dx / length
        # 侧偏 = 起点->节点向量在右法向上的投影；正值表示位于航迹右侧。
        return (state.x_m - route.start_x_m) * normal_x + (state.y_m - route.start_y_m) * normal_y

    @staticmethod
    def _distance_to_go(state: AircraftState, route: RouteState | None) -> float | None:
        """计算节点到当前航段终点的待飞距。注意：结果不包含后续航段距离。"""
        if route is None:
            return None
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        # 沿航段方向的单位向量。
        track_x = dx / length
        track_y = dy / length
        # 待飞距 = 节点->终点向量在航段方向上的投影；越过终点时夹到 0。
        return max(0.0, (route.end_x_m - state.x_m) * track_x + (route.end_y_m - state.y_m) * track_y)

    @staticmethod
    def _reverse_link_id(link_id: str) -> str:
        """生成通信链路反向 ID。注意：仅处理约定格式的双机链路。"""
        # 交换 "src-dst" 两端得到反向 ID；无分隔符则原样返回。
        src, sep, dst = link_id.partition("-")
        if not sep:
            return link_id
        return f"{dst}-{src}"

    def _make_snapshot_for_empty_controller(self) -> SimulationSnapshot:
        """生成空控制器快照。注意：用于未加载配置时的 GUI 初始显示。"""
        return SimulationSnapshot(
            time_s=0.0,
            duration_s=0.0,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=[],
            links=[],
            cpu_utilization=0.0,
        )

    def _derive_control_report_unlocked(self) -> ControlReport:
        """根据当前状态推导控制回报文本。注意：调用方需持锁。"""
        # 任一节点非健康即优先判为"重构"——故障会触发队形重构。
        if any(h != "normal" for h in self._disturbance.read_health().values()):
            return "重构"
        algorithms = list(self._node_algorithms.values())
        stages = [algorithm.current_stage() for algorithm in algorithms]
        rally_phases = [algorithm.current_rally_phase_str() for algorithm in algorithms]
        active_rally_phases = {"RALLY_TRANSIT", "RALLY_LOITER", "RALLY_EXITED", "CATCHUP", "LOOSE"}
        # 按优先级聚合各节点编队阶段：集结 > 保持 > 待命；故障重构已在上方判断。
        if any(phase in active_rally_phases for phase in rally_phases) or any(stage == FormStageE.RALLY for stage in stages):
            return "集结"
        if any(phase == "HOLD" for phase in rally_phases) or any(stage == FormStageE.HOLD for stage in stages):
            return "保持"
        if any(phase == "LOCAL_LOITER" for phase in rally_phases):
            return "待命"
        return "待命" if self._node_algorithms else "待命"

    def _should_refresh_display_unlocked(self) -> bool:
        """判断本 tick 是否需要刷新显示。注意：用于降低 GUI 刷新频率。"""
        # 按墙钟节流：距上次刷新满 _DISPLAY_REFRESH_S 才允许，避免高频 tick 压垮 UI。
        now_s = time.monotonic()
        if self._last_display_wall_s == 0.0 or now_s - self._last_display_wall_s >= self._DISPLAY_REFRESH_S:
            self._last_display_wall_s = now_s
            return True
        return False
