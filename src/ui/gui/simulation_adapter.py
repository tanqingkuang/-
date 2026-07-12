"""GUI 数据源适配层。注意：把真实控制器或演示数据统一成 Snapshot。"""

from __future__ import annotations

import json
import math
from pathlib import Path

from src.algorithm.context.leaf_types import WayPointInputS
from src.runner.sim_control import SimulationController
from src.runner.sim_control import SimulationSnapshot as ControllerSnapshot
from src.ui.gui.playback_view_model import PlaybackViewModel
from src.ui.gui.trail_view_model import prune_trail
from src.ui.gui.view_models import (
    WORLD_HEIGHT,
    WORLD_WIDTH,
    LinkState,
    NodeState,
    RallyGeometryView,
    ReferenceRoute,
    Snapshot,
    TrailPoint,
    link_direction_label,
    trail_seconds_for_duration,
)


def _append_trail_point(
    trail: list[TrailPoint],
    x: float,
    y: float,
    altitude: float,
    time: float,
) -> None:
    """追加带单调累计路程的尾迹点。注意：裁剪首部点不会重置已有路程基准。"""

    path_distance = 0.0
    if trail:
        previous = trail[-1]
        path_distance = previous.path_distance + math.hypot(x - previous.x, y - previous.y)
    trail.append(TrailPoint(x, y, altitude, time, path_distance))


class MockSimulation:
    """真实控制器接入前使用的小型 UI 演示数据源。注意：仅作为界面兜底。"""

    def __init__(self) -> None:
        """初始化 MockSimulation 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.duration = 120.0
        self.step = 0.1
        self.playback_vm = PlaybackViewModel()
        self.speed = 1.0
        self.time = 0.0
        self.running = False
        self.paused = False
        self.disturbance = "无"
        self.disturbance_until = 0.0
        self.fault_node: str | None = None
        self.loss_until = 0.0
        self.trail_seconds = trail_seconds_for_duration(self.duration)
        self.nodes: list[NodeState] = []
        self.links: list[LinkState] = []
        self.reset()

    def set_trail_seconds(self, seconds: float) -> None:
        """设置尾迹保留时长。注意：0 表示关闭尾迹缓存与显示。"""
        # 负数输入按关闭处理，保证外部控件和脚本调用都落到同一边界。
        self.trail_seconds = max(0.0, seconds)
        if self.trail_seconds <= 0.0:
            # Mock 节点直接持有 trail，关闭时逐节点清空即可让下一帧无尾迹。
            for node in self.nodes:
                node.trail.clear()
            return
        for node in self.nodes:
            # 切到更短时长时立即裁剪旧点，避免下一帧前仍显示超长尾迹。
            node.trail = prune_trail(node.trail, self.time, self.trail_seconds)

    def reset(self) -> Snapshot:
        """复位 MockSimulation 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        # 时间归零并清空所有扰动相关计时器，回到“待命”初始态。
        self.time = 0.0
        self.running = False
        self.paused = False
        self.disturbance = "无"
        self.disturbance_until = 0.0
        self.fault_node = None
        self.loss_until = 0.0
        # 预置三机楔形编队：1 长机 + 2 僚机，坐标为演示用初值。
        self.nodes = [
            NodeState("A01", "leader", 140.0, 260.0, 5.2, -0.1),
            NodeState("A02", "wing", 92.0, 318.0, 5.0, 0.0),
            NodeState("A03", "wing", 88.0, 202.0, 5.0, 0.0),
        ]
        self.links = [
            LinkState("A01", "A02", "duplex", 18, 0.01),
            LinkState("A01", "A03", "duplex", 21, 0.01),
            LinkState("A02", "A03", "duplex", 30, 0.02),
        ]
        return self.snapshot()

    def start(self) -> Snapshot:
        """启动或继续 MockSimulation 的运行流程。注意：重复调用应保持状态一致。"""
        self.running = True
        self.paused = False
        return self.snapshot()

    def pause(self) -> Snapshot:
        """暂停 MockSimulation 的运行流程。注意：只暂停调度，不清空当前状态。"""
        decision = self.playback_vm.command_for_pause_request(self.snapshot().run_state)
        if decision.should_pause and self.running:
            self.paused = True
        return self.snapshot()

    def set_speed(self, speed: float) -> None:
        """设置 MockSimulation 播放速度。注意：与真实 adapter 保持同名接口。"""
        playback_update = self.playback_vm.on_rate_requested(speed)
        self.speed = playback_update.display_rate

    def single_step(self) -> Snapshot:
        """执行单步推进。注意：仅在暂停或可单步状态下使用。"""
        # 进入“运行且暂停”态后只推进一拍，模拟逐帧调试。
        self.running = True
        self.paused = True
        self.advance()
        return self.snapshot()

    def inject_disturbance(self, kind: str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
        # 各扰动设置“持续到 disturbance_until 时刻”的窗口，到期后自动恢复。
        if kind == "wind":
            self.disturbance = "风场"
            self.disturbance_until = self.time + 8.0
        elif kind == "fault":
            # 节点故障锁定 A02：advance 中对该节点用更弱的控制增益模拟失效。
            self.disturbance = "节点故障"
            self.fault_node = "A02"
            self.disturbance_until = self.time + 10.0
        elif kind == "loss":
            # 链路丢包额外维护 loss_until，供链路退化判断使用。
            self.disturbance = "链路丢包"
            self.loss_until = self.time + 12.0
            self.disturbance_until = self.time + 12.0
        elif kind == "clear":
            # 清除：复位所有扰动计时器与故障节点标记。
            self.disturbance = "无"
            self.disturbance_until = 0.0
            self.loss_until = 0.0
            self.fault_node = None
        return self.snapshot()

    def advance(self) -> Snapshot:
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
        # 到达总时长则停机，不再推进时间。
        if self.time >= self.duration:
            self.running = False
            self.paused = False
            return self.snapshot()

        # 按播放倍率推进仿真时间，并夹在 duration 内防止越界。
        self.time = min(self.duration, self.time + self.step * self.speed)
        # 风场扰动时给出非零侧向风强度，否则为 0。
        wind = 1.8 if self.disturbance == "风场" else 0.0
        # 楔形队形相对长机的横/纵偏置：长机在前，两僚机在后两侧。
        formation = [(0.0, 0.0), (-54.0, 58.0), (-54.0, -58.0)]
        leader = self.nodes[0]

        for index, node in enumerate(self.nodes):
            _, dy = formation[index]
            # 长机沿正弦轨迹机动，僚机则跟随长机纵向位置加各自队形偏移。
            target_y = 238.0 + math.sin(self.time / 8.0) * 34.0 if index == 0 else leader.y + dy
            # 故障节点用更小的跟踪增益，表现为“跟不上、收敛慢”。
            gain = 0.012 if self.fault_node == node.node_id else 0.04
            node.vx = 4.8 + index * 0.12
            # 纵向速度向目标 y 收敛，并叠加随相位变化的风扰。
            node.vy += (target_y - node.y) * gain + wind * math.sin(self.time + index)
            # x 方向额外乘 3.2 让画面横向推进更明显（纯演示系数）。
            node.x += node.vx * self.step * self.speed * 3.2
            node.y += node.vy * self.step * self.speed
            # 飞出右边界后从左侧重新进入，并清空尾迹避免横贯屏幕的连线。
            if node.x > WORLD_WIDTH + 60.0:
                node.x = -30.0
                node.trail.clear()
            # 0 秒代表关闭尾迹：演示数据源也必须立即清空已有线段。
            if self.trail_seconds <= 0.0:
                node.trail.clear()
            else:
                # 追加当前采样点并裁掉超过保留时长的旧点。
                _append_trail_point(node.trail, node.x, node.y, node_altitude(index, self.time), self.time)
                node.trail = prune_trail(node.trail, self.time, self.trail_seconds)

        # 扰动窗口到期后自动清除，恢复正常显示。
        if self.disturbance != "无" and self.time > self.disturbance_until:
            self.disturbance = "无"
            self.fault_node = None

        for index, link in enumerate(self.links):
            # 丢包扰动期内除第三条外的链路进入退化态（高丢包高延迟）。
            degraded = self.time < self.loss_until and index != 2
            link.loss = 0.26 + index * 0.05 if degraded else 0.01 + index * 0.006
            link.latency_ms = 76 + index * 8 if degraded else 18 + index * 5 + round(math.sin(self.time + index) * 3)
            # 丢包率超过 20% 视为链路异常。
            link.ok = link.loss < 0.2

        return self.snapshot()

    def snapshot(self) -> Snapshot:
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
        # 运行状态与“控制回报”文案根据当前是否运行/暂停/扰动类型联合决定。
        if not self.running:
            run_state = "READY"
            report = "待命"
        elif self.paused:
            run_state = "PAUSED"
            report = "保持"
        elif self.disturbance == "风场":
            # 运行中且处于各类扰动时，给出对应的控制策略回报文案。
            run_state = "RUNNING"
            report = "抗风"
        elif self.disturbance == "节点故障":
            run_state = "RUNNING"
            report = "重构"
        elif self.disturbance == "链路丢包":
            run_state = "RUNNING"
            report = "保链"
        else:
            # 无扰动的正常运行：集结。
            run_state = "RUNNING"
            report = "集结"
        return Snapshot(
            time=self.time,
            duration=self.duration,
            step=self.step,
            run_state=run_state,
            control_report=report,
            disturbance=self.disturbance,
            nodes=self.nodes,
            links=self.links,
            route=ReferenceRoute(40.0, 238.0, 1200.0, WORLD_WIDTH - 40.0, 238.0, 1200.0),
            route_segments=[ReferenceRoute(40.0, 238.0, 1200.0, WORLD_WIDTH - 40.0, 238.0, 1200.0)],
            cpu_utilization=0.0,
        )


class ControllerSimulationAdapter:
    """把 SimulationController 快照适配为现有 GUI 绘图模型。注意：需要维护尾迹缓存。"""

    def __init__(self) -> None:
        """初始化 ControllerSimulationAdapter 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.controller = SimulationController()
        self.speed = 1.0
        self.playback_vm = PlaybackViewModel()
        self.disturbance = "无"
        # 控制器只给瞬时位置，尾迹需由本适配器按 node_id 自行累积缓存。
        self._trail_by_node: dict[str, list[TrailPoint]] = {}
        self.trail_seconds = trail_seconds_for_duration(0.0)
        # 记录上一帧位置与时间，用于差分估算速度（控制器速度字段不一定可靠）。
        self._last_xy_by_node: dict[str, tuple[float, float, float]] = {}
        # 已消费的事件数游标，避免重复处理历史扰动事件。
        self._processed_event_count = 0
        # 3D 态势显示用地形文件，只由 GUI 读取，不传入控制器算法闭环。
        self.terrain_display_file: str | None = None
        # 缓存最近一次控制器调用的返回码/消息，供 UI 记录日志与判断成败。
        self.last_result_code = "OK"
        self.last_result_message = ""

    @property
    def time(self) -> float:
        """返回当前仿真时间。注意：单位为秒。"""
        return self.controller.get_snapshot().time_s

    def set_trail_seconds(self, seconds: float) -> None:
        """设置尾迹保留时长。注意：0 表示关闭尾迹缓存与显示。"""
        # 控制器适配层同样夹到非负，避免调用端绕过 spinbox 后产生负窗口。
        self.trail_seconds = max(0.0, seconds)
        if self.trail_seconds <= 0.0:
            # 缓存整体清掉，后续转换快照时不会再把旧轨迹带回 NodeState。
            self._trail_by_node.clear()
            return
        # 当前时间来自控制器快照，确保裁剪基准和后续 _convert_snapshot 一致。
        current_time = self.controller.get_snapshot().time_s
        for trail in self._trail_by_node.values():
            # 切到更短时长时立即裁剪缓存，避免后续快照继续携带旧点。
            trail[:] = prune_trail(trail, current_time, self.trail_seconds)

    def load_config(self, path: str) -> Snapshot:
        """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""
        result = self.controller.load_config(path)
        self.last_result_code = result.code
        self.last_result_message = result.message
        # 仅在加载成功时重置缓存：清空旧尾迹/速度缓存，扰动复位为“无”。
        if result.code == "OK":
            self.terrain_display_file = _terrain_display_file_from_config(path)
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
            playback_update = self.playback_vm.on_config_loaded(self.controller.playback_rate)
            self.speed = playback_update.display_rate
            # 数据源自身也同步半程尾迹，保证非 MainWindow 调用 load_config 时行为一致。
            self.set_trail_seconds(trail_seconds_for_duration(self.controller.get_snapshot().duration_s))
            # 把事件游标推到当前末尾，避免把加载前的旧事件当成新扰动消费。
            self._processed_event_count = len(self.controller.get_recent_events(limit=1000))
            self.disturbance = "无"
        return self.snapshot()

    def start(self) -> Snapshot:
        """启动或继续 ControllerSimulationAdapter 的运行流程。注意：重复调用应保持状态一致。"""
        result = self.controller.start()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def start_rally(self) -> Snapshot:
        """开始集结流程。注意：只触发集结命令，不改变播放/暂停状态。"""
        result = self.controller.start_rally()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def pause(self) -> Snapshot:
        """暂停 ControllerSimulationAdapter 的运行流程。注意：只暂停调度，不清空当前状态。"""
        # 暂停语义（含 PAUSED 幂等、非法态报错）由控制器状态机独家裁决，适配器不复刻守卫。
        result = self.controller.pause()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def single_step(self) -> Snapshot:
        """执行单步推进。注意：仅在暂停或可单步状态下使用。"""
        result = self.controller.step()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def reset(self) -> Snapshot:
        """复位 ControllerSimulationAdapter 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        result = self.controller.reset()
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            playback_update = self.playback_vm.on_reset()
            self.speed = playback_update.display_rate
            # 控制器 reset 会按配置重建模块，需要把 UI 当前倍率重新下发给墙钟调度。
            if playback_update.controller_rate is not None:
                self.controller.set_playback_rate(playback_update.controller_rate)
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
            self.disturbance = "无"
        return self.snapshot()

    def poll(self) -> Snapshot:
        """轮询当前快照。注意：该操作不推进仿真。"""

        return self.snapshot()

    def advance(self) -> Snapshot:
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
        return self.poll()

    def snapshot(self) -> Snapshot:
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
        return self._convert_snapshot(self.controller.get_snapshot())

    def inject_disturbance(self, kind: str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
        command = self._disturbance_command(kind)
        result = self.controller.inject_disturbance(command)
        self.last_result_code = result.code
        self.last_result_message = result.message
        # 注入成功后立即把本地显示标签同步为中文名，无需等事件回流。
        if result.code == "OK":
            self.disturbance = {
                "wind": "风场",
                "fault": "节点故障",
                "loss": "链路丢包",
                "clear": "无",
            }[kind]
        return self.snapshot()

    def apply_avoidance_route(self, route: list[WayPointInputS]) -> Snapshot:
        """采用一条避障规划航线，替换长机航线。注意：成功后清空尾迹缓存（航线已变）。"""
        result = self.controller.apply_avoidance_route(route)
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
        return self.snapshot()

    def clear_avoidance_route(self) -> Snapshot:
        """清除避障航线覆盖，恢复配置原始长机航线。"""
        result = self.controller.clear_avoidance_route()
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
        return self.snapshot()

    def formation_names(self) -> list[str]:
        """返回当前配置的队形名字列表。注意：索引即 switch_formation 下发的整型队形号。"""
        return self.controller.get_formation_names()

    def formation_index(self) -> int:
        """返回当前队形索引。注意：供界面下拉框预选。"""
        return self.controller.get_formation_index()

    def switch_formation(self, index: int) -> Snapshot:
        """运行时热切换编队队形。注意：不清尾迹，保留切换过程轨迹供观察。"""
        result = self.controller.switch_formation(index)
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def set_speed(self, speed: float) -> None:
        """设置播放速度。注意：只影响界面或控制器调度倍率。"""
        playback_update = self.playback_vm.on_rate_requested(speed)
        # 记录倍率并下发给控制器调度（影响推进节奏，不影响本适配器换算）。
        self.speed = playback_update.display_rate
        if playback_update.controller_rate is not None:
            self.controller.set_playback_rate(playback_update.controller_rate)

    def set_duration(self, duration_s: float) -> Snapshot:
        """设置仿真总时长。注意：只改变停止边界，不改变步长。"""
        result = self.controller.set_duration(duration_s)
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            # 修改仿真总时长等价于重新定义默认尾迹窗口，立即裁剪缓存。
            self.set_trail_seconds(trail_seconds_for_duration(duration_s))
        return self.snapshot()

    def close(self) -> None:
        """释放 ControllerSimulationAdapter 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self.controller.close()

    def _convert_snapshot(self, snapshot: ControllerSnapshot) -> Snapshot:
        """把控制器快照转换为 GUI 绘图模型。注意：需要同步维护轨迹缓存和显示字段。"""
        # 先把事件流里的扰动状态同步过来，使显示与控制器内部状态一致。
        self._sync_disturbance_from_events()
        nodes: list[NodeState] = []
        for node in snapshot.nodes:
            previous = self._last_xy_by_node.get(node.node_id)
            if previous is None:
                # 首帧无历史可差分，直接采用控制器给出的速度分量。
                vx = node.vx_mps
                vy = node.vy_mps
            else:
                # 只有仿真时间推进时才用位移差分；暂停同帧刷新时保留控制器速度，避免机头归零朝东。
                previous_x, previous_y, previous_time = previous
                dt = snapshot.time_s - previous_time
                if dt > 1e-9:
                    vx = (node.x_m - previous_x) / dt
                    vy = (node.y_m - previous_y) / dt
                else:
                    vx = node.vx_mps
                    vy = node.vy_mps
            self._last_xy_by_node[node.node_id] = (node.x_m, node.y_m, snapshot.time_s)

            # 控制器自身不保存 GUI 尾迹，关闭时需要删除本地缓存防止重新开启后残留旧线。
            if self.trail_seconds <= 0.0:
                self._trail_by_node.pop(node.node_id, None)
                trail: list[TrailPoint] = []
            else:
                # 取出该节点尾迹缓存；仅当时间戳推进时追加新点，避免同一帧重复入栈。
                trail = self._trail_by_node.setdefault(node.node_id, [])
                if not trail or trail[-1].time != snapshot.time_s:
                    _append_trail_point(trail, node.x_m, node.y_m, node.altitude_m, snapshot.time_s)
                # 裁剪阈值跟随工具栏输入，保留同一列表对象便于后续引用稳定。
                trail[:] = prune_trail(trail, snapshot.time_s, self.trail_seconds)
            nodes.append(
                NodeState(
                    node_id=node.node_id,
                    role=node.role,
                    x=node.x_m,
                    y=node.y_m,
                    vx=vx,
                    vy=vy,
                    altitude=node.altitude_m,
                    vertical_speed=node.vz_mps,
                    health=node.health,
                    trail=list(trail),
                    cross_track_error=node.cross_track_error_m,
                    distance_to_go=node.distance_to_go_m,
                    track_pos_err_x=node.track_pos_err_x_m,
                    track_pos_err_y=node.track_pos_err_y_m,
                    track_pos_err_z=node.track_pos_err_z_m,
                    cmd_pos_x=node.cmd_pos_east_m,
                    cmd_pos_y=node.cmd_pos_north_m,
                    rally_phase=node.rally_phase,
                )
            )

        links: list[LinkState] = []
        for link in snapshot.links:
            # 链路 id 形如 "A01-A02"，按短横线拆出源/目标节点。
            source, _, target = link.link_id.partition("-")
            links.append(
                LinkState(
                    source=source,
                    target=target,
                    direction=link.direction,
                    latency_ms=round(link.latency_ms),
                    loss=link.loss_rate,
                    ok=link.status == "normal",
                )
            )
        # 兼容“单航线”与“多航段”两种来源：优先多航段，缺省时用单航线兜底。
        route = None
        if snapshot.route is not None:
            route = self._convert_route(snapshot.route)
        route_segments = [
            self._convert_route(segment)
            for segment in snapshot.route_segments
        ]
        if not route_segments and route is not None:
            route_segments = [route]
        rally_geometry = [
            RallyGeometryView(
                node_id=node_id,
                slot_x=geometry.slot_east_m,
                slot_y=geometry.slot_north_m,
                center_x=geometry.loiter_center_east_m,
                center_y=geometry.loiter_center_north_m,
                radius=geometry.loiter_radius_m,
                entry_x=geometry.entry_east_m,
                entry_y=geometry.entry_north_m,
                local_center_x=geometry.local_center_east_m,
                local_center_y=geometry.local_center_north_m,
                local_radius=geometry.local_radius_m,
                local_tangent_x=geometry.local_tangent_east_m,
                local_tangent_y=geometry.local_tangent_north_m,
                fallback_used=geometry.fallback_used,
            )
            for node_id, geometry in snapshot.rally_geometry.items()
        ]
        return Snapshot(
            time=snapshot.time_s,
            duration=snapshot.duration_s,
            step=snapshot.step_s,
            run_state=snapshot.run_state,
            control_report=snapshot.control_report,
            disturbance=self._visible_disturbance(snapshot),
            nodes=nodes,
            links=links,
            route=route,
            route_segments=route_segments,
            cpu_utilization=snapshot.cpu_utilization,
            rally_geometry=rally_geometry,
            terrain_display_file=self.terrain_display_file,
        )

    @staticmethod
    def _convert_route(route) -> ReferenceRoute:  # noqa: ANN001
        """把控制器航线状态转换为 GUI 参考航线。注意：空航线返回空值。"""
        return ReferenceRoute(
            start_x=route.start_x_m,
            start_y=route.start_y_m,
            start_altitude=route.start_altitude_m,
            end_x=route.end_x_m,
            end_y=route.end_y_m,
            end_altitude=route.end_altitude_m,
            radius=route.radius_m,
            center_x=route.center_x_m,
            center_y=route.center_y_m,
            turn_sign=route.turn_sign,
        )

    def _visible_disturbance(self, snapshot: ControllerSnapshot) -> str:
        """返回当前界面应显示的扰动名称。注意：已清除或过期扰动显示为无。"""
        # 优先按实际效果判定：有节点异常 → 节点故障，有链路异常 → 链路丢包。
        if any(node.health != "normal" for node in snapshot.nodes):
            return "节点故障"
        if any(link.status != "normal" for link in snapshot.links):
            return "链路丢包"
        # 就绪态且本地也认为无扰动时显式返回“无”，避免残留旧标签。
        if snapshot.run_state == "READY" and self.disturbance == "无":
            return "无"
        return self.disturbance

    def _sync_disturbance_from_events(self) -> None:
        """根据控制器事件同步扰动显示状态。注意：只处理尚未消费的新事件。"""
        events = self.controller.get_recent_events(limit=1000)
        # 只遍历游标之后的新事件，按消息关键字解析出当前应显示的扰动名称。
        for event in events[self._processed_event_count:]:
            if event.source != "Disturbance":
                continue
            if event.message == "清除扰动" or event.message.startswith("扰动结束"):
                self.disturbance = "无"
            elif "wind" in event.message:
                self.disturbance = "风场"
            elif "node_fault" in event.message:
                self.disturbance = "节点故障"
            elif "link_loss" in event.message or "link_fault" in event.message:
                self.disturbance = "链路丢包"
        # 推进游标到末尾，下次只处理更新的事件。
        self._processed_event_count = len(events)

    def _disturbance_command(self, kind: str) -> dict[str, object]:
        """生成 GUI 按钮对应的扰动命令。注意：命令结构需与控制器注入接口一致。"""
        # 把 UI 按钮种类翻译为控制器扰动命令字典；目标节点/链路与参数为预设演示值。
        if kind == "wind":
            return {"type": "wind", "duration_s": 8.0, "params": {"speed_mps": 8.0, "direction_deg": 90.0}}
        if kind == "fault":
            # 节点故障：目标 A02，降级模式持续 10s。
            return {"type": "node_fault", "target": "A02", "duration_s": 10.0, "params": {"mode": "degraded"}}
        if kind == "loss":
            return {"type": "link_loss", "target": "A01-A02", "duration_s": 12.0, "params": {"loss_rate": 0.3}}
        # 其余（clear）统一下发清除命令。
        return {"type": "clear"}

def node_altitude(index: int, time_value: float) -> float:
    """读取节点高度用于侧视图显示。注意：缺省时使用 0 作为兜底。"""

    # 基准高度 1200m，按机序错开 35m 层差，再叠加随时间起伏的正弦扰动。
    return 1200.0 + index * 35.0 + math.sin(time_value / 6.0 + index) * 12.0


def _terrain_display_file_from_config(path: str) -> str | None:
    """从主配置读取 3D 地形文件路径。注意：该字段只影响显示层。"""

    config_path = Path(path)
    try:
        text = config_path.read_text(encoding="utf-8")
        if config_path.suffix.lower() == ".json":
            data = json.loads(text)
        elif config_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError:
                return None
            data = yaml.safe_load(text)
        else:
            return None
    except (OSError, json.JSONDecodeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    raw_file = data.get("terrain_display_file")
    if not isinstance(raw_file, str) or not raw_file.strip():
        return None
    display_path = Path(raw_file)
    if not display_path.is_absolute():
        display_path = config_path.parent / display_path
    return str(display_path.resolve())
