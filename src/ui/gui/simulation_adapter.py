"""GUI 数据源适配层。注意：把真实控制器或演示数据统一成 Snapshot。"""

from __future__ import annotations

from collections.abc import Callable
import threading
from pathlib import Path

from src.runner.sim_control import (
    AvoidancePlanOutcome,
    CommandResult,
    DisturbanceType,
    GeoReference,
    GuiConfigData,
    ObstacleSpec,
    PlannedRoute,
    apply_planned_route,
    export_planned_route,
    geodetic_from_enu,
    load_gui_config,
    persist_config_duration,
    plan_route_for_gui,
    planned_route_from_waypoints,
    route_export_defaults,
)
from src.runner.sim_control import LinkState as ControllerLinkState
from src.runner.sim_control import NodeState as ControllerNodeState
from src.runner.sim_control import RouteState as ControllerRouteState
from src.runner.sim_control import SimulationController, TimedSnapshotCursor
from src.runner.sim_control import SimulationSnapshot as ControllerSnapshot
from src.ui.gui.disturbance_view_model import active_disturbance_text, disturbance_action
from src.ui.gui.playback_view_model import PlaybackViewModel
from src.ui.gui.trail_view_model import TrailBuffer
from src.ui.gui.view_models import (
    LinkState,
    NodeState,
    RallyGeometryView,
    ReferenceRoute,
    Snapshot,
    trail_seconds_for_duration,
)


class ControllerSimulationAdapter:
    """把 SimulationController 快照适配为现有 GUI 绘图模型。注意：需要维护尾迹缓存。"""

    def __init__(self) -> None:
        """初始化 ControllerSimulationAdapter 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.controller = SimulationController()
        self.speed = 1.0
        self.playback_vm = PlaybackViewModel()
        # 普通显示快照只给瞬时位置；固定时钟批次由本适配器按 node_id 累积成尾迹。
        self._trail_by_node: dict[str, TrailBuffer] = {}
        # 游标消费控制器固定仿真时钟快照；GUI 轮询再慢也不会漏掉中间尾迹节点。
        self._trail_cursor: TimedSnapshotCursor | None = None
        self.trail_seconds = trail_seconds_for_duration(0.0)
        # 记录上一帧位置与时间，用于差分估算速度（控制器速度字段不一定可靠）。
        self._last_xy_by_node: dict[str, tuple[float, float, float]] = {}
        # 3D 态势显示用地形文件，只由 GUI 读取，不传入控制器算法闭环。
        self.terrain_display_file: str | None = None
        # 控制器成功加载后由 runner 应用层提供 GUI 辅助配置，界面不再自行读配置文件。
        self.gui_config = GuiConfigData()
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
        previous_seconds = self.trail_seconds
        self.trail_seconds = max(0.0, seconds)
        if self.trail_seconds <= 0.0:
            # 缓存整体清掉，后续转换快照时不会再把旧轨迹带回 NodeState。
            self._reset_trail_state()
            # 关闭期间仍推进游标，重新开启时不会把用户明确隐藏的历史一次性回填。
            self._trail_cursor, _ = self.controller.read_timed_snapshots(self._trail_cursor)
            return
        if previous_seconds <= 0.0:
            # 重新开启从当前机位建立新队列；此前固定时钟样本已在关闭分支被跳过。
            self._reset_trail_state()
            self._trail_cursor, _ = self.controller.read_timed_snapshots(self._trail_cursor)
            self._append_trail_sample(self.controller.get_snapshot())
        # 当前时间来自控制器快照，确保裁剪基准和后续 _convert_snapshot 一致。
        current_time = self.controller.get_snapshot().time_s
        for trail in self._trail_by_node.values():
            # 切到更短时长时只弹出队首过期点，不扫描或复制整个缓存。
            trail.expire(current_time, self.trail_seconds)

    def load_config(self, path: str) -> Snapshot:
        """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""
        result = self.controller.load_config(path)
        self._record_result(result)
        # 仅在加载成功时重置缓存：清空旧尾迹/速度缓存，扰动复位为“无”。
        if result.code == "OK":
            self.gui_config = load_gui_config(path)
            self.terrain_display_file = self.gui_config.terrain_display_file
            # 后台预热 3D 高度场缓存:用户打开 3D 窗口时直接命中,避免主线程卡数秒。
            _warm_terrain_field_cache(self.terrain_display_file)
            self._reset_trail_state(reset_velocity=True)
            playback_update = self.playback_vm.on_config_loaded(self.controller.playback_rate)
            self.speed = playback_update.display_rate
            # 数据源自身也同步半程尾迹，保证非 MainWindow 调用 load_config 时行为一致。
            self.set_trail_seconds(trail_seconds_for_duration(self.controller.get_snapshot().duration_s))
        return self.snapshot()

    def start(self) -> Snapshot:
        """启动或继续 ControllerSimulationAdapter 的运行流程。注意：重复调用应保持状态一致。"""
        return self._run_controller_command(self.controller.start)

    def start_rally(self) -> Snapshot:
        """开始集结流程。注意：只触发集结命令，不改变播放/暂停状态。"""
        return self._run_controller_command(self.controller.start_rally)

    def pause(self) -> Snapshot:
        """暂停 ControllerSimulationAdapter 的运行流程。注意：只暂停调度，不清空当前状态。"""
        # 暂停语义（含 PAUSED 幂等、非法态报错）由控制器状态机独家裁决，适配器不复刻守卫。
        return self._run_controller_command(self.controller.pause)

    def single_step(self) -> Snapshot:
        """执行单步推进。注意：仅在暂停或可单步状态下使用。"""
        return self._run_controller_command(self.controller.step)

    def reset(self) -> Snapshot:
        """复位 ControllerSimulationAdapter 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        result = self.controller.reset()
        self._record_result(result)
        if result.code == "OK":
            playback_update = self.playback_vm.on_reset()
            self.speed = playback_update.display_rate
            # 控制器 reset 会按配置重建模块，需要把 UI 当前倍率重新下发给墙钟调度。
            if playback_update.controller_rate is not None:
                self.controller.set_playback_rate(playback_update.controller_rate)
            self._reset_trail_state(reset_velocity=True)
        return self.snapshot()

    def poll(self) -> Snapshot:
        """轮询当前快照。注意：该操作不推进仿真。"""

        return self.snapshot()

    def advance(self) -> Snapshot:
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
        return self.poll()

    def snapshot(self) -> Snapshot:
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
        controller_snapshot = self.controller.get_snapshot()
        self._synchronize_trails(controller_snapshot)
        return self._convert_snapshot(controller_snapshot)

    def inject_disturbance(self, kind: DisturbanceType | str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
        action = disturbance_action(kind)
        return self._run_controller_command(
            lambda: self.controller.inject_disturbance(action.command)
        )

    def plan_avoidance_route(
        self,
        waypoints: list[tuple[float, float, float]],
        obstacles: list[ObstacleSpec],
        **kwargs: object,
    ) -> AvoidancePlanOutcome:
        """规划避障航线。注意：算法类型与转换全部封装在 runner 应用层。"""

        return plan_route_for_gui(waypoints, obstacles, **kwargs)

    def route_export_defaults(self, config_path: Path) -> tuple[Path, str]:
        """获取航线导出默认路径与过滤器。"""

        return route_export_defaults(config_path)

    def export_route(
        self,
        config_path: Path,
        route_path: Path,
        route: PlannedRoute,
        speed_mps: float,
        geo_reference: GeoReference | None,
    ) -> Path:
        """输出规划航线。注意：具体格式与数据转换由 runner 应用层负责。"""

        normalized_route = route if isinstance(route, PlannedRoute) else planned_route_from_waypoints(route)
        return export_planned_route(config_path, route_path, normalized_route, speed_mps, geo_reference)

    def persist_duration(self, config_path: Path, duration_s: float) -> None:
        """把时长写回主配置。注意：只更新 duration_s 字段。"""

        persist_config_duration(config_path, duration_s)

    def to_geodetic(
        self,
        east_m: float,
        north_m: float,
        reference: GeoReference | None = None,
    ) -> tuple[float, float] | None:
        """把 ENU 点击坐标转换为经纬度。注意：无地理原点时返回 None。"""

        return geodetic_from_enu(east_m, north_m, reference or self.gui_config.geo_reference)

    def apply_avoidance_route(self, route: PlannedRoute) -> Snapshot:
        """采用一条避障规划航线，替换长机航线。注意：成功后清空尾迹缓存（航线已变）。"""
        normalized_route = route if isinstance(route, PlannedRoute) else planned_route_from_waypoints(route)
        result = apply_planned_route(self.controller, normalized_route)
        self._record_result(result)
        if result.code == "OK":
            self._reset_trail_state(reset_velocity=True)
        return self.snapshot()

    def clear_avoidance_route(self) -> Snapshot:
        """清除避障航线覆盖，恢复配置原始长机航线。"""
        result = self.controller.clear_avoidance_route()
        self._record_result(result)
        if result.code == "OK":
            self._reset_trail_state(reset_velocity=True)
        return self.snapshot()

    def formation_names(self) -> list[str]:
        """返回当前配置的队形名字列表。注意：索引即 switch_formation 下发的整型队形号。"""
        return self.controller.get_formation_names()

    def formation_index(self) -> int:
        """返回当前队形索引。注意：供界面下拉框预选。"""
        return self.controller.get_formation_index()

    def switch_formation(self, index: int) -> Snapshot:
        """运行时热切换编队队形。注意：不清尾迹，保留切换过程轨迹供观察。"""
        return self._run_controller_command(lambda: self.controller.switch_formation(index))

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
        self._record_result(result)
        if result.code == "OK":
            # 修改仿真总时长等价于重新定义默认尾迹窗口，立即裁剪缓存。
            self.set_trail_seconds(trail_seconds_for_duration(duration_s))
        return self.snapshot()

    def close(self) -> None:
        """释放 ControllerSimulationAdapter 持有的资源。注意：关闭后不应继续调用运行接口。"""
        # 关闭属于尾迹生命周期终点，必须丢弃全部节点队列与差分速度基准。
        self._reset_trail_state(reset_velocity=True, reset_cursor=True)
        self.controller.close()

    def _record_result(self, result: CommandResult) -> None:
        """保存最近一次控制器命令结果，供主窗口统一记录与判断。"""

        self.last_result_code = result.code
        self.last_result_message = result.message

    def _run_controller_command(self, command: Callable[[], CommandResult]) -> Snapshot:
        """执行无额外适配器副作用的控制器命令，并返回转换后的最新快照。"""

        self._record_result(command())
        return self.snapshot()

    def _reset_trail_state(
        self,
        *,
        reset_velocity: bool = False,
        reset_cursor: bool = False,
    ) -> None:
        """清理尾迹，并按生命周期边界选择是否同步清理速度基准和采样游标。"""

        self._trail_by_node.clear()
        if reset_velocity:
            self._last_xy_by_node.clear()
        if reset_cursor:
            self._trail_cursor = None

    def _synchronize_trails(self, current_snapshot: ControllerSnapshot) -> None:
        """消费固定时钟样本并裁剪队列。注意：当前显示机位始终留在队列外。"""

        previous_cursor = self._trail_cursor
        next_cursor, timed_snapshots = self.controller.read_timed_snapshots(previous_cursor)
        generation_changed = (
            previous_cursor is None
            or previous_cursor.run_generation != next_cursor.run_generation
        )
        self._trail_cursor = next_cursor
        if self.trail_seconds <= 0.0:
            # 即使关闭尾迹也已消费到最新游标，防止重新开启时回灌隐藏区间。
            self._reset_trail_state()
            return
        if generation_changed:
            # 配置加载、reset 与航线重建会换运行代，旧队列不能跨代连接。
            self._reset_trail_state()
        if timed_snapshots:
            for timed_snapshot in timed_snapshots:
                self._append_trail_sample(timed_snapshot)
        elif generation_changed or not self._trail_by_node:
            # 正式 10 Hz 首样本在 0.1 秒；运行起点或显式重启边界先保留一个稳定锚点。
            self._append_trail_sample(current_snapshot)
        for trail in self._trail_by_node.values():
            # 统一以当前显示时间裁剪，批量补入多点时仍只需在末尾弹头一次。
            trail.expire(current_snapshot.time_s, self.trail_seconds)

    def _append_trail_sample(self, snapshot: ControllerSnapshot) -> None:
        """把一个固定仿真时钟快照追加到各机队列。注意：不负责窗口淘汰。"""

        for node in snapshot.nodes:
            trail = self._trail_by_node.get(node.node_id)
            if trail is None:
                trail = TrailBuffer()
                self._trail_by_node[node.node_id] = trail
            trail.append_position(node.x_m, node.y_m, node.altitude_m, snapshot.time_s)

    def _convert_snapshot(self, snapshot: ControllerSnapshot) -> Snapshot:
        """把控制器快照转换为 GUI 绘图模型。注意：需要同步维护轨迹缓存和显示字段。"""
        nodes = [self._convert_node(node, snapshot.time_s) for node in snapshot.nodes]
        links = [self._convert_link(link) for link in snapshot.links]
        route, route_segments, blocked_route_segments = self._convert_routes(snapshot)
        return Snapshot(
            time=snapshot.time_s,
            duration=snapshot.duration_s,
            step=snapshot.step_s,
            run_state=snapshot.run_state,
            control_report=snapshot.control_report,
            disturbance=active_disturbance_text(snapshot.active_disturbances),
            nodes=nodes,
            links=links,
            route=route,
            route_segments=route_segments,
            blocked_route_segments=blocked_route_segments,
            cpu_utilization=snapshot.cpu_utilization,
            rally_geometry=self._convert_rally_geometry(snapshot),
            terrain_display_file=self.terrain_display_file,
        )

    def _convert_node(self, node: ControllerNodeState, snapshot_time_s: float) -> NodeState:
        """转换单个节点，并只在仿真时间前进时用位置差分修正水平速度。"""

        previous = self._last_xy_by_node.get(node.node_id)
        if previous is None:
            # 首帧无历史可差分，直接采用控制器给出的速度分量。
            velocity_x = node.vx_mps
            velocity_y = node.vy_mps
        else:
            previous_x, previous_y, previous_time = previous
            delta_time = snapshot_time_s - previous_time
            if delta_time > 1e-9:
                velocity_x = (node.x_m - previous_x) / delta_time
                velocity_y = (node.y_m - previous_y) / delta_time
            else:
                # 暂停同帧刷新保留控制器速度，避免机头因零时间差错误归零朝东。
                velocity_x = node.vx_mps
                velocity_y = node.vy_mps
        self._last_xy_by_node[node.node_id] = (node.x_m, node.y_m, snapshot_time_s)

        # 当前位置不写回稳定队列；绘制端只补一条队尾到飞机的实时末段。
        if self.trail_seconds <= 0.0:
            self._trail_by_node.pop(node.node_id, None)
            trail = []
        else:
            trail_buffer = self._trail_by_node.get(node.node_id)
            trail = trail_buffer.snapshot() if trail_buffer is not None else []
        return NodeState(
            node_id=node.node_id,
            role=node.role,
            x=node.x_m,
            y=node.y_m,
            vx=velocity_x,
            vy=velocity_y,
            altitude=node.altitude_m,
            vertical_speed=node.vz_mps,
            health=node.health,
            trail=trail,
            cross_track_error=node.cross_track_error_m,
            distance_to_go=node.distance_to_go_m,
            track_pos_err_x=node.track_pos_err_x_m,
            track_pos_err_y=node.track_pos_err_y_m,
            track_pos_err_z=node.track_pos_err_z_m,
            cmd_pos_x=node.cmd_pos_east_m,
            cmd_pos_y=node.cmd_pos_north_m,
            rally_phase=node.rally_phase,
        )

    @staticmethod
    def _convert_link(link: ControllerLinkState) -> LinkState:
        """把控制器链路状态转换为 GUI 表格与绘图使用的简化状态。"""

        # 链路 id 形如 A01-A02，按第一个短横线拆出源和目标节点。
        source, _, target = link.link_id.partition("-")
        return LinkState(
            source=source,
            target=target,
            direction=link.direction,
            latency_ms=round(link.latency_ms),
            loss=link.loss_rate,
            ok=link.status == "normal",
        )

    def _convert_routes(
        self,
        snapshot: ControllerSnapshot,
    ) -> tuple[ReferenceRoute | None, list[ReferenceRoute], list[ReferenceRoute]]:
        """转换当前、完整和封锁航线，并保持既有单航段兼容回退。"""

        route = self._convert_route(snapshot.route) if snapshot.route is not None else None
        route_segments = [self._convert_route(segment) for segment in snapshot.route_segments]
        # 旧控制器只提供当前航段时仍保留一条可绘制参考路线。
        if not route_segments and route is not None:
            route_segments = [route]
        # 封锁航线只来自权威字段，不能回退当前航段以免伪造封锁语义。
        blocked = [self._convert_route(segment) for segment in snapshot.blocked_route_segments]
        return route, route_segments, blocked

    @staticmethod
    def _convert_rally_geometry(snapshot: ControllerSnapshot) -> list[RallyGeometryView]:
        """转换集结圆显示参数。注意：运行期切线不进入 GUI 快照。"""

        return [
            RallyGeometryView(
                node_id=node_id,
                center_x=geometry.loiter_center_east_m,
                center_y=geometry.loiter_center_north_m,
                radius=geometry.loiter_radius_m,
                local_center_x=geometry.local_center_east_m,
                local_center_y=geometry.local_center_north_m,
                local_radius=geometry.local_radius_m,
            )
            for node_id, geometry in snapshot.rally_geometry.items()
        ]

    @staticmethod
    def _convert_route(route: ControllerRouteState) -> ReferenceRoute:
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

def _warm_terrain_field_cache(display_file: str | None) -> None:
    """后台线程预热高度场缓存。注意：失败静默,正式回退诊断由 scene_data 负责。"""

    if not display_file:
        return

    def _worker() -> None:
        """线程体:按正式分辨率生成一次高度场,填充进程级缓存。"""

        try:
            from src.ui.gui.situation3d import scene_data
            from src.ui.gui.situation3d.terrain_field import get_terrain_field, load_terrain_layout

            layout = load_terrain_layout(display_file)
            get_terrain_field(display_file, resolution=scene_data._layout_resolution(layout))
        except Exception:  # noqa: BLE001
            # 预热只是性能优化,任何异常都不能影响配置加载流程。
            return

    threading.Thread(target=_worker, name="terrain-field-warmup", daemon=True).start()
