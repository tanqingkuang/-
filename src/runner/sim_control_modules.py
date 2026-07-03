"""仿真控制器运行期子模块。注意：配置、算法适配、扰动和日志各自封装。"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, replace
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from typing import Any

from src.algorithm.context.leaf_types import (
    FormSelfInitS,
    FormStageE,
    FormationAnalysisS,
    MotionProfS,
    PosInEarthS,
    PosTrackDiagS,
    RallyPhaseE,
    RemoteCmdS,
    WayLineS,
    WayPointInputS,
    copy_motion,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.follower import FollowerEntity
from src.algorithm.entity.leader_follower_hold.leader import LeaderEntity
from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity
from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS, VelCmdLimitS
from src.common.envelope import MessageEnvelope
from src.data.config_loader import resolve_config_references
from src.environment.comm import CommunicationChannel
from src.environment.model import AccelerationCommand, AircraftState, ModelIterator, node_id_from_config
from src.runner.sim_control_constants import (
    _DEFAULT_ALGORITHM_DECIMATION,
    _MAX_PLAYBACK_RATE,
    _MIN_PLAYBACK_RATE,
)
from src.runner.sim_control_routes import (
    _build_formation_comm_init,
    _build_leader_route,
    _build_rally_approach_speed,
    _build_rally_route,
    _build_rally_task_init,
    _build_vel_cmd_limit,
    _motion_from_aircraft_state,
)
from src.runner.sim_control_types import (
    DisturbanceCommand,
    SimulationEvent,
    SimulationSnapshot,
    _NodeAlgorithmOutput,
)

class _ConfigLoader:
    """控制器首版使用的轻量 JSON/YAML 加载器。注意：YAML 依赖缺失时只支持 JSON。"""

    # 加载器只负责解析和结构校验，不创建模型、通信或算法实例。
    # 这样 load_config 能在锁外完成文件 IO，真正的运行状态初始化留给控制器。
    def load(self, path: str) -> dict[str, object]:
        """加载控制器配置并构造运行所需对象。注意：重复加载会覆盖当前场景。"""
        config_path = Path(path)
        if not config_path.exists():
            raise FileNotFoundError(path)
        text = config_path.read_text(encoding="utf-8")
        # 按扩展名选择解析器：JSON 内建，YAML 需可选依赖 PyYAML。
        if config_path.suffix.lower() == ".json":
            data = json.loads(text)
        elif config_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover - 依赖运行环境
                raise ValueError("YAML config requires PyYAML") from exc
            data = yaml.safe_load(text)
        else:
            raise ValueError("config must be .json, .yaml, or .yml")
        # 根必须是对象；解析后立即做结构校验再返回副本。
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        data = resolve_config_references(data, config_path)
        self.validate(data)
        return dict(data)

    def validate(self, config: dict[str, object]) -> None:
        """校验配置结构和关键字段。注意：这里只做控制器需要的基础校验。"""
        # 核心时序参数取值范围校验：时长/步长为正，倍率落在允许范围内。
        duration_s = float(config.get("duration_s", 120.0))
        step_s = float(config.get("step_s", 0.005))
        playback_rate = float(config.get("playback_rate", 1.0))
        algorithm_decimation = config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION)
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if step_s <= 0:
            raise ValueError("step_s must be positive")
        if not _MIN_PLAYBACK_RATE <= playback_rate <= _MAX_PLAYBACK_RATE:
            raise ValueError(f"playback_rate must be in [{_MIN_PLAYBACK_RATE}, {_MAX_PLAYBACK_RATE}]")
        if (
            isinstance(algorithm_decimation, bool)
            or not isinstance(algorithm_decimation, int)
            or algorithm_decimation <= 0
        ):
            raise ValueError("algorithm_decimation must be a positive integer")
        nodes = config.get("nodes", [])
        links = config.get("links", [])
        model = config.get("model", {})
        if nodes is not None and not isinstance(nodes, list):
            raise ValueError("nodes must be a list")
        if links is not None and not isinstance(links, list):
            raise ValueError("links must be a list")
        # 集结角色必填字段前置校验：早于深层构造函数，以便给出明确报错而非 AttributeError。
        node_list: list[dict] = [n for n in (nodes or []) if isinstance(n, dict)]
        has_rally_leader = any(str(n.get("role") or "") == "rally_leader" for n in node_list)
        has_rally_role = has_rally_leader or any(
            str(n.get("role") or "") == "rally_follower" for n in node_list
        )
        if has_rally_role and config.get("rally_cfg") is None:
            raise ValueError("rally_cfg is required when any node has a rally role")
        if has_rally_leader:
            if config.get("rally_route") is None:
                raise ValueError("rally_route is required for rally_leader role")
            if config.get("route") is None:
                raise ValueError("route is required for rally_leader role")
        for n in node_list:
            if str(n.get("role") or "") == "rally_follower" and n.get("rally_target") is None:
                raise ValueError(f"node {n.get('node_id')!r}: rally_target is required for rally_follower")
        # 复用构造函数做深层校验：航线、编队/通信、模型配置任一非法都会在此抛错。
        # validate 不保留这些构造结果，只利用构造函数的类型和取值检查。
        # 这样可以避免校验逻辑和实际初始化逻辑分叉。
        _build_leader_route(config)
        _build_rally_route(config)
        _build_formation_comm_init(list(nodes or []), list(links or []), config)
        _build_vel_cmd_limit(config)
        step_s_v = float(config.get("step_s", 0.005))
        decimation_v = int(config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION))
        _build_rally_task_init(config, step_s_v * decimation_v, list(nodes or []))
        _build_rally_approach_speed(config)
        ModelIterator._parse_model_config(model)



class _NodeAlgorithm:
    """把可移植编队实体 API 适配到 SimulationController。注意：负责端口数据转换。"""

    # _NodeAlgorithm 是控制器和算法实体之间的薄适配层。
    # 它持有单个节点的实体、远控阶段和首步前显示航线缓存。
    # 模型状态、通信收件箱、扰动健康状态都由控制器在 step 时注入。
    # 输出统一转成 AccelerationCommand 和诊断对象，控制器无需了解具体实体类型。
    def __init__(
        self,
        node_id: str,
        role: str,
        comm_init: FormCommInitS,
        initial_leader_state: MotionProfS | None,
        leader_route: list[WayPointInputS] | None,
        control_period_s: float,
        vel_cmd_limit: VelCmdLimitS | None = None,
        rally_route: list[WayPointInputS] | None = None,
        rally_cfg: object | None = None,
        rally_target: PosInEarthS | None = None,
        rally_leader_id: str = "",
        rally_approach_speed_mps: float = 20.0,
    ) -> None:
        """初始化 _NodeAlgorithm 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._node_id = node_id
        self._role = role
        # 初始队形索引：僚机冷启动预置 cmd.pattern 用它，避免冷启动无参考。
        self._initial_pattern = int(comm_init.initialPattern)
        # 标记长机是否已执行过算法步：未跑前 current_route 回退到航线首段。
        self._has_route_step = False
        # 集结角色从 RALLY 开始；其余角色（leader/wingman）默认 HOLD。
        self._remote_stage = FormStageE.RALLY if role in {"rally_leader", "rally_follower"} else FormStageE.HOLD
        self._initial_remote_stage = self._remote_stage
        self._rally_completed: bool = False
        # 按角色选择编队实体。
        # leader/rally_leader/rally_follower/wingman 对应不同算法实现，但对外 step 接口一致。
        if role == "leader":
            self._entity: EntityBase = LeaderEntity()
        elif role == "rally_leader":
            self._entity = RallyLeaderEntity()
        elif role == "rally_follower":
            self._entity = RallyFollowerEntity()
        else:
            self._entity = FollowerEntity()
        self._entity.init(
            EntityInitS(
                selfInit=FormSelfInitS(node_id),
                commInit=comm_init,
                route=leader_route or [],
                control_period_s=control_period_s,
                velCmdLimit=vel_cmd_limit or VelCmdLimitS(),
                rally_route=rally_route,
                rally_cfg=rally_cfg,
                rally_target=rally_target,
                rally_leader_id=rally_leader_id,
                rally_approach_speed_mps=rally_approach_speed_mps,
            )
        )
        # 保存长机初始航线（内部 WayLineS），供首步前 current_route() 回退显示。
        self._initial_route_lines: list[WayLineS] = []
        if role in {"leader", "rally_leader"}:
            for _attr in ("_tra_plan", "_tra_plan_mission"):
                tra_plan = getattr(self._entity, _attr, None)
                if tra_plan is not None and hasattr(tra_plan, "get_route"):
                    self._initial_route_lines = tra_plan.get_route()
                    break
        # 僚机预置：直接进入 HOLD/三角队形并写入长机初态，避免冷启动时无参考。
        # 这段只影响实体上下文初值，后续仍由通信和算法输出持续刷新长机状态。
        self._cold_start_leader_state: MotionProfS | None = (
            initial_leader_state
            if role not in {"leader", "rally_leader", "rally_follower"}
            else None
        )
        self._apply_cold_start_preset()

    def _apply_cold_start_preset(self) -> None:
        """将僚机冷启动预置写入实体上下文，__init__ 与 reset 共用。"""
        if self._cold_start_leader_state is not None and hasattr(self._entity, "cxt"):
            self._entity.cxt.cmd.stage = FormStageE.HOLD  # type: ignore[attr-defined]
            self._entity.cxt.cmd.pattern = self._initial_pattern  # type: ignore[attr-defined]
            copy_motion(self._cold_start_leader_state, self._entity.cxt.leaderState)  # type: ignore[attr-defined]

    def step(
        self,
        state: AircraftState,
        inbox: list[MessageEnvelope],
        time_s: float,
        health: str = "normal",
    ) -> _NodeAlgorithmOutput:
        """推进 _NodeAlgorithm 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        entity_output = EntityOutputS()
        # 算法实体只接收当前模型状态、通信收件箱、远控阶段和时间戳。
        # 健康状态不直接传给算法，而是在输出适配时用于控制器回报。
        self._entity.step(
            EntityInputS(
                selfState=_motion_from_aircraft_state(state),
                inbox=inbox,
                remote=RemoteCmdS(self._remote_stage),
                now_s=time_s,
            ),
            entity_output,
        )
        # 长机（包含集结长机）一旦跑过即标记，使 current_route() 从上下文取实时值。
        if self._role in {"leader", "rally_leader"}:
            self._has_route_step = True
        # 集结完成时自动切换为 HOLD，防止重复触发完成流程。
        # 用专用标志位锁存，与诊断载荷解耦（诊断仅一帧有效，标志持久到 reset）。
        formation_analysis = entity_output.formationAnalysis
        if not self._rally_completed and formation_analysis is not None:
            self._rally_completed = True
            self._remote_stage = FormStageE.HOLD
        # 优先用输出加速度，缺省回退到实体上下文中的加速度。
        # 部分实体实现仍把最终命令留在上下文中，这里兼容两种输出方式。
        acc_cmd = entity_output.selfAccCmd or self._entity.cxt.selfAccCmd  # type: ignore[attr-defined]
        control = AccelerationCommand(
            acc_cmd.accEast,
            acc_cmd.accNorth,
            acc_cmd.accUp,
        )
        # 给待发消息打上当前仿真时间戳，供接收端做时延/时序判断。
        outbox = [
            replace(message, timestamp=time_s)
            for message in entity_output.outbox
        ]
        # 节点非健康时上报"重构"，否则"组队"，供控制回报聚合。
        status = "reconfiguring" if health != "normal" else "forming"
        control_diag = entity_output.controlDiag or PosTrackDiagS()
        return _NodeAlgorithmOutput(control, outbox, status, control_diag, formation_analysis)

    def reset(self) -> None:
        """复位 _NodeAlgorithm 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._has_route_step = False
        self._remote_stage = self._initial_remote_stage
        self._rally_completed = False
        self._entity.reset()
        self._apply_cold_start_preset()
        return None

    def close(self) -> None:
        """释放 _NodeAlgorithm 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self._entity.close()

    def current_stage(self) -> FormStageE:
        """读取当前编队阶段。注意：返回值用于 GUI 回报显示。"""
        # 实体无上下文时视为无阶段（NONE）。
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None:
            return FormStageE.NONE
        return FormStageE(cxt.cmd.stage)

    def current_rally_phase_str(self) -> str:
        """返回人类可读的集结阶段字符串，JOINING 阶段含本机汇合状态。"""
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None:
            return ""
        stage = cxt.cmd.stage
        step = cxt.cmd.step
        if stage == FormStageE.RALLY:
            try:
                phase_e = RallyPhaseE(step)
                phase = phase_e.name
            except ValueError:
                phase = f"STEP{step}"
            if step == RallyPhaseE.JOINING:
                rally_join = getattr(self._entity, "_rally_join", None)
                join_state = getattr(rally_join, "state", "") if rally_join is not None else ""
                _join_abbr = {"FLYING": "FLY", "LOITERING": "LOIT", "EXITED": "EXIT"}
                if join_state:
                    phase = f"JN·{_join_abbr.get(join_state, join_state)}"
            return phase
        if stage == FormStageE.HOLD:
            return "HOLD"
        return ""

    def current_route(self) -> WayLineS | None:
        """读取当前航线状态。注意：返回副本避免外部改写内部状态。"""
        cxt = getattr(self._entity, "cxt", None)
        # 仅长机（含集结长机）持有航线；无上下文或非长机返回 None。
        if cxt is None or self._role not in {"leader", "rally_leader"}:
            return None
        # 算法尚未跑过时上下文 wayLine 未初始化，回退到航线首段用于初始显示。
        if not self._has_route_step and self._initial_route_lines:
            return self._initial_route_lines[0]
        return cxt.wayLine


class _DisturbanceEngine:
    """动态扰动执行器。注意：当前实现覆盖风场、节点故障和链路扰动。"""

    # 扰动引擎只记录动态影响，不改写原始配置对象。
    # 多个扰动可重叠生效；任一扰动到期后先清空再重放剩余扰动。
    # 节点健康和链路质量都保留基线/原值，clear 能回到注入前状态。
    # 模型风场、通信链路和节点健康分属不同子系统，这里统一协调撤销顺序。
    def __init__(self) -> None:
        """初始化 _DisturbanceEngine 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        # 活跃扰动列表，元素为 (命令, 到期时刻)。
        self._active: list[tuple[DisturbanceCommand, float]] = []
        self._model: ModelIterator | None = None
        self._comm: CommunicationChannel | None = None
        self._node_health: dict[str, str] = {}  # 运行期节点健康（被扰动修改）。
        self._baseline_health: dict[str, str] = {}  # 健康基线，清除扰动时恢复目标。
        self._faulted_links: set[str] = set()  # 被中断链路集合，便于恢复。
        self._degraded_links: dict[str, float] = {}  # 降级链路 -> 原始丢包率，便于回填。

    def init(
        self,
        config: dict[str, object],
        seed: int,
        model: ModelIterator,
        comm: CommunicationChannel,
    ) -> None:
        """按配置初始化 _DisturbanceEngine。注意：调用方需先准备好必要依赖和输入数据。"""
        del seed
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
        self._model = model
        self._comm = comm
        nodes = config.get("nodes") or []
        # 从配置记录各节点基线健康，作为扰动清除后的恢复目标。
        self._baseline_health = {
            node_id_from_config(node, i): str(node.get("health", "normal"))
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 当前健康初始等于基线（副本，运行期被扰动修改）。
        self._node_health = dict(self._baseline_health)

    def read_health(self) -> dict[str, str]:
        """读取扰动模块健康状态。注意：用于状态表和回报显示。"""
        return dict(self._node_health)

    def inject(self, command: DisturbanceCommand, current_time_s: float) -> SimulationEvent:
        """注入扰动命令。注意：扰动类型和目标由命令字段决定。"""
        # clear 命令撤销全部已注入扰动并复位受影响子系统。
        if command.type == "clear":
            self.clear()
            return SimulationEvent(current_time_s, "INFO", "Disturbance", "清除扰动")
        # 其余扰动登记到活跃表（带到期时刻）并立即生效。
        until_s = current_time_s + float(command.duration_s or 0.0)
        self._active.append((command, until_s))
        self._apply(command, until_s)
        return SimulationEvent(current_time_s, "INFO", "Disturbance", f"注入扰动: {command.type}")

    def tick(self, time_s: float, dt_s: float) -> list[SimulationEvent]:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
        del dt_s
        events: list[SimulationEvent] = []
        remaining: list[tuple[DisturbanceCommand, float]] = []
        had_expiry = False
        # 扫描活跃扰动，超过到期时刻的剔除并生成"扰动结束"事件。
        for command, until_s in self._active:
            if time_s > until_s:
                events.append(SimulationEvent(time_s, "INFO", "Disturbance", f"扰动结束: {command.type}"))
                had_expiry = True
                continue
            remaining.append((command, until_s))
        self._active = remaining
        # 有扰动到期时，先清空所有动态影响，再重放仍活跃的扰动——
        # 这样能正确撤销过期项，又不误伤共享同一资源的未过期项。
        if had_expiry:
            self._clear_dynamic_effects()
            for command, until_s in self._active:
                self._apply(command, until_s)
        return events

    def clear(self) -> None:
        """清除动态扰动。注意：只撤销扰动影响，不重置仿真时间。"""
        self._active = []
        self._clear_dynamic_effects()

    def _apply(self, command: DisturbanceCommand, until_s: float) -> None:
        """把扰动命令分发到对应模型或通信模块。注意：新增扰动类型需同步扩展。"""
        # wind：交给模型施加风场扰动。
        if command.type == "wind" and self._model is not None:
            self._model.inject_wind(command)
        # node_fault：把目标节点健康置为给定模式（默认 degraded），影响算法状态判定。
        elif command.type == "node_fault":
            target = str(ModelIterator._command_value(command, "target") or "")
            if target in self._node_health:
                params = ModelIterator._command_params(command)
                self._node_health[target] = str(params.get("mode", "degraded"))
        # link_fault：使目标链路中断；记录到 faulted 集合以便后续恢复。
        elif command.type == "link_fault" and self._comm is not None:
            link_id = str(command.target or "")
            if link_id:
                try:
                    self._comm.inject_link_fault(link_id, "lost")
                    self._faulted_links.add(link_id)
                except (KeyError, ValueError):
                    pass
        # link_loss：临时抬高目标链路丢包率；先保存原始丢包率以便清除时回填。
        elif command.type == "link_loss" and self._comm is not None:
            link_id = str(command.target or "")
            # 同一链路已降级则不重复处理，避免覆盖已保存的原始值。
            if link_id and link_id not in self._degraded_links:
                params = ModelIterator._command_params(command)
                rate_raw = params.get("loss_rate", 1.0)
                try:
                    # 非法/布尔丢包率退化为 1.0（完全丢包）。
                    rate = float(rate_raw) if isinstance(rate_raw, (int, float)) and not isinstance(rate_raw, bool) else 1.0
                    states = {s.link_id: s for s in self._comm.read_link_states()}
                    original = states[link_id].loss_rate if link_id in states else 0.0
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=rate)
                    self._degraded_links[link_id] = original
                except (KeyError, ValueError):
                    pass

    def _clear_dynamic_effects(self) -> None:
        """清除已注入的动态影响。注意：需要同时处理模型和通信两类扰动。"""
        # 撤风、把节点健康恢复到基线。
        if self._model is not None:
            self._model.clear_wind()
        self._node_health = dict(self._baseline_health)
        if self._comm is not None:
            # 恢复曾被中断的链路。
            for link_id in self._faulted_links:
                try:
                    self._comm.inject_link_fault(link_id, "normal")
                except (KeyError, ValueError):
                    pass
            # 把降级链路的丢包率回填为注入前的原始值。
            for link_id, original_rate in self._degraded_links.items():
                try:
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=original_rate)
                except (KeyError, ValueError):
                    pass
        # 清空跟踪集合，标记动态影响已全部撤销。
        self._faulted_links = set()
        self._degraded_links = {}

    def reset(self) -> None:
        """复位 _DisturbanceEngine 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self.clear()

    def close(self) -> None:
        """释放 _DisturbanceEngine 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
        self._model = None
        self._comm = None
        self._node_health.clear()
        self._baseline_health.clear()


class _DataLogger:
    """关键数据日志记录器。注意：同时保留内存副本并写入 JSONL 文件。"""

    # 文件日志失败后允许降级为内存日志，避免磁盘问题中断正在运行的仿真。
    # snapshots/events 分开落盘，便于 run-to-run 对比和告警检索。
    # 数值序列化统一在本类完成，控制器和快照模块不需要关心日志精度。
    # 输出目录集中在 logs/run-*，测试清理时可以按运行目录粒度处理生成物。
    _TIME_KEYS = {"time_s", "duration_s", "step_s"}
    _SNAPSHOT_OMIT_KEYS = {"step_s", "route", "route_segments"}
    _LOAD_FACTOR_KEYS = {"nx", "nz"}
    _ANGLE_SUFFIXES = ("_deg", "_deg_s")
    _ACCELERATION_SUFFIXES = ("_mps2", "_mps3")
    _SPEED_SUFFIXES = ("_mps",)
    _POSITION_SUFFIXES = ("_m",)

    def __init__(self) -> None:
        """初始化 _DataLogger 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.snapshots: list[SimulationSnapshot] = []
        self.events: list[SimulationEvent] = []
        self.opened = False
        self.run_dir: Path | None = None
        self._snapshot_file = None
        self._event_file = None
        self._file_logging_disabled = False
        self.last_error_message = ""

    def reset(self) -> None:
        """重置日志记录器状态。注意：只清当前运行，不创建文件目录。"""
        self.close()
        self.snapshots.clear()
        self.events.clear()
        self.run_dir = None
        self._file_logging_disabled = False
        self.last_error_message = ""

    def open(self, run_id: str, config: dict[str, object]) -> bool:
        """打开数据记录器资源。注意：文件打开失败时返回 False 而不打断仿真。"""
        if self.opened:
            return True
        if self._file_logging_disabled:
            return False
        try:
            self.run_dir = self._make_run_dir(run_id)
            self.run_dir.mkdir(parents=True, exist_ok=False)
            (self.run_dir / "config.json").write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            # 使用行缓冲，仿真中断时也尽量保留已记录数据。
            self._snapshot_file = (self.run_dir / "snapshots.jsonl").open("w", encoding="utf-8", buffering=1)
            self._event_file = (self.run_dir / "events.jsonl").open("w", encoding="utf-8", buffering=1)
            for event in self.events:
                self._event_file.write(json.dumps(self._serialize_record(asdict(event)), ensure_ascii=False) + "\n")
        except OSError as exc:
            self._disable_file_logging(exc)
            return False
        self.opened = True
        return True

    def write_snapshot(self, snapshot: SimulationSnapshot) -> bool:
        """写入一帧仿真快照。注意：文件失败返回 False，内存记录仍保留。"""
        self.snapshots.append(snapshot)
        if self._snapshot_file is not None:
            record = self._serialize_record(asdict(snapshot), omit_keys=self._SNAPSHOT_OMIT_KEYS)
            try:
                self._snapshot_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            except OSError as exc:
                self._disable_file_logging(exc)
                return False
        return True

    def write_event(self, event: SimulationEvent) -> bool:
        """写入一条仿真事件。注意：文件失败返回 False，内存记录仍保留。"""
        self.events.append(event)
        if self._event_file is not None:
            try:
                self._event_file.write(json.dumps(self._serialize_record(asdict(event)), ensure_ascii=False) + "\n")
            except OSError as exc:
                self._disable_file_logging(exc)
                return False
        return True

    def flush(self) -> None:
        """刷新记录缓冲。注意：频繁调用会增加 IO 开销。"""
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                try:
                    handle.flush()
                except OSError as exc:
                    self._disable_file_logging(exc)
                    break

    def close(self) -> None:
        """释放 _DataLogger 持有的资源。注意：关闭后不应继续调用运行接口。"""
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                handle.close()
        self._snapshot_file = None
        self._event_file = None
        self.opened = False

    def _disable_file_logging(self, exc: OSError) -> None:
        """停用当前运行的文件落盘。注意：调用方负责把错误转为 WARN 事件。"""
        self.last_error_message = str(exc)
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass
        self._snapshot_file = None
        self._event_file = None
        self.opened = False
        self._file_logging_disabled = True

    @staticmethod
    def _make_run_dir(run_id: str) -> Path:
        """生成不冲突的运行日志目录。注意：同一秒多次启动会自动加序号。"""
        base = Path("logs") / run_id
        if not base.exists():
            return base
        index = 1
        while True:
            candidate = Path("logs") / f"{run_id}-{index}"
            if not candidate.exists():
                return candidate
            index += 1

    @classmethod
    def _serialize_record(cls, record: dict[str, Any], *, omit_keys: set[str] | None = None) -> dict[str, Any]:
        """按日志精度规则序列化记录。注意：只改变落盘值，不改内存快照。"""
        ignored = omit_keys or set()
        return {key: cls._round_log_value(key, value) for key, value in record.items() if key not in ignored}

    @classmethod
    def _round_log_value(cls, key: str, value: Any) -> Any:
        """按字段语义四舍五入日志值。注意：嵌套列表和字典递归处理。"""
        if isinstance(value, dict):
            return cls._serialize_record(value)
        if isinstance(value, list):
            return [cls._round_log_value(key, item) for item in value]
        if not isinstance(value, float) or not math.isfinite(value):
            return value
        decimals = cls._decimals_for_key(key)
        if decimals is None:
            return value
        quant = Decimal("1").scaleb(-decimals)
        return float(Decimal(str(value)).quantize(quant, rounding=ROUND_HALF_UP))

    @classmethod
    def _decimals_for_key(cls, key: str) -> int | None:
        """返回日志字段小数位规则。注意：未知物理量保持原始精度。"""
        if key in cls._TIME_KEYS:
            return 3
        if key in cls._LOAD_FACTOR_KEYS:
            return 4
        if key.endswith(cls._ACCELERATION_SUFFIXES):
            return 3
        if key.endswith(cls._ANGLE_SUFFIXES):
            return 2
        if key.endswith(cls._SPEED_SUFFIXES) or key.endswith(cls._POSITION_SUFFIXES):
            return 2
        return None
