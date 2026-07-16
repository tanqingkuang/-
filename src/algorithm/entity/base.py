"""编队实体基础接口。注意：具体实体实现初始化、边界钩子、复位和关闭。"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import PosTrackDiagS, RemoteCmdS
from src.algorithm.entity.types import (
    EntityInitS,
    EntityInputS,
    EntityOutputS,
    EntityProcessTableS,
    EntityProfileS,
)
from src.algorithm.units.algo.pos_calc import PosCalcInputS, PosCalcManager, PosCalcOutputS
from src.algorithm.units.algo.pos_track import PosTrackInputS, PosTrackManager, PosTrackOutputS
from src.algorithm.units.process.formation_task.rally import (
    Rally,
    RallyTaskInputS,
    RallyTaskOutputS,
)
from src.algorithm.units.process.inbound import (
    FormationInbound,
    FormationInboundOutputS,
    InboundInputS,
)
from src.algorithm.units.process.outbound import (
    FormationOutbound,
    FormationOutboundInputS,
    OutboundOutputS,
)
from src.algorithm.units.process.tra_plan import (
    TraPlanInputS,
    TraPlanManager,
    TraPlanOutputS,
)
from src.common.envelope import MessageEnvelope


# 流程类型是 Entity 架构契约，不属于外部 Profile；Profile 只决定各 Manager 的策略产品。
# 元组顺序必须与 EntityProcessTableS 保持一致，基类据此生成唯一执行链。
_PROCESS_TYPES: tuple[tuple[str, type[object]], ...] = (
    ("inbound", FormationInbound),
    ("formation_task", Rally),
    ("tra_plan", TraPlanManager),
    ("pos_calc", PosCalcManager),
    ("pos_track", PosTrackManager),
    ("outbound", FormationOutbound),
)


@dataclass(frozen=True)
class EntityProcessStepS:
    """已绑定端口的实体流程步骤。注意：process 的具体类型在初始化后不再变化。"""

    process: Any  # 各流程具有不同端口泛型，Entity 只依赖统一 step 调用约定
    input_port: Any  # 初始化时绑定的输入端口对象
    output_port: Any  # 初始化时绑定的输出端口对象

    def step(self) -> None:
        """推进当前流程一步。注意：端口引用在整个实体生命周期内保持不变。"""
        self.process.step(self.input_port, self.output_port)


class EntityBase:
    """编队实体模板基类。注意：固定流程由初始化表装配，运行期统一顺序执行。"""

    PROFILE: EntityProfileS | None = None

    @property
    def profile(self) -> EntityProfileS:
        """返回实例身份证。注意：Profile 由实体类固定，外部不可替换。"""
        if self.PROFILE is None:
            raise ValueError(f"{type(self).__name__} 未配置实体 Profile")
        return self.PROFILE

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 EntityBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """按初始化顺序推进全部流程。注意：使用装配表的子类不得重新实现处理链。"""
        # 边界输入先写入共享黑板，后续流程通过预绑定引用读取同一拍数据。
        self._prepare_input(u)
        # 流程对象和端口均在 init 后锁定，运行期只做固定顺序调用。
        for process_step in self._process_steps:
            process_step.step()
        # 所有流程完成后再形成实体边界快照，避免输出到半拍结果。
        self._finish_output(u, y)

    def reset(self) -> None:
        """复位 EntityBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError

    def close(self) -> None:
        """释放 EntityBase 持有的资源。注意：关闭后不应继续调用运行接口。"""
        raise NotImplementedError

    def _initialize_process_chain(self, init_configs: dict[str, Any]) -> None:
        """初始化实例黑板、固定端口和流程链。注意：每次调用都会建立全新运行状态。"""
        # 重新 init 必须更换整套运行容器，不能让新端口继续引用上一轮黑板。
        self.cxt = FormContextS()
        self._remote = RemoteCmdS()
        # 收发箱由实体边界持有，通信流程只绑定列表引用，不接管列表生命周期。
        self._inbox: list[MessageEnvelope] = []
        self._outbox: list[MessageEnvelope] = []
        self._pos_track_diag = PosTrackDiagS()
        # 先建立黑板和端口，再初始化流程产品，确保所有引用在首拍前固定。
        self._create_standard_ports()
        self._create_processes(init_configs)
        self._bind_process_ports(
            inbound=(self._inbound_u, self._inbound_y),
            formation_task=(self._task_u, self._task_y),
            tra_plan=(self._tra_plan_u, self._tra_plan_y),
            pos_calc=(self._pos_calc_u, self._pos_calc_y),
            pos_track=(self._pos_track_u, self._pos_track_y),
            outbound=(self._outbound_u, self._outbound_y),
        )

    def _create_standard_ports(self) -> None:
        """创建固定流程端口并绑定当前实例黑板。注意：具体策略只读取所需字段。"""
        # 入站直接更新完整黑板，使长机广播和僚机回报可以按 topic 原地提交。
        self._inbound_u = InboundInputS(inbox=self._inbox)
        self._inbound_y = FormationInboundOutputS(context=self.cxt)
        # 任务端口同时读取上一拍位置解算状态，并原地发布任务指令和同步计划。
        self._task_u = RallyTaskInputS(
            remote=self._remote,
            cmd=self.cxt.cmd,
            followerStates=self.cxt.followerStates,
            clock=self.cxt.clock,
            posCalcStatus=self.cxt.posCalcStatus,
        )
        self._task_y = RallyTaskOutputS(cmd=self.cxt.cmd, rallyPlan=self.cxt.rallyPlan)
        # 轨迹规划只负责推进航段，当前和下一航段分别供位置解算跟踪及前瞻。
        self._tra_plan_u = TraPlanInputS(
            cmd=self.cxt.cmd,
            wayLine=self.cxt.wayLine,
            selfState=self.cxt.selfState,
        )
        self._tra_plan_y = TraPlanOutputS(
            wayLine=self.cxt.wayLine,
            nextWayLine=self.cxt.nextWayLine,
        )
        # 位置解算使用统一超集端口；长、僚机策略各自忽略不需要的引用。
        self._pos_calc_u = PosCalcInputS(
            selfState=self.cxt.selfState,
            leaderState=self.cxt.leaderState,
            leaderCmd=self.cxt.leaderCmd,
            cmd=self.cxt.cmd,
            wayLine=self.cxt.wayLine,
            nextWayLine=self.cxt.nextWayLine,
            clock=self.cxt.clock,
            rallyPlan=self.cxt.rallyPlan,
        )
        self._pos_calc_y = PosCalcOutputS(
            selfCmd=self.cxt.selfCmd,
            status=self.cxt.posCalcStatus,
            posTrackCommand=self.cxt.posTrackCommand,
        )
        # 位置跟踪只消费 PosCalc 发布的控制语义，不反向感知任务阶段。
        self._pos_track_u = PosTrackInputS(
            command=self.cxt.posTrackCommand,
            selfCmd=self.cxt.selfCmd,
            selfState=self.cxt.selfState,
        )
        self._pos_track_y = PosTrackOutputS(
            accCmd=self.cxt.selfAccCmd,
            diag=self._pos_track_diag,
            effectiveCmd=self.cxt.effectiveCmd,
        )
        # 出站从本拍最终黑板统一组包，避免 Entity 手工搬运协议字段。
        self._outbound_u = FormationOutboundInputS(context=self.cxt)
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

    def _create_processes(
        self,
        init_configs: dict[str, Any],
    ) -> None:
        """遍历固定流程类型创建并初始化实例。注意：具体策略仍由 EntityInitS 配置。"""
        # 配置表字段是流程槽位的规范来源，顺序漂移必须在初始化期立即失败。
        expected_names = tuple(table_field.name for table_field in fields(EntityProcessTableS))
        actual_names = tuple(field_name for field_name, _ in _PROCESS_TYPES)
        if actual_names != expected_names:
            raise ValueError(f"固定流程顺序无效: expected={expected_names!r}, actual={actual_names!r}")
        if set(init_configs) != set(expected_names):
            missing = ", ".join(sorted(set(expected_names) - set(init_configs)))
            extra = ", ".join(sorted(set(init_configs) - set(expected_names)))
            raise ValueError(f"流程初始化参数不完整: missing=[{missing}], extra=[{extra}]")
        self._process_slots: list[tuple[str, str]] = []
        # 每个流程实例只创建和初始化一次，Manager 内部有状态策略不得在阶段切换时重建。
        for field_name, process_class in _PROCESS_TYPES:
            process = process_class()
            for method_name in ("init", "step", "reset"):
                if not callable(getattr(process, method_name, None)):
                    raise ValueError(f"processes.{field_name} 缺少 {method_name} 接口")
            process.init(init_configs[field_name])
            attr_name = "_task" if field_name == "formation_task" else f"_{field_name}"
            setattr(self, attr_name, process)
            self._process_slots.append((field_name, attr_name))

    def _bind_process_ports(self, **ports: tuple[Any, Any]) -> None:
        """按装配表顺序绑定统一执行链。注意：每个流程必须提供一组输入输出端口。"""
        expected = {field_name for field_name, _ in self._process_slots}
        actual = set(ports)
        if actual != expected:
            missing = ", ".join(sorted(expected - actual))
            extra = ", ".join(sorted(actual - expected))
            raise ValueError(f"流程端口与装配表不一致: missing=[{missing}], extra=[{extra}]")
        self._process_steps = [
            EntityProcessStepS(getattr(self, attr_name), *ports[field_name])
            for field_name, attr_name in self._process_slots
        ]

    def _reset_processes(self) -> None:
        """按装配表复位全部流程实例。注意：不得重建流程对象或改变执行顺序。"""
        for _, attr_name in self._process_slots:
            getattr(self, attr_name).reset()

    def _prepare_input(self, u: EntityInputS) -> None:
        """把实体边界输入写入黑板。注意：由具体角色实现，不得推进业务流程。"""
        raise NotImplementedError

    def _finish_output(self, u: EntityInputS, y: EntityOutputS) -> None:
        """把黑板结果回填实体边界。注意：由具体角色实现，不得重复推进业务流程。"""
        raise NotImplementedError
