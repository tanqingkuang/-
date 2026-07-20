"""编队实体基础接口。注意：具体实体只实现初始化与角色输出，公共边界和生命周期由基类管理。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from src.algorithm.context.context import reset_context
from src.algorithm.context.leaf_types import (
    FormStageE,
    PosTrackDiagS,
    RemoteCmdS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.types import (
    EntityInitS,
    EntityInputS,
    EntityOutputS,
    EntityProfileS,
    EntityRuntimeS,
)
from src.algorithm.units.algo.pos_calc import PosCalcManager
from src.algorithm.units.algo.pos_track import PosTrackManager
from src.algorithm.units.process.formation_task.rally import Rally
from src.algorithm.units.process.inbound import FormationInbound
from src.algorithm.units.process.outbound import FormationOutbound
from src.algorithm.units.process.tra_plan import TraPlanManager


# 流程类型是 Entity 架构契约，不属于外部 Profile；Profile 只决定各 Manager 的策略产品。
# 元组顺序是固定流程链的唯一来源；这里登记流程容器类而不是算法产品。
# Manager 在 init 中依据 Profile 创建产品，运行期不得替换流程容器。
# 固定表使 Entity.step 不需要了解长机、僚机、集结或保持等业务身份。
# 新增算法策略时应修改对应 Manager 和 Profile，不应扩展这条主链。
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
    """已绑定运行环境的实体流程步骤。注意：process 的具体类型在初始化后不再变化。"""

    process: Any  # 各流程自行维护端口，Entity 只依赖统一无参 step 调用约定

    def step(self) -> None:
        """推进当前流程一步。注意：流程自行从已绑定运行环境读取和写回数据。"""
        self.process.step()


class EntityBase:
    """编队实体模板基类。注意：固定流程由初始化表装配，运行期统一顺序执行。"""

    PROFILE: EntityProfileS | None = None
    MISSING_REMOTE_STAGE: FormStageE | None = None

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
        # Entity边界只负责外部数据与共享运行环境之间的交换。
        # 边界输入先写入共享黑板，后续流程通过预绑定引用读取同一拍数据。
        self._prepare_input(u)
        # 每个流程已经在初始化期绑定运行环境，这里不传递任何业务端口。
        # 流程对象和端口均在 init 后锁定，运行期只做固定顺序调用。
        for process_step in self._process_steps:
            process_step.step()
        # 输出只能在完整流程链结束后生成，防止调用方观察到半拍状态。
        # 所有流程完成后再形成实体边界快照，避免输出到半拍结果。
        self._finish_output(u, y)

    def reset(self) -> None:
        """复位实体动态状态。注意：保留初始化期装配的流程和产品实例。"""
        reset_context(self.cxt)
        self._remote.stage = RemoteCmdS().stage
        self._reset_processes()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._inbox.clear()
        self._outbox.clear()

    def close(self) -> None:
        """释放实体资源。注意：当前流程均无外部资源，保留统一生命周期接口。"""
        return None

    def _initialize_process_chain(self, init_configs: dict[str, Any]) -> None:
        """初始化实例黑板和固定流程链。注意：每次调用都会建立全新运行状态。"""
        # 重新 init 必须更换整套运行环境，随后由每个流程自行绑定所需对象。
        self._runtime = EntityRuntimeS()
        # 以下属性只保留边界兼容名称，不再承担各流程端口装配职责。
        self.cxt = self._runtime.context
        self._remote = self._runtime.remote
        self._inbox = self._runtime.inbox
        self._outbox = self._runtime.outbox
        self._pos_track_diag = self._runtime.posTrackDiag
        self._create_processes(init_configs)

    def _create_processes(
        self,
        init_configs: dict[str, Any],
    ) -> None:
        """遍历固定流程类型创建并初始化实例。注意：具体策略仍由 EntityInitS 配置。"""
        expected_names = tuple(field_name for field_name, _ in _PROCESS_TYPES)
        if set(init_configs) != set(expected_names):
            missing = ", ".join(sorted(set(expected_names) - set(init_configs)))
            extra = ", ".join(sorted(set(init_configs) - set(expected_names)))
            raise ValueError(f"流程初始化参数不完整: missing=[{missing}], extra=[{extra}]")
        self._process_slots: list[tuple[str, str]] = []
        # bind先于init，使Manager创建子策略时可以把同一黑板继续下传。
        # 每个流程实例只创建和初始化一次，Manager 内部有状态策略不得在阶段切换时重建。
        # init_configs 只携带静态装配数据，不能保存每拍变化的运动状态。
        # runtime 由 Entity 独占，禁止在不同飞机实例之间共享。
        # 流程自行建立专属端口，Entity 不再为算法字段变化维护公共端口。
        # 创建失败应直接终止初始化，避免留下只装配了一半的可运行实例。
        for field_name, process_class in _PROCESS_TYPES:
            process = process_class()
            for method_name in ("bind", "init", "step", "reset"):
                if not callable(getattr(process, method_name, None)):
                    raise ValueError(f"processes.{field_name} 缺少 {method_name} 接口")
            process.bind(self._runtime)
            process.init(init_configs[field_name])
            # formation_task沿用历史_task属性名，其他流程按槽位名暴露诊断引用。
            # 属性仅供调试和既有控制入口定位流程，不参与运行期选择。
            attr_name = "_task" if field_name == "formation_task" else f"_{field_name}"
            setattr(self, attr_name, process)
            self._process_slots.append((field_name, attr_name))
        self._process_steps = [
            EntityProcessStepS(getattr(self, attr_name)) for _, attr_name in self._process_slots
        ]

    def _reset_processes(self) -> None:
        """按装配表复位全部流程实例。注意：不得重建流程对象或改变执行顺序。"""
        for _, attr_name in self._process_slots:
            getattr(self, attr_name).reset()

    def _prepare_input(self, u: EntityInputS) -> None:
        """把实体边界输入写入共享运行时。注意：空字段沿用上一拍状态。"""
        if u.remote is not None:
            remote_stage = u.remote.stage
            # 外部远控必须属于当前 Profile 的可执行阶段；预留枚举不得污染本拍其他输入。
            if not isinstance(remote_stage, FormStageE) or not any(
                state_stage == remote_stage for state_stage, _ in self.profile.state_sequence
            ):
                raise ValueError(f"{type(self).__name__} 不支持的远控阶段: {remote_stage!r}")
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = remote_stage
        elif self.MISSING_REMOTE_STAGE is not None:
            # 僚机以空 remote 释放本地待命覆盖；其他身份默认沿用上一拍远控值。
            self._remote.stage = self.MISSING_REMOTE_STAGE
        self.cxt.clock.now_s = u.now_s
        self._inbox.clear()
        self._inbox.extend(u.inbox)

    def _finish_output(self, u: EntityInputS, y: EntityOutputS) -> None:
        """把共享运行时结果回填实体边界。注意：所有身份使用同一输出协议。"""
        del u
        if y.selfAccCmd is None:
            y.selfAccCmd = self.cxt.selfAccCmd
        else:
            y.selfAccCmd.accEast = self.cxt.selfAccCmd.accEast
            y.selfAccCmd.accNorth = self.cxt.selfAccCmd.accNorth
            y.selfAccCmd.accUp = self.cxt.selfAccCmd.accUp
        if y.selfCmd is None:
            y.selfCmd = self.cxt.selfCmd
        else:
            copy_motion(self.cxt.selfCmd, y.selfCmd)
        if y.controlDiag is None:
            y.controlDiag = self._pos_track_diag
        else:
            copy_pos_track_diag(self._pos_track_diag, y.controlDiag)
        y.outbox.clear()
        y.outbox.extend(self._outbox)
        y.rallyCompleted = self._task.rally_completed
