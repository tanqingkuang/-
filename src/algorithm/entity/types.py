"""实体边界类型。注意：用于控制器和算法实体之间传递数据。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import IntEnum
from types import MappingProxyType

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormCommInitS,
    FormationAnalysisS,
    FormSelfInitS,
    MotionProfS,
    PosTrackDiagS,
    PosCalcStrategyE,
    PosTrackStrategyE,
    RallyPhaseE,
    RemoteCmdS,
    FormStageE,
    WayPointInputS,
)
from src.algorithm.units.process.tra_plan.base import TraPlanStrategyE
from src.common.envelope import MessageEnvelope
DEFAULT_CONTROL_PERIOD_S = 0.05


@dataclass
class VelCmdLimitS:
    """前向/垂向速度指令限幅(串级 P+PI 外环输出)。注意：非对称，默认 ±inf 表示不限；侧向不限速。"""

    forwardMin: float = float("-inf")  # 前向速度指令下限(前向恒正时设 >0)
    forwardMax: float = float("inf")  # 前向速度指令上限
    verticalMin: float = float("-inf")  # 垂向速度指令下限(下降速度上限取负)
    verticalMax: float = float("inf")  # 垂向速度指令上限(爬升速度上限)


class EntityProfileE(IntEnum):
    """实体身份枚举。注意：外部只选择身份，不拼装流程策略。"""

    RALLY_LEADER = 1  # 集结长机：集结位置解算、任务航线和速度控制
    RALLY_FOLLOWER = 2  # 集结僚机：集结/槽位位置解算和速度/位置控制


EntityStateT = tuple[FormStageE, RallyPhaseE]


@dataclass(frozen=True)
class EntityStrategiesS:
    """单个任务状态下三个可切换流程的完整策略组合。"""

    tra_plan: TraPlanStrategyE  # 轨迹规划产品
    pos_calc: PosCalcStrategyE  # 位置解算产品
    pos_track: PosTrackStrategyE  # 位置跟踪产品


@dataclass(frozen=True)
class EntityRouteChangeS:
    """从指定状态开始生效的策略变化点。注意：后续状态沿用到下一变化点。"""

    state: EntityStateT  # 当前变化点对应的合法状态
    strategies: EntityStrategiesS  # 从当前状态开始使用的完整策略组合


@dataclass(frozen=True)
class EntityProfileS:
    """实体不可变身份证。注意：同一身份的实例共享配置，不共享运行状态。"""

    identity: EntityProfileE  # 工厂选择键
    state_sequence: tuple[EntityStateT, ...]  # Entity 定义的合法状态顺序
    route_changes: tuple[EntityRouteChangeS, ...]  # 用户只填写策略变化点
    route_table: Mapping[EntityStateT, EntityStrategiesS] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """校验变化点并展开完整不可变路由表。"""
        # state_sequence 是路由表的唯一合法键空间，先建立索引才能校验变化点顺序。
        # 使用显式顺序而非枚举数值排序，是因为阶段和子阶段并非笛卡尔积。
        state_sequence = self.state_sequence
        if not state_sequence:
            raise ValueError("EntityProfile.state_sequence 不得为空")
        state_indexes: dict[EntityStateT, int] = {}
        for index, state in enumerate(state_sequence):
            _validate_entity_state(state, "state_sequence")
            if state in state_indexes:
                raise ValueError(f"EntityProfile.state_sequence 状态重复: {state!r}")
            state_indexes[state] = index
        if not self.route_changes:
            raise ValueError("EntityProfile.route_changes 不得为空")
        if self.route_changes[0].state != state_sequence[0]:
            raise ValueError("EntityProfile 第一个合法状态必须配置策略变化点")

        # 变化点只描述策略发生变化的位置，未变化状态由前项继承。
        # 严格递增保证展开结果与作者阅读配置时的先后认知一致。
        # 连续写入相同策略没有信息量，通常意味着遗漏了真正的变化状态。
        changes: dict[EntityStateT, EntityStrategiesS] = {}
        previous_index = -1
        previous_strategies: EntityStrategiesS | None = None
        for change in self.route_changes:
            _validate_entity_state(change.state, "route_changes")
            _validate_entity_strategies(change.strategies)
            if change.state not in state_indexes:
                raise ValueError(f"EntityProfile 变化点不是合法状态: {change.state!r}")
            current_index = state_indexes[change.state]
            if current_index <= previous_index:
                raise ValueError("EntityProfile.route_changes 必须按合法状态顺序填写且不得重复")
            if change.strategies == previous_strategies:
                raise ValueError(f"EntityProfile 存在未改变策略的冗余变化点: {change.state!r}")
            changes[change.state] = change.strategies
            previous_index = current_index
            previous_strategies = change.strategies

        # 单次线性扫描把稀疏变化点展开为完整表，运行期即可只做严格字典查询。
        # 展开后使用 MappingProxyType 冻结，避免一个实体修改所有实例共享的 Profile。
        # 第一个状态已强制提供变化点，因此 active_strategies 在首轮后必定有效。
        routes: dict[EntityStateT, EntityStrategiesS] = {}
        active_strategies: EntityStrategiesS | None = None
        for state in state_sequence:
            if state in changes:
                active_strategies = changes[state]
            if active_strategies is None:
                raise ValueError(f"EntityProfile 状态缺少可继承策略: {state!r}")
            routes[state] = active_strategies
        object.__setattr__(self, "route_table", MappingProxyType(routes))

    def require_strategies(self, stage: FormStageE, step: int) -> EntityStrategiesS:
        """严格查询当前状态策略。注意：表外状态不得回退到默认策略。"""
        # 边界先拒绝普通整数 stage，避免 IntEnum 的相等规则掩盖协议类型错误。
        # step 接口允许传输层使用整数，但必须能还原为当前仍受支持的 RallyPhaseE。
        # 状态组合合法性最终由展开表判定，不能仅凭两个枚举分别合法就接受。
        # 查询失败显式报错，使状态机与 Profile 漂移在首拍暴露而非静默选错产品。
        if not isinstance(stage, FormStageE):
            raise ValueError(f"EntityProfile stage 非法: {stage!r}")
        if not isinstance(step, int) or isinstance(step, bool):
            raise ValueError(f"EntityProfile step 非法: {step!r}")
        try:
            phase = RallyPhaseE(step)
        except ValueError as exc:
            raise ValueError(f"EntityProfile step 非法: {step!r}") from exc
        state = (stage, phase)
        try:
            return self.route_table[state]
        except KeyError as exc:
            raise ValueError(f"EntityProfile 未配置任务状态: {state!r}") from exc


def _validate_entity_state(state: object, field_name: str) -> None:
    """校验状态键必须由两个现有任务枚举组成。"""
    if (
        not isinstance(state, tuple)
        or len(state) != 2
        or not isinstance(state[0], FormStageE)
        or not isinstance(state[1], RallyPhaseE)
    ):
        raise ValueError(f"EntityProfile.{field_name} 必须使用 (FormStageE, RallyPhaseE)")


def _validate_entity_strategies(strategies: EntityStrategiesS) -> None:
    """校验变化点包含三个模块的完整策略枚举。"""
    if not isinstance(strategies, EntityStrategiesS):
        raise ValueError("EntityProfile.strategies 必须是 EntityStrategiesS")
    if not isinstance(strategies.tra_plan, TraPlanStrategyE):
        raise ValueError("EntityProfile.tra_plan 必须是 TraPlanStrategyE")
    if not isinstance(strategies.pos_calc, PosCalcStrategyE):
        raise ValueError("EntityProfile.pos_calc 必须是 PosCalcStrategyE")
    if not isinstance(strategies.pos_track, PosTrackStrategyE):
        raise ValueError("EntityProfile.pos_track 必须是 PosTrackStrategyE")


@dataclass
class EntityInitS:
    """实体一次性初始化配置。注意：集结实体共用 route 前两点确定集结中心和航向。"""

    selfInit: FormSelfInitS = field(default_factory=FormSelfInitS)  # 本机标识
    commInit: FormCommInitS = field(default_factory=FormCommInitS)  # 通信拓扑与队形配置
    route: list[WayPointInputS] = field(default_factory=list)  # 任务航线；集结实体同时读取前两点计算集结几何
    control_period_s: float = DEFAULT_CONTROL_PERIOD_S  # 控制算法处理周期，单位 s
    velCmdLimit: VelCmdLimitS = field(default_factory=VelCmdLimitS)  # 前向/垂向速度指令限幅
    rally_cfg: object | None = None  # RallyTaskInitS；长机使用完整参数，僚机只取 convergenceRadius_m
    rally_approach_speed_mps: float = 20.0  # 僚机飞向 M_i 的速度
    rally_leader_id: str = ""  # 僚机回报消息的发送目标（来自节点配置 leader_id）
    rally_layer_altitude_m: float | None = None  # 待命/JOINING/CATCHUP 分层目标高度；None 表示沿用集结槽位高度
    rally_enabled: bool = True  # 当前实例是否执行集结任务；直接 HOLD 时关闭集结专用槽位配置


@dataclass(frozen=True)
class EntityManagerInitS:
    """流程 Manager 内部初始化参数。注意：由 Entity 根据自身 Profile 生成。"""

    entity: EntityInitS  # 每架飞机不同的运行初始化参数
    profile: EntityProfileS  # 三个可切换 Manager 共用的完整策略表


@dataclass
class EntityRuntimeS:
    """实体流程共享运行环境。注意：各流程自行绑定所需对象，Entity 不维护具体端口。"""

    context: FormContextS = field(default_factory=FormContextS)  # 算法共享黑板
    remote: RemoteCmdS = field(default_factory=RemoteCmdS)  # 外部任务指令
    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本拍收件箱
    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本拍发件箱
    posTrackDiag: PosTrackDiagS = field(default_factory=PosTrackDiagS)  # 控制诊断输出


@dataclass
class EntityInputS:
    """实体每帧输入。注意：各字段可为空，缺省时沿用上一帧状态。"""

    selfState: MotionProfS | None = None  # 本机最新运动状态反馈
    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧收到的消息
    remote: RemoteCmdS | None = None  # 外部遥控指令
    now_s: float = 0.0  # 当前仿真时间戳（秒）；由仿真框架每帧注入，用于僚机报文超时检测


@dataclass
class EntityOutputS:
    """实体每帧输出。注意：控制器只从该边界读取算法结果。"""

    selfAccCmd: AccInEarthS | None = None  # 本机加速度指令
    selfCmd: MotionProfS | None = None  # 本机位置/速度指令快照
    controlDiag: PosTrackDiagS | None = None  # 位置跟踪诊断快照
    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧待发送的消息
    formationAnalysis: FormationAnalysisS | None = None  # 仅集结完成首帧非 None；仿真层须另行锁存
