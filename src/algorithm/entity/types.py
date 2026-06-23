"""实体边界类型。注意：用于控制器和算法实体之间传递数据。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import AccInEarthS, FormCommInitS, FormSelfInitS, MotionProfS, RemoteCmdS, RouteS
from src.common.envelope import MessageEnvelope


DEFAULT_CONTROL_PERIOD_S = 0.05


@dataclass
class EntityInitS:
    """实体一次性初始化配置。注意：route 仅长机使用，僚机可为空。"""

    selfInit: FormSelfInitS = field(default_factory=FormSelfInitS)  # 本机标识
    commInit: FormCommInitS = field(default_factory=FormCommInitS)  # 通信拓扑与队形配置
    route: RouteS | None = None  # 预置航线，僚机无需航线时为 None
    control_period_s: float = DEFAULT_CONTROL_PERIOD_S  # 控制算法处理周期，单位 s


@dataclass
class EntityInputS:
    """实体每帧输入。注意：各字段可为空，缺省时沿用上一帧状态。"""

    selfState: MotionProfS | None = None  # 本机最新运动状态反馈
    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧收到的消息
    remote: RemoteCmdS | None = None  # 外部遥控指令


@dataclass
class EntityOutputS:
    """实体每帧输出。注意：selfAccCmd 给控制器，outbox 待发送。"""

    selfAccCmd: AccInEarthS | None = None  # 本机加速度指令
    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧待发送的消息
