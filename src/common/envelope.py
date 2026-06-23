"""通用通信消息信封。注意：载荷保持为算法无关对象。"""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MessageEnvelope:
    """与算法载荷解耦的传输层消息包装。注意：source/target/topic 是路由关键字段。"""

    topic: str
    source: str
    target: str | list[str]
    timestamp: float
    payload: Any

