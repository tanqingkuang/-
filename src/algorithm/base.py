"""算法基础类型和消息结构声明接口。注意：通信契约变更需同步算法实现。"""


class AlgorithmBase:
    """协同算法和节点算法插件基类。注意：子类需要保持统一生命周期接口。"""

    def declare_message_schema(self) -> dict[str, object]:
        """声明算法需要收发的消息结构。注意：返回内容是通信层约定，字段名变更需同步上下游。"""
        raise NotImplementedError

