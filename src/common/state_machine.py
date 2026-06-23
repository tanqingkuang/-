"""共享状态机工具。注意：当前为后续阶段机预留。"""


class StateMachine:
    """通用阶段和任务状态机占位类。注意：后续扩展时保持 step 接口稳定。"""

    def step(self) -> None:
        """推进 StateMachine 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

