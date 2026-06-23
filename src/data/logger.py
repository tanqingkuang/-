"""说明该模块的职责。注意：模块接口变更需同步相关调用方。"""


class SimulationLogger:
    """持久化关键仿真变量。注意：当前实现可按需要替换为文件记录器。"""

    def write(self, record: dict[str, object]) -> None:
        """写入一条仿真记录。注意：具体落盘格式由 logger 实现决定。"""
        raise NotImplementedError

