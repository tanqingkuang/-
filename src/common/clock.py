"""仿真时钟工具。注意：只记录仿真时间，不依赖墙钟。"""


class SimulationClock:
    """跟踪仿真时间。注意：tick 调用方负责传入正确步长。"""

    def __init__(self) -> None:
        """初始化 SimulationClock 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.time = 0.0

    def tick(self, dt: float) -> float:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
        self.time += dt
        return self.time

