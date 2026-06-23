"""说明该模块的职责。注意：模块接口变更需同步相关调用方。"""


class DisturbanceManager:
    """管理随机和动态扰动。注意：当前扰动会转发到模型或通信模块。"""

    def tick(self, dt: float) -> None:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
        raise NotImplementedError

