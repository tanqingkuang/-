"""保持模式任务编排实现。注意：当前固定输出三角队形保持。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormPatE, FormStageE
from src.algorithm.units.process.formation_task.base import FormationTaskBase, FormationTaskInitS, FormationTaskInputS, FormationTaskOutputS


class Hold(FormationTaskBase):
    """保持任务：忽略输入，恒定输出 HOLD 阶段 + 三角队形。注意：当前为固定策略，不响应遥控阶段切换。"""

    def init(self, cfg: FormationTaskInitS) -> None:
        """按配置初始化 Hold。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        """推进 Hold 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        del u  # 保持模式不依赖任何输入
        if y.cmd is None:
            raise ValueError("Hold output port must be bound")
        # 固定下发：保持阶段 + 三角队形，step 计数保持不变
        y.cmd.stage = FormStageE.HOLD
        y.cmd.pattern = FormPatE.TRIANGLE

    def reset(self) -> None:
        """复位 Hold 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
