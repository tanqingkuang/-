"""保持模式任务编排实现。注意：下发 HOLD 阶段 + 可运行时切换的队形索引。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormStageE
from src.algorithm.units.process.formation_task.base import FormationTaskBase, FormationTaskInitS, FormationTaskInputS, FormationTaskOutputS


@dataclass
class HoldTaskInitS(FormationTaskInitS):
    """Hold 任务初始化参数。注意：initialPattern 为初始队形索引（formPos 行号）。"""

    initialPattern: int = 0  # 初始队形索引


class Hold(FormationTaskBase):
    """保持任务：恒定输出 HOLD 阶段 + 当前目标队形索引。注意：目标索引可由外部(界面)运行时热切换。"""

    def __init__(self) -> None:
        """构造 Hold 实例。注意：未调用 init 时目标队形默认索引 0。"""
        self._initial_pattern = 0
        self._pattern_index = 0

    def init(self, cfg: FormationTaskInitS | None) -> None:
        """按配置初始化 Hold。注意：cfg 为 None 时目标队形回退到索引 0。"""
        self._initial_pattern = int(getattr(cfg, "initialPattern", 0)) if cfg is not None else 0
        self._pattern_index = self._initial_pattern

    def set_pattern_index(self, index: int) -> None:
        """运行时切换目标队形索引。注意：由控制器 switch_formation 触达，下一拍生效。"""
        self._pattern_index = int(index)

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        """推进 Hold 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        del u  # 保持模式不依赖任何输入
        if y.cmd is None:
            raise ValueError("Hold output port must be bound")
        # 固定下发保持阶段 + 当前目标队形索引，step 计数保持不变
        y.cmd.stage = FormStageE.HOLD
        y.cmd.pattern = self._pattern_index

    def reset(self) -> None:
        """复位 Hold 的动态状态。注意：目标队形回到初始索引。"""
        self._pattern_index = self._initial_pattern
