"""目标位置计算基础接口。注意：具体策略自行维护与黑板字段直接绑定的专属端口。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    PosCalcStatusS,
    PosCalcStrategyE as PosCalcStrategyE,
)

if TYPE_CHECKING:
    from src.algorithm.context.context import FormContextS


@dataclass
class PosCalcInitS:
    """目标位置计算初始化基类。注意：具体算法可继承后追加配置字段。"""

    pass


class PosCalcBase:
    """目标位置计算算法基类。注意：子类只负责生成目标运动剖面，不直接输出加速度。"""

    def bind(self, cxt: FormContextS) -> None:
        """绑定算法黑板。注意：具体策略只保存所需字段的输入输出引用。"""
        raise NotImplementedError

    def init(self, cfg: PosCalcInitS) -> None:
        """按配置初始化 PosCalcBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self) -> None:
        """推进一个处理周期。注意：策略从绑定黑板读取并原地提交结果。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 PosCalcBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError


def reset_pos_calc_status(status: PosCalcStatusS, strategy: PosCalcStrategyE) -> None:
    """重置并标记公共位置解算状态。注意：具体策略随后写入自身扩展字段。"""
    status.active_strategy = strategy
    status.rally_state = ""
    status.planned_path_length_m = -1.0
    status.remaining_path_length_m = -1.0
    status.remaining_loops = 0
    status.reached_slot_once = False
    status.join_exited = False


def copy_pos_calc_status(src: PosCalcStatusS, dst: PosCalcStatusS) -> None:
    """原地复制位置解算状态。注意：不得替换黑板持有的状态对象。"""
    dst.active_strategy = src.active_strategy
    dst.rally_state = src.rally_state
    dst.planned_path_length_m = src.planned_path_length_m
    dst.remaining_path_length_m = src.remaining_path_length_m
    dst.remaining_loops = src.remaining_loops
    dst.reached_slot_once = src.reached_slot_once
    dst.join_exited = src.join_exited
