"""位置解算停控策略。注意：输出当前位置和零速度，并完整发布停控命令。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    MotionProfS,
    PosCalcStatusS,
    PosCalcStrategyE,
    PosTrackCommandE,
    PosTrackCommandS,
    copy_position,
    zero_velocity,
)
from src.algorithm.units.algo.pos_calc.base import (
    PosCalcBase,
    PosCalcInitS,
)


@dataclass
class _NoopInputS:
    """停控策略私有输入端口。"""

    selfState: MotionProfS = field(default_factory=MotionProfS)


@dataclass
class _NoopOutputS:
    """停控策略私有输出端口。"""

    selfCmd: MotionProfS = field(default_factory=MotionProfS)
    status: PosCalcStatusS = field(default_factory=PosCalcStatusS)
    posTrackCommand: PosTrackCommandS = field(default_factory=PosTrackCommandS)


class NoopPosCalc(PosCalcBase):
    """NONE阶段位置解算策略。注意：使用私有端口集中提交停控结果。"""

    def __init__(self) -> None:
        """建立停控策略私有端口。"""
        self._u = _NoopInputS()
        self._y = _NoopOutputS()
        self._bound = False

    def bind(self, cxt: FormContextS) -> None:
        """绑定专属输入输出端口。注意：后续 step 不再访问完整黑板。"""
        self._u = _NoopInputS(selfState=cxt.selfState)
        self._y = _NoopOutputS(
            selfCmd=cxt.selfCmd,
            status=cxt.posCalcStatus,
            posTrackCommand=cxt.posTrackCommand,
        )
        self._bound = True

    def init(self, cfg: PosCalcInitS) -> None:
        """初始化停控策略。注意：无静态配置和动态资源。"""
        del cfg

    def step(self) -> None:
        """输出当前位置和零速度。注意：直接写入 bind 阶段绑定的输出对象。"""
        if not self._bound:
            raise ValueError("NoopPosCalc 尚未绑定端口")
        self._calculate(self._u.selfState, self._y.selfCmd)
        self._y.status.active_strategy = PosCalcStrategyE.NOOP
        self._y.posTrackCommand.mode = PosTrackCommandE.NOOP

    def reset(self) -> None:
        """复位停控策略。注意：无跨帧算法状态。"""
        return None

    @staticmethod
    def _calculate(self_state: MotionProfS, self_cmd: MotionProfS) -> None:
        """生成停控目标。注意：本方法不访问黑板。"""
        copy_position(self_state.pos, self_cmd.pos)
        zero_velocity(self_cmd.v)
