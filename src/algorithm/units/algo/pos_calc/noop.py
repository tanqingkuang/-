"""位置解算停控策略。注意：输出当前位置和零速度，并完整发布停控命令。"""

from __future__ import annotations

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    MotionProfS,
    PosCalcStatusS,
    PosCalcStrategyE,
    PosTrackCommandE,
    PosTrackCommandS,
    copy_motion,
    copy_position,
    zero_velocity,
)
from src.algorithm.units.algo.pos_calc.base import (
    PosCalcBase,
    PosCalcInitS,
)


class NoopPosCalc(PosCalcBase):
    """NONE阶段位置解算策略。注意：使用内部快照集中提交停控结果。"""

    def __init__(self) -> None:
        """建立停控策略内部快照。"""
        self._cxt: FormContextS | None = None
        self._self_state = MotionProfS()
        self._self_cmd = MotionProfS()
        self._status = PosCalcStatusS()
        self._track_command = PosTrackCommandS()

    def bind(self, cxt: FormContextS) -> None:
        """绑定黑板。注意：运行时只通过内部快照读取和提交。"""
        self._cxt = cxt

    def init(self, cfg: PosCalcInitS) -> None:
        """初始化停控策略。注意：无静态配置和动态资源。"""
        del cfg

    def step(self) -> None:
        """输出当前位置和零速度。注意：按读取、计算、提交顺序更新黑板。"""
        self._read_context()
        self._calculate(self._self_state, self._self_cmd)
        self._write_context()

    def reset(self) -> None:
        """复位停控策略。注意：无跨帧算法状态。"""
        return None

    def _read_context(self) -> None:
        """从黑板生成本拍输入快照。"""
        if self._cxt is None:
            raise ValueError("NoopPosCalc 尚未绑定黑板")
        # 输入使用独立对象，避免停控计算意外修改本机实测状态。
        copy_motion(self._cxt.selfState, self._self_state)

    @staticmethod
    def _calculate(self_state: MotionProfS, self_cmd: MotionProfS) -> None:
        """生成停控目标。注意：本方法不访问黑板。"""
        copy_position(self_state.pos, self_cmd.pos)
        zero_velocity(self_cmd.v)

    def _write_context(self) -> None:
        """把完整停控结果原地提交到黑板。"""
        assert self._cxt is not None
        # NOOP不拥有集结专有诊断，因此只更新自身命令和活动策略。
        self._status.active_strategy = PosCalcStrategyE.NOOP
        self._track_command.mode = PosTrackCommandE.NOOP
        copy_motion(self._self_cmd, self._cxt.selfCmd)
        # 黑板对象必须原地更新，不能替换其他流程已经持有的引用。
        self._cxt.posCalcStatus.active_strategy = self._status.active_strategy
        self._cxt.posTrackCommand.mode = self._track_command.mode
