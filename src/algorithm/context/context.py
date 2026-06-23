"""编队算法黑板。注意：实体内多单元通过该对象共享状态。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormSnapshotS,
    MotionProfS,
    WayLineS,
    copy_motion,
    copy_snapshot,
    copy_wayline,
)


@dataclass
class FormContextS:
    """单个实体持有的跨帧编队状态。注意：reset 时需要清理运行期缓存。"""

    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)
    state: list[FormSnapshotS] = field(default_factory=list)
    wayLine: WayLineS = field(default_factory=WayLineS)
    leaderState: MotionProfS = field(default_factory=MotionProfS)
    selfCmd: MotionProfS = field(default_factory=MotionProfS)
    selfState: MotionProfS = field(default_factory=MotionProfS)
    selfAccCmd: AccInEarthS = field(default_factory=AccInEarthS)


def reset_context(dst: FormContextS) -> None:
    """重置全局算法上下文，清空本轮仿真遗留数据。注意：只应在重新初始化场景时调用。"""
    fresh = FormContextS()
    copy_snapshot(fresh.cmd, dst.cmd)
    dst.state.clear()
    copy_wayline(fresh.wayLine, dst.wayLine)
    copy_motion(fresh.leaderState, dst.leaderState)
    copy_motion(fresh.selfCmd, dst.selfCmd)
    copy_motion(fresh.selfState, dst.selfState)
    dst.selfAccCmd.accEast = fresh.selfAccCmd.accEast
    dst.selfAccCmd.accNorth = fresh.selfAccCmd.accNorth
    dst.selfAccCmd.accUp = fresh.selfAccCmd.accUp
