"""编队算法黑板。注意：实体内多单元通过该对象共享状态。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FollowerStateS,
    FormSnapshotS,
    MotionProfS,
    RallySlotScaleS,
    WayLineS,
    copy_motion,
    copy_snapshot,
    copy_wayline,
)


@dataclass
class FormContextS:
    """单个实体持有的跨帧编队状态。注意：reset 时需要清理运行期缓存。"""

    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)  # 当前编队指令(任务单元产出)
    state: list[FormSnapshotS] = field(default_factory=list)  # 历史/各机状态快照列表
    wayLine: WayLineS = field(default_factory=WayLineS)  # 当前跟踪航段(航路规划产出)
    nextWayLine: WayLineS = field(default_factory=WayLineS)  # 下一航段(供曲率前馈前瞻跨段采样)
    leaderState: MotionProfS = field(default_factory=MotionProfS)  # 长机状态(僚机由入站解析得到)
    selfCmd: MotionProfS = field(default_factory=MotionProfS)  # 本机目标运动状态(位置解算产出)
    selfState: MotionProfS = field(default_factory=MotionProfS)  # 本机实测运动状态(外部反馈)
    selfAccCmd: AccInEarthS = field(default_factory=AccInEarthS)  # 本机加速度指令(位置跟踪产出)
    slotScale: RallySlotScaleS = field(default_factory=RallySlotScaleS)  # 槽位缩放因子(Rally写/SlotGeometry读)
    followerStates: list[FollowerStateS] = field(default_factory=list)  # 僚机集结状态(FollowerStatus写/Rally读)
    rally_t_ref: float = 0.0  # 固定公共到达时刻，仅用于 RallyJoinPos 全航程调速（秒）
    rally_t_ref_valid: bool = False  # 是否已收齐全队基础航程并生成固定协调计划
    rally_loop_counts: dict[str, int] = field(default_factory=dict)  # 固定计划的节点圈数映射（长机任务写/广播与僚机接线读）


def reset_context(dst: FormContextS) -> None:
    """重置全局算法上下文，清空本轮仿真遗留数据。注意：只应在重新初始化场景时调用。"""
    # 构造一份全默认值的临时上下文，逐字段拷回 dst：原地清零而非替换对象，
    # 这样各单元先前绑定到 dst 字段的端口引用依然有效
    fresh = FormContextS()
    copy_snapshot(fresh.cmd, dst.cmd)  # 重置编队指令
    dst.state.clear()  # 清空状态快照列表
    copy_wayline(fresh.wayLine, dst.wayLine)  # 重置当前航段
    copy_wayline(fresh.nextWayLine, dst.nextWayLine)  # 重置下一航段
    copy_motion(fresh.leaderState, dst.leaderState)  # 重置长机状态
    copy_motion(fresh.selfCmd, dst.selfCmd)  # 重置本机目标状态
    copy_motion(fresh.selfState, dst.selfState)  # 重置本机实测状态
    # 加速度指令逐分量清零(无 copy 辅助函数，直接赋值)
    dst.selfAccCmd.accEast = fresh.selfAccCmd.accEast
    dst.selfAccCmd.accNorth = fresh.selfAccCmd.accNorth
    dst.selfAccCmd.accUp = fresh.selfAccCmd.accUp
    # 集结扩展字段复位
    dst.slotScale.scale = fresh.slotScale.scale
    dst.slotScale.scaleRate = fresh.slotScale.scaleRate
    dst.followerStates.clear()
    dst.rally_t_ref = 0.0
    dst.rally_t_ref_valid = False
    dst.rally_loop_counts.clear()
