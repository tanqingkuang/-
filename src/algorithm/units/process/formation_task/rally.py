"""集结任务编排：管理 JOINING→CATCHUP→LOOSE→HOLD 状态机。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from fractions import Fraction
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FollowerStateS,
    FormSnapshotS,
    FormStageE,
    PosCalcStatusS,
    RallyPhaseE,
    RallyPlanS,
    RemoteCmdS,
)
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
)
from src.algorithm.units.process.formation_task.base import (
    FormationTaskBase,
    FormationTaskInitS,
)

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS


@dataclass
class RallyTaskInitS(FormationTaskInitS):
    """Rally 任务初始化参数。注意：dt_s 在 init 时校验，违反则抛异常。"""

    leaderId: str = "R01"  # 长机节点 ID，用于输出长机自身的完整圈分配
    looseScale: float = 3.0  # 集结圆目标点的水平槽位偏置倍数，不参与运行期队形缩放
    convergenceRadius_m: float = 5.0  # LOOSE→HOLD 槽位误差阈值，米
    stableHold_s: float = 5.0  # LOOSE→HOLD 需稳定的时间
    tightRadius_m: float = 2.0  # 完成分析中的入位判定阈值，米
    expectedFollowerIds: list[str] = field(default_factory=list)  # 期望参与集结的僚机 ID；空列表→立即通过（测试用）
    staleTimeout_s: float = 2.0  # 超过此时长未收到某机报文则视为数据失效
    targetPattern: int = 0  # 集结目标队形索引；集结只用单队形，恒为 0（formPos 第 0 行）
    dt_s: float = 0.02  # 控制周期（秒）；进 InitS 才能在 init 时校验 > 0
    # 集结汇合新增参数（RallyJoinPos 使用，Rally 任务直接透传给实体）
    loiter_radius_m: float = 200.0  # 盘旋圆半径，米
    arrival_radius_m: float = 100.0  # 进入盘旋的触发距离，米
    catchup_radius_m: float = 200.0  # CATCHUP→LOOSE 位置误差阈值（dist3d to slot），米
    catchup_heading_thresh_rad: float = 0.17  # CATCHUP→LOOSE 航向误差阈值，弧度（≈10°）
    catchup_stable_s: float = 3.0  # CATCHUP→LOOSE 需连续满足的时长，秒
    altitude_separation_m: float = 60.0  # 待命/JOINING/CATCHUP 各机高度层间隔，米
    loiter_speed_min_mps: float = 14.0  # 盘旋速度下限，用于可达时间区间的最慢整圈时间
    loiter_speed_max_mps: float = 25.0  # 盘旋速度上限，用于可达时间区间的最快整圈时间
    passive: bool = False  # 僚机被动模式：保留入站 cmd，仅允许本地 STANDBY 覆盖
    enabled: bool = True  # 是否启用集结状态机；直接 HOLD 实体关闭后只响应 NONE/HOLD


@dataclass
class RallyTaskInputS:
    """Rally 任务输入端口。注意：动态状态均绑定到 Context 黑板对象。"""

    remote: RemoteCmdS | None = None  # 外部遥控指令
    cmd: FormSnapshotS | None = None  # 当前编队指令快照
    followerStates: list[FollowerStateS] | None = None  # 端口 → Context.followerStates
    clock: AlgorithmClockS | None = None  # 端口 → Context.clock，用于超时判断和计划起点
    posCalcStatus: PosCalcStatusS | None = None  # 端口 → Context.posCalcStatus，读取长机上一拍位置解算反馈


@dataclass
class RallyTaskOutputS:
    """Rally 任务输出端口。注意：协调计划原地写入绑定的黑板对象。"""

    cmd: FormSnapshotS | None = None  # 输出的编队指令
    rallyCompleted: bool = False  # LOOSE→HOLD 正常完成时置 True，仅该拍有效
    rallyPlan: RallyPlanS = field(default_factory=RallyPlanS)  # 端口 → Context.rallyPlan

    @property
    def t_ref(self) -> float:
        """返回公共到达时刻。注意：仅兼容既有任务单测读取。"""
        return self.rallyPlan.t_ref

    @property
    def t_ref_valid(self) -> bool:
        """返回计划有效标记。注意：仅兼容既有任务单测读取。"""
        return self.rallyPlan.valid

    @property
    def loopCounts(self) -> dict[str, int]:
        """返回圈数映射。注意：仅兼容既有任务单测读取。"""
        return self.rallyPlan.loop_counts


class Rally(FormationTaskBase):
    """集结任务编排器：管理 JOINING→CATCHUP→LOOSE→HOLD 单向状态机。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定实体运行环境。注意：任务流程自行维护黑板端口。"""
        cxt = runtime.context
        self._u = RallyTaskInputS(
            remote=runtime.remote,
            cmd=cxt.cmd,
            followerStates=cxt.followerStates,
            clock=cxt.clock,
            posCalcStatus=cxt.posCalcStatus,
        )
        self._y = RallyTaskOutputS(cmd=cxt.cmd, rallyPlan=cxt.rallyPlan)

    @property
    def rally_completed(self) -> bool:
        """返回本拍正常完成事件。注意：仅在 LOOSE 切入 HOLD 的一拍有效。"""
        return self._y.rallyCompleted

    def init(self, cfg: RallyTaskInitS) -> None:
        """按配置初始化 Rally。注意：校验参数合法性，违反则抛 ValueError。"""
        if cfg.enabled and not cfg.leaderId:
            raise ValueError("leaderId must be non-empty")
        if cfg.enabled and cfg.looseScale < 1.0:
            raise ValueError("looseScale must be >= 1.0")
        if cfg.enabled and cfg.staleTimeout_s <= 0:
            raise ValueError("staleTimeout_s must be > 0")
        if cfg.enabled and cfg.dt_s <= 0:
            raise ValueError("dt_s must be > 0")
        if cfg.enabled and (
            not math.isfinite(cfg.loiter_radius_m) or cfg.loiter_radius_m <= 0
        ):
            raise ValueError("loiter_radius_m must be > 0")
        if cfg.enabled and (
            not math.isfinite(cfg.loiter_speed_min_mps)
            or not math.isfinite(cfg.loiter_speed_max_mps)
            or cfg.loiter_speed_min_mps <= 0
            or cfg.loiter_speed_max_mps <= cfg.loiter_speed_min_mps
        ):
            raise ValueError("loiter speed bounds must satisfy 0 < min < max")
        self._conv_radius_m = cfg.convergenceRadius_m
        self._stable_hold_s = cfg.stableHold_s
        self._tight_radius_m = cfg.tightRadius_m
        self._catchup_radius_m = cfg.catchup_radius_m
        self._catchup_heading_thresh_rad = cfg.catchup_heading_thresh_rad
        self._catchup_stable_s = cfg.catchup_stable_s
        self._leader_id = cfg.leaderId
        self._expected_ids: list[str] = list(cfg.expectedFollowerIds)
        self._stale_timeout_s = cfg.staleTimeout_s
        self._initial_pattern = int(cfg.targetPattern)
        self._target_pattern = self._initial_pattern
        self._dt_s = cfg.dt_s
        self._loiter_circumference_m = 2.0 * math.pi * cfg.loiter_radius_m
        self._speed_min = cfg.loiter_speed_min_mps
        self._speed_max = cfg.loiter_speed_max_mps
        self._passive = bool(cfg.passive)
        self._enabled = bool(cfg.enabled)
        # 运行期计时器
        self._catchup_stable_timer: float = 0.0
        self._stable_timer: float = 0.0
        # 首次接受 RALLY 后锁存生命周期；NONE 只停控，不能替代显式 reset 开启新任务。
        self._rally_started: bool = False
        # 协调计划只在每轮集结首次收齐航程时生成，之后不再跟随回报变化。
        self._plan_start_s: float = 0.0
        self._plan_ready: bool = False
        self._t_ref: float = 0.0
        self._loop_counts: dict[str, int] = {}

    def set_pattern_index(self, index: int) -> None:
        """运行时切换目标队形索引。注意：集结完成进入 HOLD 后，下一拍广播生效。"""
        self._target_pattern = int(index)

    def step(self) -> None:
        """推进 Rally 一个处理周期。注意：每拍先置 rallyCompleted=False，再按 remote/step 路由。"""
        self._advance(self._u, self._y)

    def _advance(self, u: RallyTaskInputS, y: RallyTaskOutputS) -> None:
        """按已绑定端口推进任务状态机。"""
        if y.cmd is None or u.clock is None:
            raise ValueError("Rally ports must be bound")
        y.rallyCompleted = False
        if not self._enabled:
            # 直接 HOLD 共用同一任务流程容器，但不进入任何集结子阶段。
            remote_stage = u.remote.stage if u.remote is not None else FormStageE.NONE
            y.cmd.stage = FormStageE.NONE if remote_stage == FormStageE.NONE else FormStageE.HOLD
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = 0 if y.cmd.stage == FormStageE.NONE else self._target_pattern
            return
        if self._passive:
            # 僚机的任务指令已由 Inbound 写入黑板；本流程只保证本地待命优先于旧广播。
            remote_stage = u.remote.stage if u.remote is not None else FormStageE.NONE
            if remote_stage == FormStageE.STANDBY:
                y.cmd.stage = FormStageE.STANDBY
                y.cmd.step = RallyPhaseE.JOINING
            return
        self._write_plan(y)

        remote_stage = u.remote.stage if u.remote is not None else FormStageE.NONE
        now_s = u.clock.now_s
        states = u.followerStates if u.followerStates is not None else []

        if remote_stage == FormStageE.NONE:
            if y.cmd.stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._reset_phase_timers()
                self._write_plan(y)
            y.cmd.stage = FormStageE.NONE
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = 0
            return

        if self._rally_started and y.cmd.stage == FormStageE.NONE:
            # 已开始任务进入 NONE 后，RALLY/STANDBY 都保持停控，避免新位置计划复用旧协调计划。
            y.cmd.stage = FormStageE.NONE
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = 0
            return

        if self._rally_started and remote_stage == FormStageE.STANDBY:
            # 反向 STANDBY 不是中断协议，按当前 RALLY/HOLD 阶段继续处理。
            remote_stage = FormStageE.RALLY

        if remote_stage == FormStageE.HOLD:
            if y.cmd.stage == FormStageE.RALLY:
                self._reset_phase_timers()
            y.cmd.stage = FormStageE.HOLD
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = self._target_pattern
            return
        if remote_stage == FormStageE.STANDBY:
            # STANDBY 是实体内本地盘旋阶段，任务单元只保持状态，不推进 Rally 子流程。
            self._reset_phase_timers()
            self._write_plan(y)
            y.cmd.stage = FormStageE.STANDBY
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = self._target_pattern
            return

        # remote == RALLY
        if y.cmd.stage == FormStageE.HOLD:
            # HOLD 是当前实体生命周期终态；新一轮必须先显式 entity.reset()，控制器不得反向重启。
            y.cmd.stage = FormStageE.HOLD
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = self._target_pattern
            return
        if y.cmd.stage in (FormStageE.NONE, FormStageE.STANDBY):
            # 首次进入集结
            self._reset_phase_timers()
            self._write_plan(y)
            y.cmd.step = RallyPhaseE.JOINING
        self._rally_started = True

        # cmd.stage == RALLY（或从 NONE/STANDBY 首次进入）— 按 cmd.step 路由
        y.cmd.stage = FormStageE.RALLY
        step = RallyPhaseE(y.cmd.step) if y.cmd.step in RallyPhaseE._value2member_map_ else RallyPhaseE.JOINING
        # state_map 在整个 step() 内共用，避免各门控 helper 重复构建。
        state_map: dict[str, FollowerStateS] = {s.id: s for s in states}

        # Rally 任务只看僚机回报的离散门控，不直接读取飞机连续状态。
        # JOINING 使用 EXITED 锁存；CATCHUP 使用位置和航向误差；LOOSE 使用位置误差。
        # 计时器只在连续满足条件时累加，任一僚机失效或超阈值都会清零。
        if step == RallyPhaseE.JOINING:
            if not self._plan_ready:
                # 未收齐时不保留部分映射，下一拍仍以完整快照重新检查。
                path_lengths = self._collect_path_lengths(u, state_map, now_s)
                if path_lengths is not None:
                    # 公共时间从全队航程收齐并生成计划的本拍起算，不能沿用首次进入 RALLY 的时刻。
                    self._plan_start_s = now_s
                    duration_s, self._loop_counts = self._coordinate_paths(path_lengths)
                    self._t_ref = self._plan_start_s + duration_s
                    self._plan_ready = True
                    self._write_plan(y)

            leader_exited = u.posCalcStatus.join_exited if u.posCalcStatus is not None else False
            if self._all_participants_exited(state_map, now_s, leader_exited):
                next_step = RallyPhaseE.CATCHUP
            else:
                next_step = RallyPhaseE.JOINING
            y.cmd.step = next_step
            y.cmd.pattern = self._target_pattern

        elif step == RallyPhaseE.CATCHUP:
            if self._all_catchup_ok(state_map, now_s):
                self._catchup_stable_timer += self._dt_s
                if self._catchup_stable_timer >= self._catchup_stable_s:
                    next_step = RallyPhaseE.LOOSE
                    self._catchup_stable_timer = 0.0
                else:
                    next_step = RallyPhaseE.CATCHUP
            else:
                self._catchup_stable_timer = 0.0
                next_step = RallyPhaseE.CATCHUP
            y.cmd.step = next_step
            y.cmd.pattern = self._target_pattern

        elif step == RallyPhaseE.LOOSE:
            if self._all_followers_ok(state_map, now_s, self._conv_radius_m):
                self._stable_timer += self._dt_s
                # 粗收敛稳定只表示可以检查最终入位，完成事件仍要求全部僚机满足紧门限。
                if (
                    self._stable_timer >= self._stable_hold_s
                    and self._all_followers_ok(state_map, now_s, self._tight_radius_m)
                ):
                    y.cmd.stage = FormStageE.HOLD
                    y.cmd.step = RallyPhaseE.JOINING
                    y.rallyCompleted = True
                    self._stable_timer = 0.0
                else:
                    y.cmd.step = RallyPhaseE.LOOSE
            else:
                self._stable_timer = 0.0
                y.cmd.step = RallyPhaseE.LOOSE
            y.cmd.pattern = self._target_pattern

    def reset(self) -> None:
        """复位 Rally 的动态状态，清除阶段计时器与一次性协调计划。"""
        self._reset_phase_timers()
        self._reset_plan()
        self._rally_started = False
        self._target_pattern = self._initial_pattern

    def _reset_phase_timers(self) -> None:
        """清零阶段推进计时器，不影响已锁存的一次性协调计划。"""
        self._catchup_stable_timer = 0.0
        self._stable_timer = 0.0

    def _reset_plan(self) -> None:
        """仅供显式复位清除一次性协调计划。"""
        # 计划起点、有效标记和搜索结果必须作为同一锁存整体清空。
        self._plan_start_s = 0.0
        self._plan_ready = False
        self._t_ref = 0.0
        self._loop_counts = {}

    def _write_plan(self, output: RallyTaskOutputS) -> None:
        """把锁存计划原地写入黑板。注意：不得替换共享对象及其圈数映射。"""

        output.rallyPlan.t_ref = self._t_ref
        output.rallyPlan.valid = self._plan_ready
        output.rallyPlan.loop_counts.clear()
        output.rallyPlan.loop_counts.update(self._loop_counts)

    def _collect_path_lengths(
        self,
        task_input: RallyTaskInputS,
        state_map: dict[str, FollowerStateS],
        now_s: float,
    ) -> dict[str, float] | None:
        """收齐并校验长机与所有期望僚机的基础航程。"""

        leader_length_m = (
            task_input.posCalcStatus.planned_path_length_m
            if task_input.posCalcStatus is not None
            else -1.0
        )
        # -1.0 是 RallyJoinPos 尚未完成路径规划的协议哨兵。
        if not math.isfinite(leader_length_m) or leader_length_m < 0.0:
            return None
        # 长机与僚机共用节点 ID 到基础航程的统一搜索输入。
        path_lengths = {self._leader_id: leader_length_m}
        # 按配置顺序加入僚机，使输出圈数映射保持确定顺序。
        for follower_id in self._expected_ids:
            entry = state_map.get(follower_id)
            # 僚机航程必须来自当前仍有效的状态回报。
            if entry is None or not self._is_valid(entry, now_s):
                return None
            follower_length_m = entry.plannedPathLength_m
            # 非有限值和未规划哨兵都只阻止本拍生成计划。
            if not math.isfinite(follower_length_m) or follower_length_m < 0.0:
                return None
            path_lengths[follower_id] = follower_length_m
        return path_lengths

    def _coordinate_paths(self, path_lengths: dict[str, float]) -> tuple[float, dict[str, int]]:
        """返回最早公共相对到达时间和各节点额外完整圈数。"""

        circumference = self._loiter_circumference_m
        if not path_lengths:
            raise ValueError("path_lengths must not be empty")
        if not math.isfinite(circumference) or circumference <= 0.0:
            raise ValueError("loiter circumference must be finite and positive")
        if any(not math.isfinite(length) or length < 0.0 for length in path_lengths.values()):
            raise ValueError("path lengths must be finite and non-negative")

        # 保留配置浮点的真实二进制值，避免大圈数下按 ULP 扩大速度窗。
        exact_circumference = Fraction.from_float(circumference)
        exact_speed_min = Fraction.from_float(self._speed_min)
        exact_speed_max = Fraction.from_float(self._speed_max)
        # 相位、跳跃和区间相交均在有理数域完成，浮点只用于最终输出时间。
        exact_lengths = {
            node_id: Fraction.from_float(length)
            for node_id, length in path_lengths.items()
        }
        maximum_base_length = max(exact_lengths.values())
        relative_width = (exact_speed_max - exact_speed_min) / exact_speed_max
        # 基础区间直接相交时，零圈计划既最早也符合最小补圈语义。
        zero_loops = dict.fromkeys(path_lengths, 0)
        zero_time = self._earliest_representable_common_time(exact_lengths, zero_loops)
        if zero_time is not None:
            return zero_time[1], zero_loops

        # 把公共时刻乘以 V_max 得到公共距离 X。对任一候选相位，其他航程序列在 X 前的
        # 最近点与 X 的差恒为模 C 相位差 G；存在公共区间当且仅当 G <= (1-V_min/V_max)X。
        # 因而每个节点相位只需一次数学跳跃，候选总数严格等于节点数，不再逐圈推进。
        phases = {
            node_id: length % exact_circumference
            for node_id, length in exact_lengths.items()
        }
        best_key: tuple[Fraction, int, tuple[int, ...]] | None = None
        best_time: float | None = None
        best_loops: dict[str, int] | None = None
        # 每种候选相位对应“哪架飞机在最早时刻以 V_max 到达”的一种可能。
        for candidate_id, candidate_phase in phases.items():
            # 模差给出所有其他等周期航程序列落后该候选点的最坏距离。
            # 该模差不随圈数变化，所以每个候选相位只需评估一次。
            maximum_gap = max(
                (candidate_phase - phase) % exact_circumference
                for phase in phases.values()
            )
            # 直接解 G <= (1 - V_min/V_max)X，跳到首个可能可行的公共距离。
            minimum_distance = max(maximum_base_length, maximum_gap / relative_width)
            candidate_base = exact_lengths[candidate_id]
            required_loops = (minimum_distance - candidate_base) / exact_circumference
            candidate_loops = max(0, self._ceil_fraction(required_loops))
            candidate_distance = candidate_base + candidate_loops * exact_circumference
            # 候选相位只决定公共距离；各机仍选择覆盖速度窗的最小非负圈数。
            loop_counts = self._loops_for_common_distance(candidate_distance, exact_lengths)
            if loop_counts is None:
                continue
            common_time = self._earliest_representable_common_time(exact_lengths, loop_counts)
            if common_time is None:
                continue
            exact_lower, represented_time = common_time
            # 并列最早时优先总圈数更少，再按节点输入顺序确定唯一结果。
            candidate_key = (
                exact_lower,
                sum(loop_counts.values()),
                tuple(loop_counts[node_id] for node_id in path_lengths),
            )
            if best_key is None or candidate_key < best_key:
                best_key = candidate_key
                best_time = represented_time
                best_loops = loop_counts

        if best_time is None or best_loops is None:
            raise RuntimeError("Rally path-time coordination exceeds finite numeric range")
        return best_time, best_loops

    def _loops_for_common_distance(
        self,
        common_distance: Fraction,
        path_lengths: dict[str, Fraction],
    ) -> dict[str, int] | None:
        """返回覆盖公共距离窗的最小圈数；当前候选不可达时返回 None。"""

        exact_circumference = Fraction.from_float(self._loiter_circumference_m)
        speed_ratio = Fraction.from_float(self._speed_min) / Fraction.from_float(self._speed_max)
        minimum_distance = common_distance * speed_ratio
        loop_counts: dict[str, int] = {}
        for node_id, length in path_lengths.items():
            # 下界决定最小圈数，上界随后验证该圈数是否仍能以 V_max 按时到达。
            required_loops = (minimum_distance - length) / exact_circumference
            loops = max(0, self._ceil_fraction(required_loops))
            assigned_distance = length + loops * exact_circumference
            # 使用精确有理数比较真实闭区间，任何浮点 ULP 都不能放宽物理边界。
            if assigned_distance < minimum_distance or assigned_distance > common_distance:
                return None
            loop_counts[node_id] = loops
        return loop_counts

    def _earliest_representable_common_time(
        self,
        path_lengths: dict[str, Fraction],
        loop_counts: dict[str, int],
    ) -> tuple[Fraction, float] | None:
        """返回计划精确下界及不早于该下界的最小有限浮点时刻。"""

        circumference = Fraction.from_float(self._loiter_circumference_m)
        speed_min = Fraction.from_float(self._speed_min)
        speed_max = Fraction.from_float(self._speed_max)
        distances = {
            node_id: length + loop_counts[node_id] * circumference
            for node_id, length in path_lengths.items()
        }
        # 公共物理时刻必须位于所有 [D/V_max, D/V_min] 区间的精确交集。
        exact_lower = max(distance / speed_max for distance in distances.values())
        exact_upper = min(distance / speed_min for distance in distances.values())
        # 交集为空时拒绝该分配，不允许按公共距离量级引入容差。
        if exact_lower > exact_upper:
            return None
        try:
            represented_time = float(exact_lower)
        except OverflowError:
            return None
        if not math.isfinite(represented_time):
            return None
        # float() 采用最近舍入；若落在精确下界下方，则推进到紧邻的上一个可表示时刻。
        if Fraction.from_float(represented_time) < exact_lower:
            represented_time = math.nextafter(represented_time, math.inf)
        # 极窄合法速度窗可能没有位于精确交集内的 float；圈数可行性仍由上面的有理区间判定。
        return exact_lower, represented_time

    @staticmethod
    def _ceil_fraction(value: Fraction) -> int:
        """返回精确有理数的向上取整整数。"""

        return -(-value.numerator // value.denominator)

    def _is_valid(self, entry: FollowerStateS, now_s: float) -> bool:
        """判断单架僚机状态条目是否有效（未超时且 valid=True）。"""
        if not entry.valid or not math.isfinite(entry.lastUpdate_s) or not math.isfinite(now_s):
            return False
        return (now_s - entry.lastUpdate_s) <= self._stale_timeout_s

    def _all_participants_exited(
        self, state_map: dict[str, FollowerStateS], now_s: float, leader_exited: bool
    ) -> bool:
        """JOINING→CATCHUP 门控：期望僚机全部 EXITED 且长机自身也已 EXITED。"""
        if not leader_exited:
            return False
        if not self._expected_ids:
            return True
        for fid in self._expected_ids:
            entry = state_map.get(fid)
            if entry is None:
                return False
            # EXITED 是终态：曾经切出的僚机不因随后丢链而被撤销。
            if entry.rally_state == RALLY_STATE_EXITED:
                continue
            # 尚未 EXITED：要求报文新鲜，防止无效数据误判为"仍在飞"。
            if not self._is_valid(entry, now_s):
                return False
            return False  # 新鲜报文但 state != EXITED，确认尚未切出
        return True

    def _all_catchup_ok(self, state_map: dict[str, FollowerStateS], now_s: float) -> bool:
        """CATCHUP→LOOSE 门控：期望僚机同时满足三维位置（dist3d to slot）和航向误差阈值。"""
        if not self._expected_ids:
            return True
        for fid in self._expected_ids:
            entry = state_map.get(fid)
            if entry is None or not self._is_valid(entry, now_s):
                return False
            if not math.isfinite(entry.posErr_m) or entry.posErr_m >= self._catchup_radius_m:
                return False
            if (
                not math.isfinite(entry.headingErr_rad)
                or entry.headingErr_rad >= self._catchup_heading_thresh_rad
            ):
                return False
        return True

    def _all_followers_ok(self, state_map: dict[str, FollowerStateS], now_s: float, threshold_m: float) -> bool:
        """LOOSE→HOLD 门控：期望僚机全部有效且槽位误差收敛。"""
        if not self._expected_ids:
            return True
        for fid in self._expected_ids:
            entry = state_map.get(fid)
            if (
                entry is None
                or not self._is_valid(entry, now_s)
                or not math.isfinite(entry.posErr_m)
                or entry.posErr_m >= threshold_m
            ):
                return False
        return True
