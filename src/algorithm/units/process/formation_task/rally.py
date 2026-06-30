"""集结任务编排：管理 APPROACH→LOOSE→COMPRESS→HOLD 状态机。注意：写出 cmd 和 slotScale，不直接感知单机位置。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    FollowerStateS,
    FormPatE,
    FormStageE,
    RallyPhaseE,
    RallySlotScaleS,
)
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RALLY_STATE_LOITERING,
)
from src.algorithm.units.process.formation_task.base import (
    FormationTaskBase,
    FormationTaskInitS,
    FormationTaskInputS,
    FormationTaskOutputS,
)


@dataclass
class RallyTaskInitS(FormationTaskInitS):
    """Rally 任务初始化参数。注意：dt_s 在 init 时校验，违反则抛异常。"""

    looseScale: float = 3.0  # 松散槽位放大倍数（松散间距=最终间距×looseScale）
    convergenceRadius_m: float = 5.0  # LOOSE→COMPRESS 槽位误差阈值，米
    stableHold_s: float = 5.0  # LOOSE→COMPRESS 需稳定的时间
    compressTime_s: float = 30.0  # COMPRESS 阶段持续时间（scale 从 looseScale→1.0）
    tightRadius_m: float = 2.0  # COMPRESS→HOLD 精度阈值，米
    expectedFollowerIds: list[str] = field(default_factory=list)  # 期望参与集结的僚机 ID；空列表→立即通过（测试用）
    staleTimeout_s: float = 2.0  # 超过此时长未收到某机报文则视为数据失效
    targetPattern: FormPatE = FormPatE.TRIANGLE  # LOOSE/COMPRESS 时 cmd.pattern 写入此值
    dt_s: float = 0.02  # 控制周期（秒）；进 InitS 才能在 init 时校验 > 0
    # 集结汇合新增参数（RallyJoinPos 使用，Rally 任务直接透传给实体）
    loiter_radius_m: float = 200.0  # 盘旋圆半径，米
    arrival_radius_m: float = 100.0  # 进入盘旋的触发距离，米
    mission_heading_deg: float = 0.0  # 切出后飞行方向（度，从东向起算）
    catchup_radius_m: float = 200.0  # CATCHUP→LOOSE 位置误差阈值（dist2d to slot），米
    catchup_heading_thresh_rad: float = 0.17  # CATCHUP→LOOSE 航向误差阈值，弧度（≈10°）
    catchup_stable_s: float = 3.0  # CATCHUP→LOOSE 需连续满足的时长，秒
    catchup_kp_speed: float = 0.05  # 沿航迹误差→速度增益（m/s per m）


@dataclass
class RallyTaskInputS(FormationTaskInputS):
    """Rally 任务输入端口。注意：followerStates 绑到 Context.followerStates。"""

    # 继承 remote: RemoteCmdS, cmd: FormSnapshotS
    followerStates: list[FollowerStateS] | None = None  # 端口 → Context.followerStates
    now_s: float = 0.0  # 当前仿真时间（秒），由实体从边界输入注入，用于超时判断
    leader_eta_s: float = 0.0  # 长机自身 RallyJoinPos.eta_s（由长机实体每帧注入）
    leader_join_exited: bool = False  # 长机自身是否已 EXITED（由长机实体每帧注入）
    leader_join_flying: bool = False  # 长机自身是否仍在 FLYING 状态（用于 T_ref 计算，LOITERING 不计入）


@dataclass
class RallyTaskOutputS(FormationTaskOutputS):
    """Rally 任务输出端口。注意：slotScale 绑到 Context.slotScale。"""

    # 继承 cmd: FormSnapshotS
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale
    rallyCompleted: bool = False  # COMPRESS→HOLD 正常完成时置 True，仅该拍有效
    t_ref: float = 0.0  # 本帧计算出的集结基准时刻（最晚 ETA），供长机广播给僚机
    t_ref_valid: bool = False  # 是否已收齐所有参与者的首个有效汇合状态


class Rally(FormationTaskBase):
    """集结任务编排器：管理 APPROACH→LOOSE→COMPRESS→HOLD 状态机。注意：子阶段通过 cmd.step 编码，不另增枚举。"""

    def init(self, cfg: RallyTaskInitS) -> None:
        """按配置初始化 Rally。注意：校验参数合法性，违反则抛 ValueError。"""
        if cfg.looseScale < 1.0:
            raise ValueError("looseScale must be >= 1.0")
        if cfg.compressTime_s <= 0:
            raise ValueError("compressTime_s must be > 0")
        if cfg.staleTimeout_s <= 0:
            raise ValueError("staleTimeout_s must be > 0")
        if cfg.dt_s <= 0:
            raise ValueError("dt_s must be > 0")
        self._loose_scale = cfg.looseScale
        self._conv_radius_m = cfg.convergenceRadius_m
        self._stable_hold_s = cfg.stableHold_s
        self._compress_time_s = cfg.compressTime_s
        self._tight_radius_m = cfg.tightRadius_m
        self._catchup_radius_m = cfg.catchup_radius_m
        self._catchup_heading_thresh_rad = cfg.catchup_heading_thresh_rad
        self._catchup_stable_s = cfg.catchup_stable_s
        self._expected_ids: list[str] = list(cfg.expectedFollowerIds)
        self._stale_timeout_s = cfg.staleTimeout_s
        self._target_pattern = cfg.targetPattern
        self._dt_s = cfg.dt_s
        # 运行期计时器
        self._catchup_stable_timer: float = 0.0
        self._stable_timer: float = 0.0
        self._compress_elapsed: float = 0.0
        self._t_ref: float = 0.0  # 最近一次有效 T_ref（有 FLYING 参与者时更新，之后锁存）

    def step(self, u: RallyTaskInputS, y: RallyTaskOutputS) -> None:
        """推进 Rally 一个处理周期。注意：每拍先置 rallyCompleted=False，再按 remote/step 路由。"""
        if y.cmd is None or y.slotScale is None:
            raise ValueError("Rally output ports must be bound")
        y.rallyCompleted = False
        y.t_ref_valid = False

        remote_stage = u.remote.stage if u.remote is not None else FormStageE.NONE
        now_s = u.now_s if hasattr(u, "now_s") else 0.0
        states = u.followerStates if u.followerStates is not None else []

        if remote_stage == FormStageE.NONE:
            if y.cmd.stage in (FormStageE.RALLY, FormStageE.HOLD):
                self._reset_timers()
            y.cmd.stage = FormStageE.NONE
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = FormPatE.NONE
            y.slotScale.scale = self._loose_scale
            y.slotScale.scaleRate = 0.0
            return

        if remote_stage == FormStageE.HOLD:
            if y.cmd.stage == FormStageE.RALLY:
                self._reset_timers()
            y.cmd.stage = FormStageE.HOLD
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = self._target_pattern
            y.slotScale.scale = 1.0
            y.slotScale.scaleRate = 0.0
            return

        # remote == RALLY
        if y.cmd.stage == FormStageE.HOLD:
            # 已完成集结，HOLD 是终态；只有先发 NONE 再发 RALLY 才能重启
            y.cmd.stage = FormStageE.HOLD
            y.cmd.step = RallyPhaseE.JOINING
            y.cmd.pattern = self._target_pattern
            y.slotScale.scale = 1.0
            y.slotScale.scaleRate = 0.0
            return
        if y.cmd.stage == FormStageE.NONE:
            # 首次进入集结
            self._reset_timers()
            y.cmd.step = RallyPhaseE.JOINING

        # cmd.stage == RALLY（或从 NONE 首次进入）— 按 cmd.step 路由
        y.cmd.stage = FormStageE.RALLY
        step = RallyPhaseE(y.cmd.step) if y.cmd.step in RallyPhaseE._value2member_map_ else RallyPhaseE.JOINING
        # state_map 在整个 step() 内共用，避免各门控 helper 重复构建。
        state_map: dict[str, FollowerStateS] = {s.id: s for s in states}

        # Rally 任务只看僚机回报的离散门控，不直接读取飞机连续状态。
        # APPROACH 门控使用 arrived 锁存，允许先到机在近场等待时 posErr_m 变大。
        # LOOSE/COMPRESS 门控改用 posErr_m，因为此时 selfCmd 已转为松散/压缩槽位目标。
        # 计时器只在连续满足条件时累加，任一僚机失效或超阈值都会清零。
        # slotScale 每拍都写出，确保僚机漏收上一帧广播后仍可从最新消息恢复。
        if step == RallyPhaseE.JOINING:
            expected_states = [state_map[fid] for fid in self._expected_ids if fid in state_map]
            # 计算 T_ref：所有仍在 FLYING 状态的有效参与者中 ETA 最大值
            flying_etas = [
                s.eta_s for s in expected_states
                if self._is_valid(s, now_s)
                and s.rally_state == RALLY_STATE_FLYING
                and math.isfinite(s.eta_s)
                and s.eta_s > 0.0
            ]
            if u.leader_join_flying and math.isfinite(u.leader_eta_s) and u.leader_eta_s > 0.0:
                flying_etas.append(u.leader_eta_s)
            # 有 FLYING 参与者时更新 t_ref；全部离开后锁存最后值，避免"最后一架到达时塌缩为 now_s"。
            if flying_etas:
                self._t_ref = max(flying_etas)
            y.t_ref = self._t_ref if self._t_ref > 0.0 else now_s
            # 单机无僚机场景不需要等待通信；多机时须确认长机和全部期望僚机已实际运行过汇合解算。
            leader_ready = not self._expected_ids or (
                not u.leader_join_flying
                or (math.isfinite(u.leader_eta_s) and u.leader_eta_s > now_s)
            )
            followers_ready = len(expected_states) == len(self._expected_ids) and all(
                self._join_state_initialized(entry, now_s) for entry in expected_states
            )
            y.t_ref_valid = leader_ready and followers_ready

            if self._all_participants_exited(state_map, now_s, u.leader_join_exited):
                next_step = RallyPhaseE.CATCHUP
            else:
                next_step = RallyPhaseE.JOINING
            y.cmd.step = next_step
            y.cmd.pattern = self._target_pattern
            y.slotScale.scale = self._loose_scale
            y.slotScale.scaleRate = 0.0

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
            y.slotScale.scale = self._loose_scale
            y.slotScale.scaleRate = 0.0

        elif step == RallyPhaseE.LOOSE:
            if self._all_followers_ok(state_map, now_s, self._conv_radius_m):
                self._stable_timer += self._dt_s
                if self._stable_timer >= self._stable_hold_s:
                    next_step = RallyPhaseE.COMPRESS
                    self._stable_timer = 0.0
                else:
                    next_step = RallyPhaseE.LOOSE
            else:
                self._stable_timer = 0.0
                next_step = RallyPhaseE.LOOSE
            y.cmd.step = next_step
            y.cmd.pattern = self._target_pattern
            y.slotScale.scale = self._loose_scale
            y.slotScale.scaleRate = 0.0

        else:  # step == RallyPhaseE.COMPRESS
            self._compress_elapsed += self._dt_s
            progress = self._compress_elapsed / self._compress_time_s
            scale = self._loose_scale - (self._loose_scale - 1.0) * progress
            if scale <= 1.0:
                scale = 1.0
                scaleRate = 0.0
            else:
                scaleRate = -(self._loose_scale - 1.0) / self._compress_time_s
            if scale == 1.0 and self._all_followers_ok(state_map, now_s, self._tight_radius_m):
                y.cmd.stage = FormStageE.HOLD
                y.cmd.step = RallyPhaseE.JOINING
                y.rallyCompleted = True
            else:
                y.cmd.stage = FormStageE.RALLY
                y.cmd.step = RallyPhaseE.COMPRESS
            y.cmd.pattern = self._target_pattern
            y.slotScale.scale = scale
            y.slotScale.scaleRate = scaleRate

    def reset(self) -> None:
        """复位 Rally 的动态状态。注意：保留配置参数，只清理运行期计时器与指令状态。"""
        self._reset_timers()

    def _reset_timers(self) -> None:
        """清零所有内部计时器。注意：不清配置，仅清运行期状态。"""
        self._catchup_stable_timer = 0.0
        self._stable_timer = 0.0
        self._compress_elapsed = 0.0
        self._t_ref = 0.0

    def _is_valid(self, entry: FollowerStateS, now_s: float) -> bool:
        """判断单架僚机状态条目是否有效（未超时且 valid=True）。"""
        if not entry.valid or not math.isfinite(entry.lastUpdate_s) or not math.isfinite(now_s):
            return False
        return (now_s - entry.lastUpdate_s) <= self._stale_timeout_s

    def _all_participants_exited(
        self, state_map: dict[str, FollowerStateS], now_s: float, leader_exited: bool
    ) -> bool:
        """JOINING→LOOSE 门控：期望僚机全部 EXITED 且长机自身也已 EXITED。"""
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

    def _join_state_initialized(self, entry: FollowerStateS, now_s: float) -> bool:
        """判断僚机是否已实际执行过汇合解算，排除冷启动默认 FLYING/ETA=0 回报。"""
        if not self._is_valid(entry, now_s):
            return False
        if entry.rally_state == RALLY_STATE_FLYING:
            return math.isfinite(entry.eta_s) and entry.eta_s > now_s
        return entry.rally_state in (RALLY_STATE_LOITERING, RALLY_STATE_EXITED)

    def _all_catchup_ok(self, state_map: dict[str, FollowerStateS], now_s: float) -> bool:
        """CATCHUP→LOOSE 门控：期望僚机同时满足位置（dist2d to slot）和航向误差阈值。"""
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
        """LOOSE→COMPRESS 和 COMPRESS→HOLD 门控：期望僚机全部有效且槽位误差收敛。"""
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
