"""领航跟随集结算法的低层测试。"""

from __future__ import annotations

import math
import unittest

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    CommDirE,
    FollowerStateS,
    FormCommInitS,
    FormPosS,
    FormSelfInitS,
    FormSnapshotS,
    FormStageE,
    FormationAnalysisS,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
    RallyPhaseE,
    RallySlotScaleS,
    RemoteCmdS,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
    copy_follower_state,
    copy_formation_analysis,
    copy_rally_slot_scale,
)
from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity
from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS, VelCmdLimitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RALLY_STATE_LOITERING,
    RALLY_STATE_STANDBY,
    RallyJoinPos,
    RallyJoinPosInitS,
    RallyJoinPosInputS,
)
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
from src.algorithm.units.process.formation_task.rally import Rally, RallyTaskInitS, RallyTaskInputS, RallyTaskOutputS
from src.algorithm.units.process.inbound.base import InboundInputS
from src.algorithm.units.process.inbound.follower_status import FollowerStatus, FollowerStatusInputS, FollowerStatusOutputS
from src.algorithm.units.process.inbound.rally_leader_follower import RallyLeaderFollower, RallyLeaderFollowerOutputS
from src.algorithm.units.process.outbound.base import OutboundInitS, OutboundOutputS
from src.algorithm.units.process.outbound.follower_broadcast import (
    FOLLOWER_STATUS_TOPIC,
    FollowerBroadcast,
    FollowerBroadcastInitS,
    FollowerBroadcastInputS,
)
from src.algorithm.units.process.outbound.rally_leader_broadcast import (
    RallyLeaderBroadcast,
    RallyLeaderBroadcastInputS,
    _motion_payload,
)
from src.common.envelope import MessageEnvelope


def _pos(east: float = 0.0, north: float = 0.0, h: float = 0.0) -> PosInEarthS:
    """构造位置结构，便于测试表达。"""

    return PosInEarthS(east=east, north=north, h=h)


def _motion(
    east: float = 0.0,
    north: float = 0.0,
    h: float = 0.0,
    v_east: float = 0.0,
    v_north: float = 0.0,
    v_up: float = 0.0,
    vd: float | None = None,
    v_psi: float = 0.0,
    d_v_psi: float = 0.0,
) -> MotionProfS:
    """构造运动状态，默认地速取水平速度模长。"""

    ground_speed = math.hypot(v_east, v_north) if vd is None else vd
    return MotionProfS(
        pos=PosInEarthS(east=east, north=north, h=h),
        v=VdInEarthS(
            vEast=v_east,
            vNorth=v_north,
            vUp=v_up,
            vd=ground_speed,
            vPsi=v_psi,
            dVPsi=d_v_psi,
        ),
    )


def _follower_state(
    node_id: str,
    *,
    pos_err_m: float = 0.0,
    arrived: int = 0,
    valid: bool = True,
    last_update_s: float = 0.0,
    rally_state: str = "EXITED",
    eta_s: float = 0.0,
    reached_slot_once: bool = False,
) -> FollowerStateS:
    """构造长机侧保存的僚机状态。"""

    return FollowerStateS(
        id=node_id,
        pos=PosInEarthS(),
        posErr_m=pos_err_m,
        arrived=arrived,
        valid=valid,
        lastUpdate_s=last_update_s,
        rally_state=rally_state,
        eta_s=eta_s,
        reachedSlotOnce=reached_slot_once,
    )


def _follower_status_msg(
    source: str = "R02",
    *,
    pos_east: float = 0.0,
    pos_north: float = 0.0,
    pos_h: float = 500.0,
    pos_err_m: float = 0.0,
    arrived: int = 0,
    rally_state: str = "EXITED",
    eta_s: float = 0.0,
) -> MessageEnvelope:
    """构造僚机回报消息。"""

    return MessageEnvelope(
        topic=FOLLOWER_STATUS_TOPIC,
        source=source,
        target="R01",
        timestamp=0.0,
        payload={
            "id": source,
            "pos_east": pos_east,
            "pos_north": pos_north,
            "pos_h": pos_h,
            "pos_err_m": pos_err_m,
            "arrived": arrived,
            "rally_state": rally_state,
            "eta_s": eta_s,
        },
    )


def _leader_msg(
    *,
    stage: FormStageE = FormStageE.RALLY,
    pattern: int = 0,
    step: int = 0,
    scale: float = 3.0,
    scale_rate: float = 0.0,
    leader_state: MotionProfS | None = None,
    t_ref: float = 0.0,
    t_ref_valid: bool = True,
) -> MessageEnvelope:
    """构造集结长机广播消息。"""

    state = leader_state or _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
    return MessageEnvelope(
        topic="formation.leader",
        source="R01",
        target=["R02"],
        timestamp=0.0,
        payload={
            "leader_state": _motion_payload(state),
            "cmd": {"stage": int(stage), "pattern": int(pattern), "step": step},
            "slot_scale": {"scale": scale, "scale_rate": scale_rate},
            "t_ref": t_ref,
            "t_ref_valid": t_ref_valid,
        },
    )


def _rally_task(
    expected: tuple[str, ...] = ("R02", "R03"),
    *,
    dt_s: float = 0.1,
    stable_hold_s: float = 0.2,
    compress_time_s: float = 1.0,
    catchup_radius_m: float = 200.0,
    catchup_stable_s: float = 0.0,
) -> Rally:
    """构造测试用 Rally 任务单元。"""

    task = Rally()
    task.init(
        RallyTaskInitS(
            looseScale=3.0,
            convergenceRadius_m=5.0,
            stableHold_s=stable_hold_s,
            compressTime_s=compress_time_s,
            tightRadius_m=2.0,
            expectedFollowerIds=list(expected),
            staleTimeout_s=0.5,
            targetPattern=0,
            dt_s=dt_s,
            catchup_radius_m=catchup_radius_m,
            catchup_stable_s=catchup_stable_s,
        )
    )
    return task


def _task_step(
    task: Rally,
    ctx: FormContextS,
    *,
    remote: FormStageE,
    states: list[FollowerStateS] | None = None,
    now_s: float = 0.0,
    leader_join_exited: bool = True,
    leader_join_flying: bool = False,
    leader_join_reached_slot_once: bool = False,
    leader_eta_s: float = 0.0,
) -> RallyTaskOutputS:
    """推进 Rally 任务一拍并返回输出端口。"""

    output = RallyTaskOutputS(cmd=ctx.cmd, slotScale=ctx.slotScale)
    task.step(
        RallyTaskInputS(
            remote=RemoteCmdS(remote),
            cmd=ctx.cmd,
            followerStates=states or [],
            now_s=now_s,
            leader_join_exited=leader_join_exited,
            leader_join_flying=leader_join_flying,
            leader_join_reached_slot_once=leader_join_reached_slot_once,
            leader_eta_s=leader_eta_s,
        ),
        output,
    )
    return output


def _comm_init() -> FormCommInitS:
    """构造三机集结通信与三角队形槽位。"""

    return FormCommInitS(
        netWork=[
            NetWorkS("R01", "R02", CommDirE.DUPLEX),
            NetWorkS("R01", "R03", CommDirE.DUPLEX),
        ],
        formPat=[0],
        formPos=[
            [
                FormPosS("R01", 0.0, 0.0, 0.0),
                FormPosS("R02", -10.0, 0.0, -5.0),
                FormPosS("R03", -10.0, 0.0, 5.0),
            ]
        ],
    )


def _line(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speed: float = 20.0,
    idx: int = 0,
) -> WayLineS:
    """构造直线航段。"""

    return WayLineS(
        idx=idx,
        start=WayPointS(idx=idx, pos=PosInEarthS(*start)),
        end=WayPointS(idx=idx + 1, pos=PosInEarthS(*end)),
        vdCmd=speed,
    )


def _route(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speed: float = 20.0,
) -> list[WayPointInputS]:
    """构造两点航线（WayPointInputS 列表）。"""

    return [
        WayPointInputS(idx=0, pos=PosInEarthS(*start), vdCmd=speed),
        WayPointInputS(idx=1, pos=PosInEarthS(*end), vdCmd=speed),
    ]


def _rally_cfg(
    *,
    expected: tuple[str, ...] = ("R02", "R03"),
    dt_s: float = 0.1,
    stable_hold_s: float = 0.1,
    compress_time_s: float = 0.1,
    catchup_stable_s: float = 0.0,
) -> RallyTaskInitS:
    """构造实体测试用集结配置。"""

    return RallyTaskInitS(
        looseScale=3.0,
        convergenceRadius_m=5.0,
        stableHold_s=stable_hold_s,
        compressTime_s=compress_time_s,
        tightRadius_m=2.0,
        expectedFollowerIds=list(expected),
        staleTimeout_s=1.0,
        targetPattern=0,
        dt_s=dt_s,
        catchup_stable_s=catchup_stable_s,
    )


class RallyLeafTypesAndContextTests(unittest.TestCase):
    """验证集结扩展叶类型和上下文字段。"""

    def test_rally_leaf_type_defaults_and_copy_helpers(self) -> None:
        """验证默认值与复制函数覆盖所有集结扩展字段。"""

        self.assertEqual(FormStageE.STANDBY, 4)
        self.assertEqual(RallySlotScaleS(), RallySlotScaleS(scale=1.0, scaleRate=0.0))
        self.assertFalse(FollowerStateS().valid)
        self.assertEqual(FormationAnalysisS(), FormationAnalysisS())

        slot_src = RallySlotScaleS(scale=2.5, scaleRate=-0.1)
        slot_dst = RallySlotScaleS()
        copy_rally_slot_scale(slot_src, slot_dst)
        self.assertEqual(slot_dst, slot_src)

        follower_src = FollowerStateS(
            id="R02",
            pos=PosInEarthS(1.0, 2.0, 3.0),
            posErr_m=4.0,
            arrived=1,
            valid=True,
            lastUpdate_s=5.0,
        )
        follower_dst = FollowerStateS()
        copy_follower_state(follower_src, follower_dst)
        self.assertEqual(follower_dst, follower_src)
        self.assertIsNot(follower_dst.pos, follower_src.pos)

        analysis_src = FormationAnalysisS(posErrMax_m=3.0, posErrRms_m=2.0, inPositionCount=1, totalCount=2)
        analysis_dst = FormationAnalysisS()
        copy_formation_analysis(analysis_src, analysis_dst)
        self.assertEqual(analysis_dst, analysis_src)

    def test_context_contains_rally_fields_and_reset_clears_them(self) -> None:
        """验证 Context 拥有独立的集结状态列表，reset 原地清理集结字段。"""

        first = FormContextS()
        second = FormContextS()
        self.assertIsNot(first.followerStates, second.followerStates)

        first.slotScale.scale = 3.0
        first.slotScale.scaleRate = -0.2
        first.followerStates.append(_follower_state("R02"))
        first.rally_t_ref_valid = True

        reset_context(first)

        self.assertEqual(first.slotScale, RallySlotScaleS())
        self.assertEqual(first.followerStates, [])
        self.assertFalse(first.rally_t_ref_valid)


class EntityBoundaryTypesTests(unittest.TestCase):
    """验证实体边界结构已包含集结输入输出字段。"""

    def test_entity_boundary_defaults_include_rally_fields(self) -> None:
        """验证扩展字段默认值可供旧实体和集结实体同时使用。"""

        init = EntityInitS()
        self.assertIsNone(init.rally_route)
        self.assertIsNone(init.rally_cfg)
        self.assertEqual(init.rally_approach_speed_mps, 20.0)
        self.assertEqual(init.rally_leader_id, "")
        self.assertEqual(EntityInputS().now_s, 0.0)
        self.assertIsNone(EntityOutputS().formationAnalysis)


class RallyPhaseEnumTests(unittest.TestCase):
    """验证 RallyPhaseE 枚举值与历史整数协议兼容，并检查状态机写出枚举键名。"""

    def test_rally_phase_e_values_match_legacy_integers(self) -> None:
        """RallyPhaseE 的整数值必须与历史裸整数协议一致，以防向后不兼容。"""
        from src.algorithm.context.leaf_types import RallyPhaseE

        self.assertEqual(int(RallyPhaseE.JOINING), 0)
        self.assertEqual(int(RallyPhaseE.CATCHUP), 1)
        self.assertEqual(int(RallyPhaseE.LOOSE), 2)
        self.assertEqual(int(RallyPhaseE.COMPRESS), 3)

    def test_task_writes_rally_phase_e_to_cmd_step(self) -> None:
        """Rally 任务写入 cmd.step 的值应为 RallyPhaseE 成员，可按名称反查。"""
        from src.algorithm.context.leaf_types import RallyPhaseE

        task = _rally_task(expected=(), dt_s=0.1)
        ctx = FormContextS()
        _task_step(task, ctx, remote=FormStageE.RALLY, states=[], now_s=0.0)
        # cmd.step 可安全转换为 RallyPhaseE（不抛 ValueError）
        phase = RallyPhaseE(ctx.cmd.step)
        self.assertEqual(phase, RallyPhaseE.CATCHUP)  # 无僚机期望 → 立即 JOINING→CATCHUP


class RallyTaskTests(unittest.TestCase):
    """验证集结任务状态机和遥控语义。"""

    def test_init_rejects_invalid_parameters(self) -> None:
        """验证 Rally 初始化拒绝无效缩放、压缩时长、超时和周期。"""

        invalid_cases = [
            RallyTaskInitS(looseScale=0.9),
            RallyTaskInitS(compressTime_s=0.0),
            RallyTaskInitS(staleTimeout_s=0.0),
            RallyTaskInitS(dt_s=0.0),
        ]
        for cfg in invalid_cases:
            with self.subTest(cfg=cfg):
                with self.assertRaises(ValueError):
                    Rally().init(cfg)

    def test_remote_none_and_hold_write_expected_command_and_scale(self) -> None:
        """验证 NONE/HOLD 遥控分别输出待命松散缩放和最终保持缩放。"""

        task = _rally_task(expected=())
        ctx = FormContextS()

        _task_step(task, ctx, remote=FormStageE.NONE)
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        self.assertEqual(ctx.cmd.pattern, 0)
        self.assertAlmostEqual(ctx.slotScale.scale, 3.0)

        output = _task_step(task, ctx, remote=FormStageE.HOLD)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(ctx.cmd.pattern, 0)
        self.assertAlmostEqual(ctx.slotScale.scale, 1.0)
        self.assertFalse(output.rallyCompleted)

    def test_remote_standby_keeps_task_out_of_rally_state_machine(self) -> None:
        """验证 STANDBY 遥控不会被 Rally 任务误解释成开始集结。"""

        task = _rally_task(expected=())
        ctx = FormContextS()

        output = _task_step(task, ctx, remote=FormStageE.STANDBY)

        self.assertEqual(ctx.cmd.stage, FormStageE.STANDBY)
        self.assertEqual(ctx.cmd.step, RallyPhaseE.JOINING)
        self.assertAlmostEqual(ctx.slotScale.scale, 1.0)
        self.assertFalse(output.rallyCompleted)

    def test_remote_rally_from_standby_resets_as_first_entry(self) -> None:
        """验证 STANDBY→RALLY 按首次进入集结处理，不沿用待命前残留的子阶段和计时器。"""

        task = _rally_task(expected=("R02",), dt_s=0.1)
        ctx = FormContextS()
        ctx.cmd.stage = FormStageE.STANDBY
        ctx.cmd.step = RallyPhaseE.COMPRESS
        task._stable_timer = 8.0
        task._catchup_stable_timer = 7.0
        task._compress_elapsed = 6.0
        task._t_ref = 123.0

        output = _task_step(task, ctx, remote=FormStageE.RALLY, states=[], now_s=5.0)

        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(ctx.cmd.step, RallyPhaseE.JOINING)
        self.assertAlmostEqual(output.t_ref, 5.0)
        self.assertAlmostEqual(task._stable_timer, 0.0)
        self.assertAlmostEqual(task._catchup_stable_timer, 0.0)
        self.assertAlmostEqual(task._compress_elapsed, 0.0)

    def test_set_pattern_index_changes_hold_output(self) -> None:
        """验证 Rally 完成集结进入 HOLD 后，也能按运行期索引切换目标队形。"""

        task = _rally_task(expected=())
        ctx = FormContextS()

        task.set_pattern_index(1)
        _task_step(task, ctx, remote=FormStageE.HOLD)

        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(ctx.cmd.pattern, 1)
        self.assertAlmostEqual(ctx.slotScale.scale, 1.0)

    def test_approach_requires_all_expected_arrived_and_fresh(self) -> None:
        """验证 JOINING→LOOSE 由全部期望僚机 EXITED 且长机 EXITED 立即推进（无计时器）。"""

        task = _rally_task(expected=("R02", "R03"), dt_s=0.1)
        ctx = FormContextS()
        # rally_state 默认 "EXITED"；leader_join_exited 默认 True
        all_exited = [
            _follower_state("R02", valid=True, last_update_s=0.0, pos_err_m=99.0),
            _follower_state("R03", valid=True, last_update_s=0.0, pos_err_m=99.0),
        ]

        # 全部 EXITED + 长机 EXITED → 首帧即推进到 step=1
        _task_step(task, ctx, remote=FormStageE.RALLY, states=all_exited, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 1)

        # 僚机数据过期且仍在 FLYING（now_s=1.0，lastUpdate_s=0.0，stale_timeout_s=0.5）→ 不推进
        # 注：过期且已 EXITED 不阻塞（EXITED 是终态，见 #12 fix）
        ctx = FormContextS()
        task = _rally_task(expected=("R02",), dt_s=0.1)
        stale = [_follower_state("R02", valid=True, last_update_s=0.0, rally_state="FLYING")]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=stale, now_s=1.0)
        self.assertEqual(ctx.cmd.step, 0)

        # 僚机 FLYING（未切出）→ 不推进
        ctx = FormContextS()
        task = _rally_task(expected=("R02",), dt_s=0.1)
        flying = [_follower_state("R02", valid=True, last_update_s=0.0, rally_state="FLYING")]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=flying, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 0)

        # 长机未 EXITED → 不推进（即使僚机已 EXITED）
        ctx = FormContextS()
        task = _rally_task(expected=("R02",), dt_s=0.1)
        follower_exited = [_follower_state("R02", valid=True, last_update_s=0.0)]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=follower_exited, now_s=0.0, leader_join_exited=False)
        self.assertEqual(ctx.cmd.step, 0)

    def test_exited_follower_not_blocked_by_stale_timeout(self) -> None:
        """已 EXITED 僚机丢链后不应阻塞 JOINING→CATCHUP 门控（EXITED 是终态）。"""
        task = _rally_task(expected=("R02",), dt_s=0.1)
        ctx = FormContextS()
        # 先推进：R02 在 now_s=0.0 时已 EXITED，正常切到 step=1
        exited_state = [_follower_state("R02", valid=True, last_update_s=0.0)]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=exited_state, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 1)

        # 现在 reset 回 step=0，模拟重新进入 JOINING
        ctx = FormContextS()
        task = _rally_task(expected=("R02",), dt_s=0.1)
        # R02 在 t=0 时 EXITED，到 t=10 时数据已过期（stale_timeout_s=0.5）
        stale_exited = [_follower_state("R02", valid=True, last_update_s=0.0, rally_state="EXITED")]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=stale_exited, now_s=10.0)
        # EXITED 是终态，过期不应阻止推进
        self.assertEqual(ctx.cmd.step, 1, "stale EXITED entry should not block JOINING→CATCHUP transition")

    def test_t_ref_stays_invalid_until_all_join_states_are_initialized(self) -> None:
        """验证参与者首个有效 ETA 未收齐时，不发布可用于切出的集结基准时刻。"""
        task = _rally_task(expected=("R02", "R03"))
        ctx = FormContextS()
        uninitialized = [
            _follower_state("R02", rally_state=RALLY_STATE_FLYING, eta_s=20.0),
            _follower_state("R03", rally_state=RALLY_STATE_FLYING, eta_s=0.0),
        ]

        cold_start = _task_step(
            task,
            ctx,
            remote=FormStageE.RALLY,
            states=uninitialized,
            now_s=0.0,
            leader_join_exited=False,
            leader_join_flying=True,
            leader_eta_s=10.0,
        )

        self.assertFalse(cold_start.t_ref_valid)

        uninitialized[1].eta_s = 30.0
        ready = _task_step(
            task,
            ctx,
            remote=FormStageE.RALLY,
            states=uninitialized,
            now_s=0.1,
            leader_join_exited=False,
            leader_join_flying=True,
            leader_eta_s=10.0,
        )

        self.assertTrue(ready.t_ref_valid)
        self.assertAlmostEqual(ready.t_ref, 30.0)

    def test_t_ref_locked_after_last_flyer_departs(self) -> None:
        """T_ref 在最后一架飞机离开 FLYING 后应锁存，而非塌缩为 now_s。"""
        task = _rally_task(expected=("R02",), dt_s=0.1)
        ctx = FormContextS()

        # 拍 1：R02 FLYING，ETA=50s，长机也 FLYING，ETA=40s → t_ref = 50.0
        flying = [_follower_state("R02", rally_state=RALLY_STATE_FLYING, eta_s=50.0, last_update_s=0.0)]
        out1 = _task_step(
            task, ctx, remote=FormStageE.RALLY, states=flying, now_s=0.0,
            leader_join_exited=False, leader_join_flying=True, leader_eta_s=40.0,
        )
        self.assertAlmostEqual(out1.t_ref, 50.0)

        # 拍 2：R02 进入 LOITERING（flying_etas 空），长机也已 EXITED
        loitering = [_follower_state("R02", rally_state=RALLY_STATE_LOITERING, last_update_s=2.0)]
        out2 = _task_step(
            task, ctx, remote=FormStageE.RALLY, states=loitering, now_s=2.0,
            leader_join_exited=False, leader_join_flying=False, leader_eta_s=0.0,
        )
        # t_ref 不应塌缩为 now_s=2.0，应锁存为上一拍的 50.0
        self.assertAlmostEqual(out2.t_ref, 50.0,
            msg="t_ref should remain locked at last valid value, not collapse to now_s")

    def test_cold_start_follower_eta_zero_cannot_overwrite_locked_t_ref(self) -> None:
        """冷启动僚机 FLYING/eta=0 不应在主机报文过期后把已锁存的 T_ref 覆盖为 0。"""
        task = _rally_task(expected=("R02", "R03"), dt_s=0.1)
        ctx = FormContextS()

        # 拍 1：R02 FLYING/eta=50（now=0），R03 FLYING/eta=0 → t_ref 锁存 50.0
        states_tick1 = [
            _follower_state("R02", rally_state=RALLY_STATE_FLYING, eta_s=50.0, last_update_s=0.0),
            _follower_state("R03", rally_state=RALLY_STATE_FLYING, eta_s=0.0,  last_update_s=0.0),
        ]
        out1 = _task_step(task, ctx, remote=FormStageE.RALLY, states=states_tick1, now_s=0.0)
        self.assertAlmostEqual(out1.t_ref, 50.0)

        # 拍 2：R02 数据过期，只剩 R03 FLYING/eta=0
        # 若 eta=0 被计入 flying_etas，max([0.0])=0 会覆盖已锁存的 50.0
        states_tick2 = [
            _follower_state("R02", rally_state=RALLY_STATE_FLYING, eta_s=50.0, last_update_s=0.0),  # stale at now=5
            _follower_state("R03", rally_state=RALLY_STATE_FLYING, eta_s=0.0,  last_update_s=5.0),
        ]
        out2 = _task_step(task, ctx, remote=FormStageE.RALLY, states=states_tick2, now_s=5.0)
        # eta=0 不应进入 flying_etas，锁存的 t_ref 应保持 50.0 不变
        self.assertAlmostEqual(out2.t_ref, 50.0,
            msg="zero-ETA cold-start must not overwrite locked t_ref when valid flyer expires")

    def test_t_ref_counts_loitering_follower_that_has_not_reached_slot_once(self) -> None:
        """回归用例：切入点 T 到 M_i 之间弧长很长时，刚进 LOITERING（尚未首次路过 M_i）的僚机
        ETA 仍应计入 T_ref 聚合，不能因为状态从 FLYING 变成 LOITERING 就被立即剔除，
        否则 T_ref 会塌缩到更快参与者的 ETA，导致同步提前（本次"切线进圆"重构引入的新问题）。"""
        task = _rally_task(expected=("R02", "R03"), dt_s=0.1)
        ctx = FormContextS()

        # R02 刚到切入点 T、进入 LOITERING，但离 M_i 还有约 61s 圆弧（尚未首次路过，reached_slot_once=False）。
        # R03 仍在 FLYING，ETA 更短（30s）。若 R02 被错误剔除，t_ref 会塌缩为 30.0 而不是 63.59。
        states = [
            _follower_state("R02", rally_state=RALLY_STATE_LOITERING, eta_s=63.59, reached_slot_once=False, last_update_s=0.0),
            _follower_state("R03", rally_state=RALLY_STATE_FLYING, eta_s=30.0, last_update_s=0.0),
        ]
        out = _task_step(
            task, ctx, remote=FormStageE.RALLY, states=states, now_s=0.0,
            leader_join_exited=True, leader_join_flying=False, leader_eta_s=0.0,
        )
        self.assertAlmostEqual(out.t_ref, 63.59,
            msg="LOITERING follower that has not reached M_i once must still count toward T_ref")

    def test_t_ref_excludes_loitering_follower_after_reaching_slot_once(self) -> None:
        """验证已经首次路过 M_i、纯粹在盘旋等待的僚机不再计入 T_ref 聚合，
        避免其每圈波动的 ETA（下一次路过所需时间）反复推高/拉低 T_ref。"""
        task = _rally_task(expected=("R02", "R03"), dt_s=0.1)
        ctx = FormContextS()

        # R02 已经路过 M_i 一次，正在等待 T_ref（"下一圈还要飞的时间"，这里故意给一个很大的值验证被排除）。
        # R03 仍在 FLYING，ETA=30s。若 R02 未被排除，max(90, 30)=90 会覆盖掉正确的 30。
        states = [
            _follower_state("R02", rally_state=RALLY_STATE_LOITERING, eta_s=90.0, reached_slot_once=True, last_update_s=0.0),
            _follower_state("R03", rally_state=RALLY_STATE_FLYING, eta_s=30.0, last_update_s=0.0),
        ]
        out = _task_step(
            task, ctx, remote=FormStageE.RALLY, states=states, now_s=0.0,
            leader_join_exited=True, leader_join_flying=False, leader_eta_s=0.0,
        )
        self.assertAlmostEqual(out.t_ref, 30.0,
            msg="LOITERING follower that already reached M_i once must not count toward T_ref")

    def test_loose_and_compress_gate_on_position_error(self) -> None:
        """验证 CATCHUP/LOOSE/COMPRESS 使用位置误差阈值依次推进到 HOLD 并只在转换拍置完成标志。"""

        # catchup_radius_m=3：pos_err>=3 滞留 CATCHUP(1)，pos_err<3 进 LOOSE(2)
        # convergenceRadius_m=5（默认）：pos_err>=5 滞留 LOOSE(2)，pos_err<5 开始计时
        task = _rally_task(
            expected=("R02",), dt_s=0.1, stable_hold_s=0.1,
            compress_time_s=0.2, catchup_radius_m=3.0,
        )
        ctx = FormContextS()
        ok = [_follower_state("R02", pos_err_m=1.0, arrived=1, valid=True, last_update_s=0.0)]
        bad = [_follower_state("R02", pos_err_m=5.0, arrived=1, valid=True, last_update_s=0.0)]

        # JOINING→CATCHUP(1)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 1)

        # CATCHUP(1)：pos_err=5 >= catchup=3 → 滞留 CATCHUP(1)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=bad, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 1)

        # CATCHUP(1)：pos_err=1 < catchup=3 → 进入 LOOSE(2)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 2)

        # LOOSE(2)：pos_err=5 >= loose=5 → 滞留 LOOSE(2)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=bad, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 2)

        # LOOSE(2)：pos_err=1 < loose=5，计时器 dt=0.1 >= stable=0.1 → 进 COMPRESS(3)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.step, 3)

        first_compress = _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertAlmostEqual(ctx.slotScale.scale, 2.0)
        self.assertAlmostEqual(ctx.slotScale.scaleRate, -10.0)
        self.assertFalse(first_compress.rallyCompleted)

        completed = _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertAlmostEqual(ctx.slotScale.scale, 1.0)
        self.assertAlmostEqual(ctx.slotScale.scaleRate, 0.0)
        self.assertTrue(completed.rallyCompleted)

        next_frame = _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertFalse(next_frame.rallyCompleted)

    def test_none_then_rally_allows_restart_after_completed_hold(self) -> None:
        """验证完成后 remote=RALLY 不重启，但先 NONE 再 RALLY 可重新进入 APPROACH。"""

        task = _rally_task(
            expected=("R02",),
            dt_s=0.1,
            stable_hold_s=0.1,
            compress_time_s=0.1,
        )
        ctx = FormContextS()
        ok = [_follower_state("R02", pos_err_m=1.0, arrived=1, valid=True, last_update_s=0.0)]
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)

        _task_step(task, ctx, remote=FormStageE.RALLY, states=ok)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)

        _task_step(task, ctx, remote=FormStageE.NONE)
        _task_step(task, ctx, remote=FormStageE.RALLY, states=[])
        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(ctx.cmd.step, 0)

    def test_non_finite_errors_do_not_advance_rally_gates(self) -> None:
        """验证非有限位置或航向误差不能绕过 CATCHUP/LOOSE 门限。"""

        for field_name in ("posErr_m", "headingErr_rad"):
            for invalid_value in (float("nan"), float("inf"), float("-inf")):
                with self.subTest(stage="CATCHUP", field=field_name, value=invalid_value):
                    task = _rally_task(expected=("R02",), dt_s=0.1, catchup_stable_s=0.1)
                    ctx = FormContextS()
                    ctx.cmd.stage = FormStageE.RALLY
                    ctx.cmd.step = 1
                    state = _follower_state("R02", pos_err_m=1.0, last_update_s=0.0)
                    setattr(state, field_name, invalid_value)

                    _task_step(task, ctx, remote=FormStageE.RALLY, states=[state], now_s=0.0)

                    self.assertEqual(ctx.cmd.step, 1)

        task = _rally_task(expected=("R02",), dt_s=0.1, stable_hold_s=0.1)
        ctx = FormContextS()
        ctx.cmd.stage = FormStageE.RALLY
        ctx.cmd.step = 2
        state = _follower_state("R02", pos_err_m=float("nan"), last_update_s=0.0)

        _task_step(task, ctx, remote=FormStageE.RALLY, states=[state], now_s=0.0)

        self.assertEqual(ctx.cmd.step, 2)


class FollowerBroadcastAndStatusTests(unittest.TestCase):
    """验证僚机回报发送与长机入站解析。"""

    def test_follower_broadcast_targets_leader_and_reports_error(self) -> None:
        """验证回报消息目标、topic、位置误差与到达锁存来自输入端口。"""

        outbound = FollowerBroadcast()
        outbound.init(FollowerBroadcastInitS(selfId="R02", leaderId="R01"))
        output = OutboundOutputS()

        outbound.step(
            FollowerBroadcastInputS(
                selfState=_motion(east=1.0, north=2.0, h=3.0),
                selfCmd=_motion(east=4.0, north=6.0, h=3.0),
                selfArrived=1,
            ),
            output,
        )

        self.assertEqual(len(output.outbox), 1)
        msg = output.outbox[0]
        self.assertEqual(msg.topic, FOLLOWER_STATUS_TOPIC)
        self.assertEqual(msg.source, "R02")
        self.assertEqual(msg.target, "R01")
        self.assertEqual(msg.payload["arrived"], 1)
        self.assertAlmostEqual(msg.payload["pos_err_m"], 5.0)

    def test_follower_broadcast_supports_standby_rally_state_constant(self) -> None:
        """验证待命回报使用统一常量，避免业务代码散落裸字符串。"""

        outbound = FollowerBroadcast()
        outbound.init(FollowerBroadcastInitS(selfId="R02", leaderId="R01"))
        output = OutboundOutputS()

        outbound.step(
            FollowerBroadcastInputS(
                selfState=_motion(east=1.0, north=2.0, h=3.0),
                selfCmd=_motion(east=1.0, north=2.0, h=3.0),
                rally_state=RALLY_STATE_STANDBY,
            ),
            output,
        )

        self.assertEqual(output.outbox[0].payload["rally_state"], RALLY_STATE_STANDBY)

    def test_follower_broadcast_rejects_empty_leader_id_and_missing_ports(self) -> None:
        """验证显式 leaderId 和端口绑定是必需条件。"""

        with self.assertRaises(ValueError):
            FollowerBroadcast().init(FollowerBroadcastInitS(selfId="R02", leaderId=""))

        outbound = FollowerBroadcast()
        outbound.init(FollowerBroadcastInitS(selfId="R02", leaderId="R01"))
        with self.assertRaises(ValueError):
            outbound.step(FollowerBroadcastInputS(selfState=_motion()), OutboundOutputS())

    def test_follower_status_parses_updates_filters_and_uses_source_id(self) -> None:
        """验证长机侧解析、原地更新、过滤非法消息，并以 envelope.source 为准。"""

        states: list[FollowerStateS] = []
        inbound = FollowerStatus()
        inbound.init(None)

        inbound.step(
            FollowerStatusInputS(
                inbox=[
                    _follower_status_msg("R02", pos_east=1.0, pos_north=2.0, pos_h=3.0, pos_err_m=4.0),
                    _follower_status_msg("R03", pos_east=5.0, pos_north=6.0, pos_h=7.0, arrived=1),
                ],
                now_s=10.0,
            ),
            FollowerStatusOutputS(followerStates=states),
        )

        self.assertEqual([state.id for state in states], ["R02", "R03"])
        self.assertTrue(states[0].valid)
        self.assertAlmostEqual(states[0].lastUpdate_s, 10.0)
        self.assertAlmostEqual(states[0].posErr_m, 4.0)
        self.assertEqual(states[1].arrived, 1)

        original = states[0]
        inbound.step(
            FollowerStatusInputS(
                inbox=[
                    MessageEnvelope("node.status", "R99", "R01", 0.0, {"health": "normal"}),
                    MessageEnvelope(FOLLOWER_STATUS_TOPIC, "R04", "R01", 0.0, {"pos_east": 1.0}),
                    _follower_status_msg("R02", pos_east=8.0, pos_err_m=1.5),
                ],
                now_s=11.0,
            ),
            FollowerStatusOutputS(followerStates=states),
        )

        self.assertIs(states[0], original)
        self.assertEqual(len(states), 2)
        self.assertAlmostEqual(states[0].pos.east, 8.0)
        self.assertAlmostEqual(states[0].posErr_m, 1.5)
        self.assertAlmostEqual(states[0].lastUpdate_s, 11.0)

        msg = _follower_status_msg("R05")
        msg.payload["id"] = "伪造ID"
        inbound.step(FollowerStatusInputS(inbox=[msg], now_s=12.0), FollowerStatusOutputS(followerStates=states))
        self.assertEqual(states[-1].id, "R05")

    def test_follower_status_defaults_missing_rally_state_to_legacy_flying(self) -> None:
        """验证旧协议缺省 rally_state 时按 FLYING 兼容，而不是误判为待命。"""

        states: list[FollowerStateS] = []
        inbound = FollowerStatus()
        inbound.init(None)
        msg = MessageEnvelope(
            FOLLOWER_STATUS_TOPIC,
            "R02",
            "R01",
            0.0,
            {
                "pos_east": 1.0,
                "pos_north": 2.0,
                "pos_h": 3.0,
                "pos_err_m": 4.0,
                "eta_s": 12.0,
            },
        )

        inbound.step(FollowerStatusInputS(inbox=[msg], now_s=5.0), FollowerStatusOutputS(followerStates=states))

        self.assertEqual(len(states), 1)
        self.assertEqual(states[0].rally_state, RALLY_STATE_FLYING)

    def test_follower_status_rejects_non_finite_payload_atomically(self) -> None:
        """验证非有限或转换失败的回报不会新增条目，也不会部分覆盖已有状态。"""

        inbound = FollowerStatus()
        inbound.init(None)
        invalid_fields = {
            "pos_east": float("nan"),
            "pos_north": float("inf"),
            "pos_h": float("-inf"),
            "pos_err_m": float("nan"),
            "heading_err_rad": float("inf"),
            "eta_s": float("-inf"),
        }
        for field_name, invalid_value in invalid_fields.items():
            with self.subTest(field=field_name):
                baseline = _follower_state("R02", pos_err_m=4.0, arrived=1, valid=True, last_update_s=10.0)
                baseline.pos = _pos(1.0, 2.0, 3.0)
                baseline.headingErr_rad = 0.2
                baseline.eta_s = 20.0
                states = [baseline]
                msg = _follower_status_msg("R02", pos_east=99.0, pos_err_m=1.0)
                msg.payload[field_name] = invalid_value

                inbound.step(
                    FollowerStatusInputS(inbox=[msg], now_s=11.0),
                    FollowerStatusOutputS(followerStates=states),
                )

                self.assertEqual(states, [baseline])
                self.assertEqual(baseline.pos, _pos(1.0, 2.0, 3.0))
                self.assertEqual(baseline.posErr_m, 4.0)
                self.assertEqual(baseline.lastUpdate_s, 10.0)

        baseline = _follower_state("R02", pos_err_m=4.0, arrived=1, valid=True, last_update_s=10.0)
        baseline.pos = _pos(1.0, 2.0, 3.0)
        malformed = _follower_status_msg("R02", pos_east=99.0)
        malformed.payload["pos_err_m"] = "非法数值"
        states = [baseline]
        inbound.step(
            FollowerStatusInputS(inbox=[malformed], now_s=11.0),
            FollowerStatusOutputS(followerStates=states),
        )
        self.assertEqual(baseline.pos, _pos(1.0, 2.0, 3.0))
        self.assertEqual(baseline.lastUpdate_s, 10.0)

        new_invalid = _follower_status_msg("R03", eta_s=float("inf"))
        inbound.step(
            FollowerStatusInputS(inbox=[new_invalid], now_s=11.0),
            FollowerStatusOutputS(followerStates=states),
        )
        self.assertEqual([state.id for state in states], ["R02"])

    def test_follower_status_requires_bound_output_list(self) -> None:
        """验证输出列表端口未绑定时失败。"""

        with self.assertRaises(ValueError):
            FollowerStatus().step(FollowerStatusInputS(inbox=[]), FollowerStatusOutputS())


class RallyLeaderBroadcastAndInboundTests(unittest.TestCase):
    """验证集结长机广播扩展和僚机解析。"""

    def test_rally_leader_broadcast_adds_slot_scale_and_preserves_contract(self) -> None:
        """验证长机广播保留 leader_state/cmd，并新增 slot_scale。"""

        outbound = RallyLeaderBroadcast()
        outbound.init(OutboundInitS(selfId="R01", netWork=[NetWorkS("R01", "R02", CommDirE.DUPLEX)]))
        output = OutboundOutputS()

        outbound.step(
            RallyLeaderBroadcastInputS(
                cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=2),
                selfState=_motion(east=1.0, north=2.0, h=3.0, v_east=4.0),
                slotScale=RallySlotScaleS(scale=2.0, scaleRate=-0.5),
                t_ref=12.0,
                t_ref_valid=True,
            ),
            output,
        )

        self.assertEqual(len(output.outbox), 1)
        payload = output.outbox[0].payload
        self.assertEqual(output.outbox[0].topic, "formation.leader")
        self.assertIn("leader_state", payload)
        self.assertEqual(payload["cmd"]["step"], 2)
        self.assertEqual(payload["slot_scale"], {"scale": 2.0, "scale_rate": -0.5})
        self.assertEqual(payload["t_ref"], 12.0)
        self.assertTrue(payload["t_ref_valid"])

    def test_rally_leader_follower_parses_cmd_state_and_defaults_missing_scale(self) -> None:
        """验证僚机入站解析同一条消息中的长机状态、编队指令和缩放因子。"""

        ctx = FormContextS()
        inbound = RallyLeaderFollower()
        inbound_output = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
            slotScale=ctx.slotScale,
        )
        inbound.step(
            InboundInputS(
                inbox=[_leader_msg(step=2, scale=2.5, scale_rate=-0.2, t_ref=12.0, t_ref_valid=True)]
            ),
            inbound_output,
        )

        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(ctx.cmd.step, 2)
        self.assertAlmostEqual(ctx.leaderState.pos.east, 100.0)
        self.assertAlmostEqual(ctx.slotScale.scale, 2.5)
        self.assertAlmostEqual(ctx.slotScale.scaleRate, -0.2)
        self.assertAlmostEqual(inbound_output.t_ref, 12.0)
        self.assertTrue(inbound_output.t_ref_valid)

        old_format = _leader_msg()
        del old_format.payload["slot_scale"]
        del old_format.payload["t_ref_valid"]
        old_output = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
            slotScale=ctx.slotScale,
        )
        inbound.step(
            InboundInputS(inbox=[old_format]),
            old_output,
        )
        self.assertEqual(ctx.slotScale, RallySlotScaleS())
        self.assertFalse(old_output.t_ref_valid)

    def test_invalid_t_ref_does_not_commit_partial_cmd_state(self) -> None:
        """t_ref 解析异常时不应提交本条消息中已解析的 cmd.stage/step，避免「新阶段 + 无效 T_ref」半截状态。"""
        ctx = FormContextS()
        inbound = RallyLeaderFollower()
        out = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
            slotScale=ctx.slotScale,
        )
        # 先建立 HOLD 基准状态
        from src.algorithm.context.leaf_types import FormStageE as FSE
        ctx.cmd.stage = FSE.HOLD
        ctx.cmd.step = 0

        # 构造一条 stage=RALLY/step=2 但 t_ref 字段为非法字符串的消息
        bad_t_ref_msg = _leader_msg(stage=FormStageE.RALLY, step=2, t_ref_valid=True)
        bad_t_ref_msg.payload["t_ref"] = "not-a-float"  # type: ignore[index]

        inbound.step(InboundInputS(inbox=[bad_t_ref_msg]), out)

        # t_ref 非法 → 整条消息应被丢弃，cmd 维持 HOLD/step=0 不变
        self.assertEqual(ctx.cmd.stage, FSE.HOLD,
            msg="bad t_ref must not commit cmd.stage change (partial state)")
        self.assertEqual(ctx.cmd.step, 0,
            msg="bad t_ref must not commit cmd.step change (partial state)")
        self.assertFalse(out.t_ref_valid)

    def test_rally_leader_follower_requires_all_output_ports(self) -> None:
        """验证三类输出端口必须同时绑定。"""

        with self.assertRaises(ValueError):
            RallyLeaderFollower().step(InboundInputS(inbox=[]), RallyLeaderFollowerOutputS())


class RallyLooseTargetTests(unittest.TestCase):
    """直接单测 rally_loose_target()：旋转、右侧轴符号、缩放、高度固定差，逐项隔离验证。"""

    def test_pure_forward_offset_at_zero_heading(self) -> None:
        """heading=0（正东）时，纯前向偏置（x 分量）应直接映射为东向偏置，北向不变。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 1.0, FormPosS("R02", 10.0, 5.0, 0.0))
        self.assertAlmostEqual(m_i.east, 10.0)
        self.assertAlmostEqual(m_i.north, 0.0)
        self.assertAlmostEqual(m_i.h, 105.0)

    def test_right_axis_sign_at_zero_heading(self) -> None:
        """heading=0（正东）时，纯右侧偏置（z 分量，正值=右）应映射为负的北向偏置（面向正东时右手边是正南）。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 1.0, FormPosS("R02", 0.0, 0.0, 10.0))
        self.assertAlmostEqual(m_i.east, 0.0)
        self.assertAlmostEqual(m_i.north, -10.0,
            msg="positive slot.z (right, facing east) must map to negative north (south), not positive")

    def test_rotates_forward_offset_with_heading(self) -> None:
        """heading=90°（正北）时，纯前向偏置应旋转成北向偏置，验证旋转矩阵方向而非仅测 heading=0。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), math.pi / 2.0, 1.0, FormPosS("R02", 10.0, 0.0, 0.0))
        self.assertAlmostEqual(m_i.east, 0.0, places=6)
        self.assertAlmostEqual(m_i.north, 10.0, places=6)

    def test_looseScale_multiplies_horizontal_offset_only(self) -> None:
        """looseScale 应线性放大水平偏置（east/north），但高度偏置（slot.y）必须保持固定，不随 scale 扩展。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target

        slot = FormPosS("R02", 10.0, 5.0, 20.0)
        m_i_scale1 = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 1.0, slot)
        m_i_scale3 = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 3.0, slot)

        self.assertAlmostEqual(m_i_scale3.east, 3.0 * m_i_scale1.east)
        self.assertAlmostEqual(m_i_scale3.north, 3.0 * m_i_scale1.north)
        self.assertAlmostEqual(m_i_scale1.h, 105.0)
        self.assertAlmostEqual(m_i_scale3.h, 105.0,
            msg="height offset must stay fixed at slot.y regardless of looseScale")

    def test_route_start_offset_carries_through(self) -> None:
        """集结区起点 A 非原点时，M_i 应在 A 的基础上叠加旋转/缩放后的偏置，而不是忽略 A。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target

        m_i = rally_loose_target(_pos(1000.0, 2000.0, 500.0), 0.0, 2.0, FormPosS("R02", 10.0, 0.0, 0.0))
        self.assertAlmostEqual(m_i.east, 1020.0)
        self.assertAlmostEqual(m_i.north, 2000.0)
        self.assertAlmostEqual(m_i.h, 500.0)


class RallyLoiterSpeedBoundsTests(unittest.TestCase):
    """直接单测 loiter_speed_bounds()：只显式配置 forwardMin/forwardMax 中的一侧时，与另一侧的默认
    兜底值反序的情形必须被显式拒绝，而不是静默产出 min>=max 的非法区间留给下游报 ERR_MODULE_INIT_FAILED。"""

    def test_only_forward_max_configured_below_default_min_rejected(self) -> None:
        """只配 forwardMax=10（< 默认 loiter_min=14）时，(14, 10) 是非法区间，必须拒绝。"""
        from src.algorithm.entity.leader_follower_rally import loiter_speed_bounds

        with self.assertRaises(ValueError):
            loiter_speed_bounds(VelCmdLimitS(forwardMax=10.0))

    def test_only_forward_min_configured_above_default_max_rejected(self) -> None:
        """只配 forwardMin=30（> 默认 loiter_max=25）时，(30, 25) 是非法区间，必须拒绝。"""
        from src.algorithm.entity.leader_follower_rally import loiter_speed_bounds

        with self.assertRaises(ValueError):
            loiter_speed_bounds(VelCmdLimitS(forwardMin=30.0))

    def test_both_unconfigured_uses_valid_defaults(self) -> None:
        """两侧都不配置时退回默认 (14, 25)，本身自洽，不应报错。"""
        from src.algorithm.entity.leader_follower_rally import loiter_speed_bounds

        loiter_min, loiter_max = loiter_speed_bounds(VelCmdLimitS())
        self.assertEqual((loiter_min, loiter_max), (14.0, 25.0))

    def test_both_explicitly_configured_and_consistent_passes_through(self) -> None:
        """两侧都显式配置且自洽时，原样透传，不受默认值影响。"""
        from src.algorithm.entity.leader_follower_rally import loiter_speed_bounds

        loiter_min, loiter_max = loiter_speed_bounds(VelCmdLimitS(forwardMin=18.0, forwardMax=22.0))
        self.assertEqual((loiter_min, loiter_max), (18.0, 22.0))


class RallyPosCalcTests(unittest.TestCase):
    """验证集结专用位置解算单元。"""

    def test_rally_join_pos_standby_outputs_ccw_local_loiter_command(self) -> None:
        """验证 STANDBY 是 RallyJoinPos 内部状态，输出本地圆盘旋目标。"""

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(40.0, 5.0, 560.0),
            approach_speed_mps=20.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
            mission_speed_mps=20.0,
            control_period_s=0.05,
            standby_altitude_m=560.0,
        ))
        output = PosCalcOutputS(selfCmd=MotionProfS())

        join.step(
            RallyJoinPosInputS(
                selfState=_motion(east=-50.0, north=20.0, h=500.0, v_east=20.0, v_psi=0.0),
                standby=True,
            ),
            output,
        )

        assert output.selfCmd is not None
        self.assertEqual(join.state, RALLY_STATE_STANDBY)
        self.assertAlmostEqual(output.selfCmd.pos.east, -50.0)
        self.assertAlmostEqual(output.selfCmd.pos.north, 20.0)
        self.assertAlmostEqual(output.selfCmd.pos.h, 560.0)
        self.assertAlmostEqual(output.selfCmd.v.vEast, 20.0)
        self.assertAlmostEqual(output.selfCmd.v.vNorth, 0.0, places=6)
        self.assertAlmostEqual(output.selfCmd.v.vPsi, 0.0, places=6)
        self.assertAlmostEqual(output.selfCmd.v.dVPsi, 0.1)
        self.assertAlmostEqual(output.selfCmd.v.vUp, 3.0)

        join.step(
            RallyJoinPosInputS(
                selfState=_motion(east=-50.0, north=20.0, h=500.0, v_east=20.0, v_psi=0.0),
                standby=False,
                t_now=1.0,
            ),
            PosCalcOutputS(selfCmd=MotionProfS()),
        )

        self.assertEqual(join.state, RALLY_STATE_FLYING)

    def test_rally_join_pos_standby_clamps_speed_to_loiter_bounds(self) -> None:
        """验证待命盘旋速度以当前地速为参考，但必须夹到盘旋速度上下限内。"""

        for initial_speed_mps, expected_speed_mps in ((5.0, 14.0), (60.0, 25.0)):
            with self.subTest(initial_speed_mps=initial_speed_mps):
                join = RallyJoinPos()
                join.init(RallyJoinPosInitS(
                    loose_slot=_pos(40.0, 5.0, 560.0),
                    approach_speed_mps=20.0,
                    loiter_radius_m=200.0,
                    loiter_speed_min_mps=14.0,
                    loiter_speed_max_mps=25.0,
                    mission_heading_rad=0.0,
                    mission_speed_mps=20.0,
                    control_period_s=0.05,
                ))
                output = PosCalcOutputS(selfCmd=MotionProfS())

                join.step(
                    RallyJoinPosInputS(
                        selfState=_motion(
                            east=-50.0,
                            north=20.0,
                            h=500.0,
                            v_east=initial_speed_mps,
                            vd=initial_speed_mps,
                            v_psi=0.0,
                        ),
                        standby=True,
                    ),
                    output,
                )

                assert output.selfCmd is not None
                self.assertAlmostEqual(output.selfCmd.v.vd, expected_speed_mps)
                self.assertAlmostEqual(output.selfCmd.v.dVPsi, expected_speed_mps / 200.0)

    def test_slot_geometry_scales_position_and_adds_compress_feedforward(self) -> None:
        """验证槽位偏置缩放和 scaleRate 速度前馈。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
        ctx.selfState = _motion(east=80.0, north=210.0, h=500.0, v_east=20.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=2)
        ctx.slotScale = RallySlotScaleS(scale=2.0, scaleRate=-0.5)
        comm_init = _comm_init()
        slot = SlotGeometry()
        slot.init(SlotGeometryInitS("R02", comm_init.formPat, comm_init.formPos))

        slot.step(
            SlotGeometryInputS(
                selfState=ctx.selfState,
                leaderState=ctx.leaderState,
                cmd=ctx.cmd,
                slotScale=ctx.slotScale,
            ),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 80.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 210.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 500.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 25.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, -2.5)

    def test_slot_geometry_altitude_fixed_not_scaled(self) -> None:
        """验证高度偏置不随 scale 扩展：slot.y=30 时，h = leader.h + 30，与 scale 无关。"""

        comm_init_alt = FormCommInitS(
            netWork=[
                NetWorkS("R01", "R02", CommDirE.DUPLEX),
                NetWorkS("R01", "R03", CommDirE.DUPLEX),
            ],
            formPat=["TRIANGLE"],
            formPos=[
                [
                    FormPosS("R01", 0.0, 0.0, 0.0),
                    FormPosS("R02", -10.0, 30.0, -5.0),  # slot.y=30 高度偏置
                    FormPosS("R03", -10.0, 0.0, 5.0),
                ]
            ],
        )

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
        ctx.selfState = _motion(east=80.0, north=210.0, h=530.0, v_east=20.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=2)
        ctx.slotScale = RallySlotScaleS(scale=2.0, scaleRate=-0.5)
        slot = SlotGeometry()
        slot.init(SlotGeometryInitS("R02", comm_init_alt.formPat, comm_init_alt.formPos))

        slot.step(
            SlotGeometryInputS(
                selfState=ctx.selfState,
                leaderState=ctx.leaderState,
                cmd=ctx.cmd,
                slotScale=ctx.slotScale,
            ),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        # 高度固定差：leader.h + slot.y = 500 + 30 = 530（不随 scale=2 变化，非 500+2*30=560）
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 530.0,
            msg="altitude must equal leader.h + slot.y regardless of scale")
        # 水平位置仍按 scale 扩展
        self.assertAlmostEqual(ctx.selfCmd.pos.east, 80.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 210.0)

    def test_slot_geometry_without_slot_scale_uses_unscaled_slot(self) -> None:
        """验证 slotScale 端口未绑定时按未缩放槽位工作，兼容普通 SlotGeometry 调用方式。"""

        comm_init = _comm_init()
        slot = SlotGeometry()
        out = PosCalcOutputS(selfCmd=MotionProfS())
        slot.init(SlotGeometryInitS("R02", comm_init.formPat, comm_init.formPos))
        slot.step(
            SlotGeometryInputS(
                selfState=_motion(),
                leaderState=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0),
                cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=0),
            ),
            out,
        )

        self.assertAlmostEqual(out.selfCmd.pos.east, 90.0)
        self.assertAlmostEqual(out.selfCmd.pos.north, 205.0)
        self.assertAlmostEqual(out.selfCmd.pos.h, 500.0)

    def test_rally_join_pos_entry_point_lies_on_loiter_circle(self) -> None:
        """验证 FLYING 阶段算出的切入点 T 落在盘旋圆上，且不再等于 init.loose_slot。"""

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=60.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
        ))
        out = PosCalcOutputS(selfCmd=MotionProfS())
        join.step(RallyJoinPosInputS(selfState=_motion(east=2000.0, north=300.0, h=500.0), t_ref_valid=False), out)

        entry = join._entry_point  # 白盒检查：切入点应已算出并固定
        self.assertIsNotNone(entry)
        dist_to_center = math.hypot(entry.east - join._loiter_center_e, entry.north - join._loiter_center_n)
        self.assertAlmostEqual(dist_to_center, 200.0, places=3,
            msg="entry point must lie exactly on the loiter circle")
        self.assertNotAlmostEqual(entry.east, 0.0, places=0,
            msg="entry point should generally differ from loose_slot A")

    def _flying_to_loitering_heading_jump_deg(self, *, loiter_radius_m: float, arrival_radius_m: float) -> float:
        """驱动 RallyJoinPos 从远处直飞到切入点，返回 FLYING→LOITERING 切换瞬间的指令航向跳变（度）。"""
        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=arrival_radius_m,
            loiter_radius_m=loiter_radius_m,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
            mission_speed_mps=20.0,
        ))
        state = _motion(east=-2000.0, north=300.0, h=500.0)
        out = PosCalcOutputS(selfCmd=MotionProfS())
        t_now = 0.0
        dt = 0.05
        prev_heading = None
        for _ in range(80000):
            join.step(RallyJoinPosInputS(selfState=state, t_ref=1e9, t_ref_valid=False, t_now=t_now), out)
            heading = math.atan2(out.selfCmd.v.vNorth, out.selfCmd.v.vEast)
            if join.state == RALLY_STATE_LOITERING:
                return abs(math.degrees(math.atan2(math.sin(heading - prev_heading), math.cos(heading - prev_heading))))
            prev_heading = heading
            state = _motion(
                east=state.pos.east + out.selfCmd.v.vEast * dt,
                north=state.pos.north + out.selfCmd.v.vNorth * dt,
                h=out.selfCmd.pos.h,
            )
            t_now += dt
        self.fail("did not reach LOITERING within the simulated step budget")
        return 0.0  # unreachable, keeps type-checkers happy

    def test_rally_join_pos_flying_to_loitering_transition_heading_jump_is_small(self) -> None:
        """回归用例：即便 `arrival_radius_m` 配置得较大，FLYING→LOITERING 切换瞬间的指令航向跳变
        也应被压到一个较小的量级（不能是配置多大跳变就多大）——T 是圆上固定点，触发半径越大，
        LOITERING 按飞机此刻实际角度算出的切向航向跟 FLYING 直飞航向（按 T 处角度算）差得越多，
        实测默认 100m 时能到 ~26°；应被内部按 loiter_radius_m 反解夹到较小触发半径，压到几度以内。"""
        jump_deg = self._flying_to_loitering_heading_jump_deg(loiter_radius_m=200.0, arrival_radius_m=100.0)
        self.assertLess(jump_deg, 10.0,
            msg="FLYING->LOITERING command heading jump must stay small regardless of arrival_radius_m")

    def test_rally_join_pos_heading_jump_bound_holds_across_loiter_radii(self) -> None:
        """回归用例：跳变角上限是按 loiter_radius_m 反解触发半径得到的（ψ=atan(d/R)），不能是固定距离
        上限——固定距离在小半径下换算出的角度会远超预期（实测固定 15m 时 R=10m 能到 ~56°）。
        验证合法范围内（不小于 init 校验的下限）不同 loiter_radius_m 下跳变角都保持在较小量级，
        不随 R 变小而显著放大。"""
        for radius in (200.0, 100.0, 50.0):
            jump_deg = self._flying_to_loitering_heading_jump_deg(loiter_radius_m=radius, arrival_radius_m=100.0)
            self.assertLess(jump_deg, 8.0,
                msg=f"heading jump bound must hold for loiter_radius_m={radius}, got {jump_deg:.2f} deg")

    def test_rally_join_pos_rejects_loiter_radius_too_small_for_capture_window(self) -> None:
        """回归用例：loiter_radius_m 太小时，按跳变角上限反解出的触发半径会被地板值/离散步进距离压过，
        init 应显式拒绝，而不是静默产出一个远超 5° 承诺的跳变角，或让飞机整拍跨过捕获窗口错过切入。
        实测固定 15m 触发半径上限时 R=10m 能到 ~56° 跳变，属于典型的"太小"场景。"""
        with self.assertRaises(ValueError):
            RallyJoinPos().init(RallyJoinPosInitS(
                loose_slot=_pos(0.0, 0.0, 500.0),
                approach_speed_mps=20.0,
                arrival_radius_m=100.0,
                loiter_radius_m=10.0,
                loiter_speed_min_mps=14.0,
                loiter_speed_max_mps=25.0,
                mission_heading_rad=0.0,
                mission_speed_mps=20.0,
                control_period_s=0.05,
            ))

    def test_rally_join_pos_rejects_arrival_radius_too_small_even_with_valid_loiter_radius(self) -> None:
        """回归用例：即便 loiter_radius_m 本身合法，运行时实际生效的触发半径是
        min(arrival_radius_m, arc_capture_radius_m)——arrival_radius_m 配得比它还小时，
        飞机可能整拍跨过这个更窄的窗口，冻结在离 T 一点点但始终不够近的位置，永远进不了 LOITERING。
        复现过：speed=20m/s、周期0.05s、R=200m、arrival_radius_m=0.1m 时，大多数起点会永久卡在 FLYING。"""
        with self.assertRaises(ValueError):
            RallyJoinPos().init(RallyJoinPosInitS(
                loose_slot=_pos(0.0, 0.0, 500.0),
                approach_speed_mps=20.0,
                arrival_radius_m=0.1,  # 合法的 loiter_radius_m=200 反解出的触发半径远大于此
                loiter_radius_m=200.0,
                loiter_speed_min_mps=14.0,
                loiter_speed_max_mps=25.0,
                mission_heading_rad=0.0,
                mission_speed_mps=20.0,
                control_period_s=0.05,
            ))

    def test_rally_join_pos_capture_window_uses_worst_case_of_approach_and_loiter_min_speed(self) -> None:
        """回归用例：捕获窗口的安全边界要用 FLYING 阶段可能达到的最大速度校验——近场按 slow_radius_m
        减速时的地板是 loiter_speed_min_mps，若它比 approach_speed_mps 还大，真实步进距离由它决定，
        只按 approach_speed_mps 算会低估最坏情况，遗漏本该拒绝的配置。"""
        # approach_speed_mps 很小、loiter_speed_min_mps 很大：真实步进距离由后者决定，
        # 若校验只看 approach_speed_mps 会误判这组参数合法。
        with self.assertRaises(ValueError):
            RallyJoinPos().init(RallyJoinPosInitS(
                loose_slot=_pos(0.0, 0.0, 500.0),
                approach_speed_mps=5.0,
                arrival_radius_m=100.0,
                loiter_radius_m=40.0,  # 按 approach_speed_mps=5 算勉强合法，但按 loiter_speed_min_mps=40 算不够
                loiter_speed_min_mps=40.0,
                loiter_speed_max_mps=45.0,
                mission_heading_rad=0.0,
                mission_speed_mps=20.0,
                control_period_s=0.05,
            ))

    def test_rally_join_pos_exit_heading_matches_mission_heading_from_opposite_arrival(self) -> None:
        """回归用例：即便从任务航向的正对侧飞来（旧版会导致切出反向），切出速度方向仍须对齐任务航向。"""

        mission_heading = 0.0  # 任务航向正东
        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=60.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=mission_heading,
            mission_speed_mps=20.0,
        ))

        # 起点在 A 的正东侧、需要一路向西飞才能到达——旧版会让盘旋圆按到达航向（向西）摆歪，
        # 切出瞬间指令方向与任务航向（正东）相差近 180°。
        state = _motion(east=2000.0, north=300.0, h=500.0)
        out = PosCalcOutputS(selfCmd=MotionProfS())
        t_now = 0.0
        dt = 0.1
        for _ in range(20000):
            join.step(RallyJoinPosInputS(selfState=state, t_ref=0.0, t_ref_valid=True, t_now=t_now), out)
            if join.state == RALLY_STATE_EXITED:
                break
            # 简化运动学：假设速度指令被完美跟踪，只推进位置，不引入动力学误差。
            state = _motion(
                east=state.pos.east + out.selfCmd.v.vEast * dt,
                north=state.pos.north + out.selfCmd.v.vNorth * dt,
                h=out.selfCmd.pos.h,
            )
            t_now += dt
        else:
            self.fail("RallyJoinPos did not reach EXITED within the simulated step budget")

        exit_heading = math.atan2(out.selfCmd.v.vNorth, out.selfCmd.v.vEast)
        self.assertAlmostEqual(exit_heading, mission_heading, delta=1e-6,
            msg="exit velocity direction must equal mission heading regardless of arrival direction")
        self.assertGreater(out.selfCmd.v.vEast, 0.0,
            msg="exit velocity must point east (mission heading), not backward toward the arrival side")

    def test_rally_join_pos_does_not_exit_immediately_when_entry_point_lands_just_past_slot_angle(self) -> None:
        """回归用例：切入点 T 弦长离 M_i 很近，但沿 CCW 方向其实在 M_i"之后"（需绕行近一整圈）时，
        不能被对称弧距 ang_dist 误判成"已到达"而在没有真正飞完圆弧的情况下立即切出。"""

        mission_heading = 0.0  # 任务航向正东
        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=60.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=mission_heading,
            mission_speed_mps=20.0,
        ))

        # 该起点对应的切入点 T ≈ (34.7, 3.0)，与 M_i=(0,0) 弦长仅约 35m，
        # 但按 CCW 方向的真实弧长约 350°（T 在角度上刚"越过"M_i，而非即将到达）。
        state = _motion(east=-950.08, north=-170.61, h=500.0)
        out = PosCalcOutputS(selfCmd=MotionProfS())
        t_now = 0.0
        dt = 0.1
        for _ in range(20000):
            join.step(RallyJoinPosInputS(selfState=state, t_ref=0.0, t_ref_valid=True, t_now=t_now), out)
            if join.state == RALLY_STATE_EXITED:
                break
            state = _motion(
                east=state.pos.east + out.selfCmd.v.vEast * dt,
                north=state.pos.north + out.selfCmd.v.vNorth * dt,
                h=out.selfCmd.pos.h,
            )
            t_now += dt
        else:
            self.fail("RallyJoinPos did not reach EXITED within the simulated step budget")

        # 直飞到切入点约需 47s；若在此刻附近就切出，说明没有真正沿圆弧飞行（复现了指令方向大跳变的 bug）。
        # 正确行为需要另外飞完约 350° 弧长（半径 200m、最大速度 25m/s 时一圈约 50s），
        # 因此切出时刻应显著晚于"刚到切入点"的时间。
        self.assertGreater(t_now, 80.0,
            msg="must not exit right after reaching the tangent entry point when its arc angle to M_i is ~350°, not ~10°")

    def test_rally_join_pos_reached_slot_once_stays_false_when_entry_point_lands_just_past_slot_angle(self) -> None:
        """回归用例：同一个"切入点弦长近、真实弧长约 350°"场景下，进 LOITERING 那一拍
        reached_slot_once 必须仍是 False（不能被对称弧距 ang_dist 误判成"已到达"，
        否则会在 T_ref 聚合里被过早剔除，复现"到达 T 就退出 T_ref 聚合"的问题）。"""

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=100.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
            mission_speed_mps=20.0,
        ))
        # t_ref_valid=False：只观察进 LOITERING 那一拍的 reached_slot_once，不触发切出评估。
        state = _motion(east=-950.08, north=-170.61, h=500.0)
        out = PosCalcOutputS(selfCmd=MotionProfS())
        t_now = 0.0
        dt = 0.1
        for _ in range(20000):
            join.step(RallyJoinPosInputS(selfState=state, t_ref_valid=False, t_now=t_now), out)
            if join.state == RALLY_STATE_LOITERING:
                self.assertFalse(join.reached_slot_once,
                    msg="entering LOITERING via a tangent point far from M_i (in true arc-length terms) "
                        "must not immediately flip reached_slot_once to True")
                return
            state = _motion(
                east=state.pos.east + out.selfCmd.v.vEast * dt,
                north=state.pos.north + out.selfCmd.v.vNorth * dt,
                h=out.selfCmd.pos.h,
            )
            t_now += dt
        self.fail("did not reach LOITERING within the simulated step budget")

    def test_rally_join_pos_loitering_targets_nominal_radius_not_actual(self) -> None:
        """回归用例：LOITERING 阶段的位置/前馈指令必须以期望半径 loiter_radius_m 为准，而不是飞机此刻的实际半径。

        注：这里只断言"指令算对了没有"（几何上确定、不依赖闭环收敛），不模拟完整动力学闭环——
        实际半径能否收敛到期望值取决于 PidCompose/飞行器动力学，已用真实仿真单独跑过验证收敛到
        ~200~202m（期望 200m）；此处只保证 RallyJoinPos 这一层给出的指令本身是"奔着期望半径去的"。
        """

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=60.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
            mission_speed_mps=20.0,
        ))
        out = PosCalcOutputS(selfCmd=MotionProfS())

        # 让飞机"实际半径"明显偏离期望值（150m，不是 200m），验证指令仍然对齐期望半径而不是跟着实际半径走。
        # 直接从 FLYING 步进到接近切入点，再手动摆一个偏离期望半径的位置进入 LOITERING 的第一拍。
        join.step(RallyJoinPosInputS(selfState=_motion(east=-2000.0, north=0.0, h=500.0), t_ref_valid=False), out)
        # 强制切到 LOITERING，模拟飞机当前实际半径为 150m（偏离期望的 200m）而非精确在切入点上。
        join._state = RALLY_STATE_LOITERING
        join._away_from_slot = True
        theta = math.radians(30.0)
        actual_radius = 150.0
        pos_e = join._loiter_center_e + actual_radius * math.cos(theta)
        pos_n = join._loiter_center_n + actual_radius * math.sin(theta)
        join.step(RallyJoinPosInputS(selfState=_motion(east=pos_e, north=pos_n, h=500.0), t_ref_valid=False), out)

        cmd_dist_to_center = math.hypot(
            out.selfCmd.pos.east - join._loiter_center_e,
            out.selfCmd.pos.north - join._loiter_center_n,
        )
        self.assertAlmostEqual(cmd_dist_to_center, 200.0, places=6,
            msg="position command must sit on the nominal-radius circle, not the aircraft's current (actual) radius")
        self.assertAlmostEqual(out.selfCmd.v.dVPsi, join._loiter_speed / 200.0, places=6,
            msg="centripetal feedforward must use the nominal radius, not the aircraft's current (actual) radius")

    def test_rally_join_pos_falls_back_to_direct_flight_when_starting_inside_circle(self) -> None:
        """已知限制：起点落在盘旋圆内部时无切线可求，应退化为直飞 loose_slot 而不是抛异常。"""

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(10.0, 20.0, 500.0),
            approach_speed_mps=20.0,
            arrival_radius_m=60.0,
            loiter_radius_m=200.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
            mission_heading_rad=0.0,
        ))
        out = PosCalcOutputS(selfCmd=MotionProfS())
        # 起点就是 loose_slot 本身（必然落在盘旋圆上/圆内），验证不抛异常且退化为直飞该点。
        join.step(RallyJoinPosInputS(selfState=_motion(east=10.0, north=20.0, h=500.0), t_ref_valid=False), out)

        self.assertEqual(join._entry_point.east, 10.0)
        self.assertEqual(join._entry_point.north, 20.0)


class RallyEntityTests(unittest.TestCase):
    """验证集结长机和僚机实体的主链路。"""

    def test_rally_follower_standby_loiters_and_broadcasts_from_entity_layer(self) -> None:
        """验证僚机实体在 STANDBY 阶段自行本地盘旋，并继续发送待命回报。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
                rally_approach_speed_mps=20.0,
                rally_layer_altitude_m=560.0,
            )
        )
        output = EntityOutputS()

        follower.step(
            EntityInputS(
                selfState=_motion(east=-50.0, north=20.0, h=500.0, v_east=20.0, v_psi=0.0),
                inbox=[_leader_msg(stage=FormStageE.RALLY, step=0)],
                remote=RemoteCmdS(FormStageE.STANDBY),
                now_s=0.0,
            ),
            output,
        )

        assert output.selfCmd is not None
        assert output.selfAccCmd is not None
        self.assertEqual(follower.cxt.cmd.stage, FormStageE.STANDBY)
        self.assertEqual(follower._rally_join.state, RALLY_STATE_STANDBY)
        self.assertFalse(hasattr(follower, "_pos_calc_standby"))
        self.assertFalse(hasattr(follower, "_standby_u"))
        self.assertAlmostEqual(follower.cxt.leaderState.pos.east, 100.0)
        self.assertAlmostEqual(output.selfCmd.pos.h, 560.0)
        self.assertGreater(math.hypot(output.selfCmd.v.vEast, output.selfCmd.v.vNorth), 1.0)
        self.assertGreater(abs(output.selfAccCmd.accEast) + abs(output.selfAccCmd.accNorth), 0.01)
        self.assertEqual(output.outbox[0].payload["rally_state"], RALLY_STATE_STANDBY)
        self.assertEqual(output.outbox[0].payload["arrived"], 0)

    def test_rally_follower_standby_keeps_subject_flow_slots(self) -> None:
        """验证僚机待命不在实体主体流程早退，仍经过轨迹规划槽位后再进入位置解算。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        tra_plan_calls: list[FormStageE] = []
        original_step = follower._tra_plan.step

        def record_tra_plan_step(u: object, y: object) -> None:
            """记录待命阶段是否进入轨迹规划槽位。"""

            tra_plan_calls.append(follower.cxt.cmd.stage)
            original_step(u, y)

        follower._tra_plan.step = record_tra_plan_step  # type: ignore[method-assign]

        follower.step(
            EntityInputS(
                selfState=_motion(east=-50.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(stage=FormStageE.RALLY, step=0)],
                remote=RemoteCmdS(FormStageE.STANDBY),
            ),
            EntityOutputS(),
        )

        self.assertEqual(tra_plan_calls, [FormStageE.STANDBY])
        self.assertEqual(follower._rally_join.state, RALLY_STATE_STANDBY)

    def test_rally_follower_catchup_keeps_layered_altitude_before_loose(self) -> None:
        """验证 CATCHUP 阶段仍保持分层高度，直到进入 LOOSE 后再收敛到正常槽位高度。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
                rally_layer_altitude_m=560.0,
            )
        )

        catchup = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=1, scale=3.0, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            catchup,
        )
        assert catchup.selfCmd is not None
        catchup_height_m = catchup.selfCmd.pos.h
        loose = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=2, scale=3.0, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            loose,
        )

        assert loose.selfCmd is not None
        self.assertAlmostEqual(catchup_height_m, 560.0)
        self.assertAlmostEqual(loose.selfCmd.pos.h, 500.0)

    def test_rally_follower_latches_arrival_and_switches_to_slot_scale_after_step_one(self) -> None:
        """验证僚机到达松散点后进入 EXITED 上报，并在长机进入 LOOSE/COMPRESS 后使用缩放槽位。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )

        first = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=0, scale=3.0)],
                now_s=0.0,
            ),
            first,
        )
        # 僚机位于目标点且 T_ref 已有效 → RallyJoinPos 进入 EXITED，上报 arrived=1
        self.assertEqual(follower._rally_join.state, RALLY_STATE_EXITED)
        self.assertEqual(first.outbox[0].payload["arrived"], 1)
        self.assertEqual(first.outbox[0].payload["rally_state"], RALLY_STATE_EXITED)

        second = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=490.0, v_east=20.0),
                inbox=[_leader_msg(step=1, scale=2.0, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            second,
        )

        assert second.selfCmd is not None
        # CATCHUP 阶段直接复用 SlotGeometry 的缩放能力：R02 槽位偏置 (x=-10,z=-5)，长机航迹东向，
        # 未缩放偏置投影为 (east=-10, north=5)；scale=2.0 → 真实槽位 = leader + scale*offset
        # = (100-20, 200+10) = (80, 210)。
        self.assertAlmostEqual(second.selfCmd.pos.east, 80.0)
        self.assertAlmostEqual(second.selfCmd.pos.north, 210.0)
        # 长机沿东向以 20 m/s 直飞（无偏航角速率），槽位只透传自身速度前馈；
        # CATCHUP 不得再按位置误差额外调速，追赶速度修正由 PidCompose 前向外环生成。
        self.assertAlmostEqual(second.selfCmd.v.vEast, 20.0)
        self.assertAlmostEqual(second.selfCmd.v.vd, 20.0)
        self.assertAlmostEqual(second.selfCmd.v.vPsi, 0.0)
        # CATCHUP 门控统一使用到真实槽位的三维距离，高度误差也必须计入 pos_err_m。
        self.assertAlmostEqual(second.outbox[0].payload["pos_err_m"], math.sqrt(70.0**2 + 190.0**2 + 10.0**2))
        self.assertEqual(second.outbox[0].target, "R01")

    def test_rally_follower_waits_when_t_ref_is_not_valid_at_cold_start(self) -> None:
        """验证冷启动尚无有效 T_ref 时，已到目标点的僚机进入盘旋而不是直接切出。"""
        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        output = EntityOutputS()

        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=0, t_ref=0.0, t_ref_valid=False)],
                now_s=0.0,
            ),
            output,
        )

        self.assertEqual(follower._rally_join.state, RALLY_STATE_LOITERING)
        self.assertEqual(output.outbox[0].payload["arrived"], 0)

    def test_rally_follower_none_resets_join_state_for_restart(self) -> None:
        """验证僚机退出到 NONE 时清除 EXITED 锁存，下一轮可重新执行 JOINING。"""
        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        state = _motion(east=10.0, north=20.0, h=500.0, v_east=20.0)
        follower.step(
            EntityInputS(selfState=state, inbox=[_leader_msg(t_ref_valid=True)]),
            EntityOutputS(),
        )
        self.assertEqual(follower._rally_join.state, RALLY_STATE_EXITED)

        none_output = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[_leader_msg(stage=FormStageE.NONE, pattern=0)],
            ),
            none_output,
        )

        self.assertEqual(follower._rally_join.state, RALLY_STATE_FLYING)
        self.assertEqual(none_output.outbox[0].payload["arrived"], 0)

    def test_rally_follower_none_outputs_current_position_zero_velocity(self) -> None:
        """验证僚机收到 NONE 时输出当前位置零速并清到达上报。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        output = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=1.0, north=2.0, h=3.0, v_east=20.0),
                inbox=[_leader_msg(stage=FormStageE.NONE, pattern=0)],
            ),
            output,
        )

        assert output.selfCmd is not None
        self.assertEqual(output.selfCmd.pos, PosInEarthS(1.0, 2.0, 3.0))
        self.assertEqual(output.selfCmd.v, VdInEarthS())
        self.assertEqual(output.outbox[0].payload["arrived"], 0)

    def test_rally_leader_completes_and_outputs_formation_analysis_once(self) -> None:
        """验证集结长机完成转换首帧输出编队分析，后续帧不重复输出。"""

        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((100.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",), dt_s=0.1),
            )
        )
        # 本用例只验证完成分析输出，直接把汇合子状态置为已切出，避免固定位置夹具无法模拟完整盘旋轨迹。
        leader._rally_join._state = RALLY_STATE_EXITED
        status = _follower_status_msg("R02", pos_err_m=1.0, arrived=1)

        outputs: list[EntityOutputS] = []
        for now_s in (0.0, 0.1, 0.2, 0.3, 0.4):
            output = EntityOutputS()
            leader.step(
                EntityInputS(
                    selfState=_motion(east=100.0, north=0.0, h=500.0, v_east=20.0),
                    inbox=[status],
                    remote=RemoteCmdS(FormStageE.RALLY),
                    now_s=now_s,
                ),
                output,
            )
            outputs.append(output)

        analysis_frames = [item.formationAnalysis for item in outputs if item.formationAnalysis is not None]
        self.assertEqual(len(analysis_frames), 1)
        analysis = analysis_frames[0]
        self.assertAlmostEqual(analysis.posErrMax_m, 1.0)
        self.assertAlmostEqual(analysis.posErrRms_m, 1.0)
        self.assertEqual(analysis.inPositionCount, 1)
        self.assertEqual(analysis.totalCount, 1)

    def test_rally_leader_none_resets_join_and_completion_latches(self) -> None:
        """验证长机退出到 NONE 时复位汇合状态和完成锁存，允许重新集结。"""
        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((100.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
            )
        )
        leader.cxt.cmd.stage = FormStageE.HOLD
        leader._rally_join._state = RALLY_STATE_EXITED
        leader._rally_completed = True

        leader.step(
            EntityInputS(
                selfState=_motion(east=100.0, north=0.0, h=500.0, v_east=20.0),
                remote=RemoteCmdS(FormStageE.NONE),
                now_s=1.0,
            ),
            EntityOutputS(),
        )

        self.assertEqual(leader._rally_join.state, RALLY_STATE_FLYING)
        self.assertFalse(leader._rally_completed)

    def test_rally_leader_standby_parses_follower_status_and_broadcasts(self) -> None:
        """验证长机待命阶段正常解析僚机回报，并继续广播 STANDBY 阶段。"""

        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((100.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
            )
        )

        output = EntityOutputS()
        leader.step(
            EntityInputS(
                selfState=_motion(east=0.0, north=0.0, h=500.0, v_east=20.0),
                inbox=[_follower_status_msg("R02", pos_err_m=1.0, arrived=1)],
                remote=RemoteCmdS(FormStageE.STANDBY),
                now_s=1.0,
            ),
            output,
        )

        self.assertEqual(leader.cxt.cmd.stage, FormStageE.STANDBY)
        self.assertEqual(leader._rally_join.state, RALLY_STATE_STANDBY)
        self.assertFalse(hasattr(leader, "_pos_calc_standby"))
        self.assertFalse(hasattr(leader, "_standby_u"))
        self.assertEqual([state.id for state in leader.cxt.followerStates], ["R02"])
        self.assertEqual(output.outbox[0].topic, "formation.leader")
        self.assertEqual(output.outbox[0].payload["cmd"]["stage"], int(FormStageE.STANDBY))

    def test_rally_leader_standby_does_not_clear_entity_state_before_flow(self) -> None:
        """验证 STANDBY 不在实体主体流程前清理已有状态，清理职责不属于待命位置解算。"""

        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((100.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
            )
        )
        leader.cxt.cmd.stage = FormStageE.RALLY
        leader.cxt.followerStates.append(_follower_state("R03", pos_err_m=3.0))
        leader._rally_completed = True

        leader.step(
            EntityInputS(
                selfState=_motion(east=0.0, north=0.0, h=500.0, v_east=20.0),
                inbox=[_follower_status_msg("R02", pos_err_m=1.0, arrived=1)],
                remote=RemoteCmdS(FormStageE.STANDBY),
                now_s=1.0,
            ),
            EntityOutputS(),
        )

        self.assertEqual([state.id for state in leader.cxt.followerStates], ["R03", "R02"])
        self.assertTrue(leader._rally_completed)

    def test_rally_leader_standby_keeps_subject_flow_slots(self) -> None:
        """验证长机待命不在实体主体流程早退，仍经过任务编排槽位后再进入位置解算。"""

        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((100.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
            )
        )
        task_calls: list[FormStageE] = []
        original_step = leader._task.step

        def record_task_step(u: object, y: object) -> None:
            """记录待命阶段是否进入任务编排槽位。"""

            assert getattr(u, "remote").stage == FormStageE.STANDBY
            task_calls.append(getattr(u, "remote").stage)
            original_step(u, y)

        leader._task.step = record_task_step  # type: ignore[method-assign]

        leader.step(
            EntityInputS(
                selfState=_motion(east=0.0, north=0.0, h=500.0, v_east=20.0),
                inbox=[_follower_status_msg("R02", pos_err_m=1.0, arrived=1)],
                remote=RemoteCmdS(FormStageE.STANDBY),
                now_s=1.0,
            ),
            EntityOutputS(),
        )

        self.assertEqual(task_calls, [FormStageE.STANDBY])
        self.assertEqual(leader._rally_join.state, RALLY_STATE_STANDBY)
        self.assertEqual([state.id for state in leader.cxt.followerStates], ["R02"])

    def test_rally_leader_init_rejects_empty_route_list(self) -> None:
        """验证 route=[] 时抛出 ValueError 而非 IndexError。"""
        with self.assertRaises(ValueError):
            RallyLeaderEntity().init(
                EntityInitS(
                    selfInit=FormSelfInitS("R01"),
                    commInit=_comm_init(),
                    route=[],  # 空列表，不是 None，守卫应捕获
                    rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                    rally_cfg=_rally_cfg(expected=()),
                )
            )

    def test_rally_route_heading_rejects_horizontally_degenerate_first_segment(self) -> None:
        """回归用例：A/A1 水平坐标重合（仅高度不同也算）时必须显式报错，不能静默按 atan2(0,0) 退化为正东。"""
        from src.algorithm.entity.leader_follower_rally import rally_route_heading_rad

        with self.assertRaises(ValueError):
            rally_route_heading_rad(_route((0.0, 0.0, 500.0), (0.0, 0.0, 520.0)))

    def test_rally_leader_init_rejects_horizontally_degenerate_rally_route(self) -> None:
        """验证长机 init 时也会拒绝水平退化的 rally_route 第一航段，而不是算出错误航向静默通过。"""
        with self.assertRaises(ValueError):
            RallyLeaderEntity().init(
                EntityInitS(
                    selfInit=FormSelfInitS("R01"),
                    commInit=_comm_init(),
                    route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                    rally_route=_route((0.0, 0.0, 500.0), (0.0, 0.0, 520.0)),
                    rally_cfg=_rally_cfg(expected=()),
                )
            )


if __name__ == "__main__":
    unittest.main()
