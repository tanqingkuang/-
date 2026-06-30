"""领航跟随集结算法的低层测试。"""

from __future__ import annotations

import math
import unittest

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    CommDirE,
    FollowerStateS,
    FormCommInitS,
    FormPatE,
    FormPosS,
    FormSelfInitS,
    FormSnapshotS,
    FormStageE,
    FormationAnalysisS,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
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
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RALLY_STATE_LOITERING,
)
from src.algorithm.units.algo.pos_calc.scaled_slot_geometry import ScaledSlotGeometry, ScaledSlotInitS, ScaledSlotInputS
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
from src.algorithm.units.process.outbound.leader_broadcast import _motion_payload
from src.algorithm.units.process.outbound.rally_leader_broadcast import RallyLeaderBroadcast, RallyLeaderBroadcastInputS
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
    pattern: FormPatE = FormPatE.TRIANGLE,
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
            targetPattern=FormPatE.TRIANGLE,
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
        formPat=[FormPatE.TRIANGLE],
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
        targetPattern=FormPatE.TRIANGLE,
        dt_s=dt_s,
        catchup_stable_s=catchup_stable_s,
    )


class RallyLeafTypesAndContextTests(unittest.TestCase):
    """验证集结扩展叶类型和上下文字段。"""

    def test_rally_leaf_type_defaults_and_copy_helpers(self) -> None:
        """验证默认值与复制函数覆盖所有集结扩展字段。"""

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
        self.assertIsNone(init.rally_target)
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
        self.assertEqual(ctx.cmd.pattern, FormPatE.NONE)
        self.assertAlmostEqual(ctx.slotScale.scale, 3.0)

        output = _task_step(task, ctx, remote=FormStageE.HOLD)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(ctx.cmd.pattern, FormPatE.TRIANGLE)
        self.assertAlmostEqual(ctx.slotScale.scale, 1.0)
        self.assertFalse(output.rallyCompleted)

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
                cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=FormPatE.TRIANGLE, step=2),
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


class RallyPosCalcTests(unittest.TestCase):
    """验证集结专用位置解算单元。"""

    def test_scaled_slot_geometry_scales_position_and_adds_compress_feedforward(self) -> None:
        """验证槽位偏置缩放和 scaleRate 速度前馈。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
        ctx.selfState = _motion(east=80.0, north=210.0, h=500.0, v_east=20.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.RALLY, pattern=FormPatE.TRIANGLE, step=2)
        ctx.slotScale = RallySlotScaleS(scale=2.0, scaleRate=-0.5)
        scaled = ScaledSlotGeometry()
        scaled.init(ScaledSlotInitS(selfId="R02", commInit=_comm_init()))

        scaled.step(
            ScaledSlotInputS(
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

    def test_scaled_slot_geometry_requires_slot_scale_port(self) -> None:
        """验证 slotScale 端口未绑定时失败。"""

        scaled = ScaledSlotGeometry()
        scaled.init(ScaledSlotInitS(selfId="R02", commInit=_comm_init()))
        with self.assertRaises(ValueError):
            scaled.step(
                ScaledSlotInputS(
                    selfState=_motion(),
                    leaderState=_motion(v_east=20.0),
                    cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=FormPatE.TRIANGLE),
                ),
                PosCalcOutputS(selfCmd=MotionProfS()),
            )


class RallyEntityTests(unittest.TestCase):
    """验证集结长机和僚机实体的主链路。"""

    def test_rally_follower_latches_arrival_and_switches_to_scaled_slot_after_step_one(self) -> None:
        """验证僚机到达松散点后进入 EXITED 上报，并在长机进入 LOOSE/COMPRESS 后使用缩放槽位。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_target=PosInEarthS(10.0, 20.0, 500.0),
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
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=1, scale=2.0, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            second,
        )

        assert second.selfCmd is not None
        # CATCHUP 阶段 selfCmd.pos = 本机在"杆"上的正交投影
        # 杆横侧向 = M_i.cross = M_i.north = 20（East 航向，mi_cross=-mi_e*0+mi_n*1=20）
        # 投影点 = (self.along=10, mi_cross=20) → ENU = (10, 20)
        self.assertAlmostEqual(second.selfCmd.pos.east, 10.0)
        self.assertAlmostEqual(second.selfCmd.pos.north, 20.0)
        # 速度航向锁定到 mission_heading=0（East）
        self.assertAlmostEqual(second.selfCmd.v.vPsi, 0.0)
        self.assertEqual(second.outbox[0].target, "R01")

    def test_rally_follower_waits_when_t_ref_is_not_valid_at_cold_start(self) -> None:
        """验证冷启动尚无有效 T_ref 时，已到目标点的僚机进入盘旋而不是直接切出。"""
        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                rally_target=PosInEarthS(10.0, 20.0, 500.0),
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
                rally_target=PosInEarthS(10.0, 20.0, 500.0),
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
                inbox=[_leader_msg(stage=FormStageE.NONE, pattern=FormPatE.NONE)],
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
                rally_target=PosInEarthS(10.0, 20.0, 500.0),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        output = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=1.0, north=2.0, h=3.0, v_east=20.0),
                inbox=[_leader_msg(stage=FormStageE.NONE, pattern=FormPatE.NONE)],
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

    def test_rally_leader_rejects_non_continuous_rally_and_mission_routes(self) -> None:
        """验证集结航线终点与任务航线起点不连续时初始化失败。"""

        leader = RallyLeaderEntity()
        with self.assertRaises(ValueError):
            leader.init(
                EntityInitS(
                    selfInit=FormSelfInitS("R01"),
                    commInit=_comm_init(),
                    route=_route((101.5, 0.0, 500.0), (200.0, 0.0, 500.0)),
                    rally_route=_route((0.0, 0.0, 500.0), (100.0, 0.0, 500.0)),
                    rally_cfg=_rally_cfg(expected=("R02",)),
                )
            )

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


if __name__ == "__main__":
    unittest.main()
