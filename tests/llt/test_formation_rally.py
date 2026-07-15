"""领航跟随集结算法的低层测试。"""

from __future__ import annotations

import math
import unittest
from copy import deepcopy
from fractions import Fraction
from itertools import product
from unittest.mock import patch

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
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
    RallyPlanS,
    RemoteCmdS,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
    copy_follower_state,
    copy_formation_analysis,
)
from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity
from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity
from src.algorithm.entity.types import EntityInitS as _EntityInitS
from src.algorithm.entity.types import EntityInputS, EntityOutputS, VelCmdLimitS
from src.algorithm.units.algo.pos_calc.base import PosCalcInputS, PosCalcOutputS, PosCalcStrategyE
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RALLY_STATE_LOITERING,
    RALLY_STATE_STANDBY,
    RallyJoinPos,
    RallyJoinPosInitS,
)
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS
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
from src.algorithm.units.process.tra_plan import TraPlanStrategyE
from src.common.envelope import MessageEnvelope


def EntityInitS(*args: object, **kwargs: object) -> _EntityInitS:
    """构造测试实体配置，并按集结长短机角色补齐流程策略表。"""

    cfg = _EntityInitS(*args, **kwargs)
    if cfg.rally_cfg is not None:
        cfg.pos_calc_default = (
            PosCalcStrategyE.SLOT_GEOMETRY
            if cfg.rally_leader_id
            else PosCalcStrategyE.ROUTE_INTERP
        )
        cfg.pos_calc_routes = (PosCalcStrategyE.RALLY_JOIN,)
        cfg.tra_plan_default = (
            TraPlanStrategyE.NOOP
            if cfg.rally_leader_id
            else TraPlanStrategyE.LEADER_ROUTE
        )
        cfg.tra_plan_strategies = (
            (TraPlanStrategyE.NOOP,)
            if cfg.rally_leader_id
            else (TraPlanStrategyE.NOOP, TraPlanStrategyE.LEADER_ROUTE)
        )
    return cfg


def _entity_rally_join(entity: RallyLeaderEntity | RallyFollowerEntity) -> RallyJoinPos:
    """取得 Manager 缓存的 RallyJoinPos，仅供既有白盒轨迹夹具驱动内部状态。"""

    strategy = entity._pos_calc._registry[PosCalcStrategyE.RALLY_JOIN]
    assert isinstance(strategy, RallyJoinPos)
    return strategy


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
    planned_path_length_m: float = -1.0,
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
        plannedPathLength_m=planned_path_length_m,
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
    planned_path_length_m: float = -1.0,
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
            "planned_path_length_m": planned_path_length_m,
        },
    )


def _leader_msg(
    *,
    stage: FormStageE = FormStageE.RALLY,
    pattern: int = 0,
    step: int = 0,
    leader_state: MotionProfS | None = None,
    t_ref: float = 0.0,
    t_ref_valid: bool = True,
    loop_counts: dict[str, int] | None = None,
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
            "t_ref": t_ref,
            "t_ref_valid": t_ref_valid,
            "loop_counts": dict(loop_counts) if loop_counts is not None else {"R02": 0},
        },
    )


def _rally_join_input(
    *,
    selfState: MotionProfS | None = None,
    t_ref: float = 0.0,
    t_ref_valid: bool = False,
    assigned_loops: int = 0,
    t_now: float = 0.0,
    standby: bool = False,
) -> PosCalcInputS:
    """把历史测试场景转换为统一位置解算输入端口。"""

    return PosCalcInputS(
        selfState=selfState,
        cmd=FormSnapshotS(stage=FormStageE.STANDBY if standby else FormStageE.RALLY),
        clock=AlgorithmClockS(now_s=t_now),
        rallyPlan=RallyPlanS(
            t_ref=t_ref,
            valid=t_ref_valid,
            loop_counts={"": assigned_loops},
        ),
    )


def _rally_task(
    expected: tuple[str, ...] = ("R02", "R03"),
    *,
    leader_id: str = "R01",
    dt_s: float = 0.1,
    stable_hold_s: float = 0.2,
    compress_time_s: float = 1.0,
    catchup_radius_m: float = 200.0,
    catchup_stable_s: float = 0.0,
    loiter_radius_m: float = 200.0,
    loiter_speed_min_mps: float = 14.0,
    loiter_speed_max_mps: float = 25.0,
) -> Rally:
    """构造测试用 Rally 任务单元。"""

    task = Rally()
    task.init(
        RallyTaskInitS(
            leaderId=leader_id,
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
            loiter_radius_m=loiter_radius_m,
            loiter_speed_min_mps=loiter_speed_min_mps,
            loiter_speed_max_mps=loiter_speed_max_mps,
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
    leader_path_length_m: float = -1.0,
) -> RallyTaskOutputS:
    """推进 Rally 任务一拍并返回输出端口。"""

    ctx.posCalcStatus.join_exited = leader_join_exited
    ctx.posCalcStatus.planned_path_length_m = leader_path_length_m
    output = RallyTaskOutputS(cmd=ctx.cmd)
    task.step(
        RallyTaskInputS(
            remote=RemoteCmdS(remote),
            cmd=ctx.cmd,
            followerStates=states or [],
            now_s=now_s,
            posCalcStatus=ctx.posCalcStatus,
        ),
        output,
    )
    return output


def _step_with_paths(
    task: Rally,
    *,
    now_s: float,
    leader_path: float,
    follower_paths: dict[str, float],
    ctx: FormContextS | None = None,
) -> RallyTaskOutputS:
    """用全队基础航程推进 Rally 任务一拍。"""

    bound_ctx = ctx if ctx is not None else FormContextS()
    states = [
        _follower_state(
            node_id,
            rally_state=RALLY_STATE_FLYING,
            planned_path_length_m=path_length,
            last_update_s=now_s,
        )
        for node_id, path_length in follower_paths.items()
    ]
    return _task_step(
        task,
        bound_ctx,
        remote=FormStageE.RALLY,
        states=states,
        now_s=now_s,
        leader_join_exited=False,
        leader_path_length_m=leader_path,
    )


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


def _comm_init_five() -> FormCommInitS:
    """构造五机集结通信和单队形槽位。"""

    node_ids = ("A01", "A02", "A03", "A04", "A05")
    return FormCommInitS(
        netWork=[NetWorkS("A01", node_id, CommDirE.DUPLEX) for node_id in node_ids[1:]],
        formPat=[0],
        formPos=[
            [
                FormPosS("A01", 0.0, 0.0, 0.0),
                FormPosS("A02", -10.0, 0.0, -10.0),
                FormPosS("A03", -20.0, 0.0, -20.0),
                FormPosS("A04", -30.0, 0.0, 10.0),
                FormPosS("A05", -40.0, 0.0, 20.0),
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


class FollowerStateTests(unittest.TestCase):
    """验证集结扩展叶类型和上下文字段。"""

    def test_rally_leaf_type_defaults_and_copy_helpers(self) -> None:
        """验证默认值与复制函数覆盖所有集结扩展字段。"""

        self.assertEqual(FormStageE.STANDBY, 4)
        self.assertFalse(FollowerStateS().valid)
        self.assertEqual(FormationAnalysisS(), FormationAnalysisS())

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

        first.followerStates.append(_follower_state("R02"))
        first.rally_t_ref_valid = True

        reset_context(first)

        self.assertEqual(first.followerStates, [])
        self.assertFalse(first.rally_t_ref_valid)


class EntityBoundaryTypesTests(unittest.TestCase):
    """验证实体边界结构已包含集结输入输出字段。"""

    def test_entity_boundary_defaults_include_rally_fields(self) -> None:
        """验证扩展字段默认值可供旧实体和集结实体同时使用。"""

        init = EntityInitS()
        self.assertEqual(init.route, [])
        self.assertFalse(hasattr(init, "rally_route"))
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
        """验证 Rally 初始化拒绝无效任务参数和盘旋时间区间参数。"""

        invalid_cases = [
            RallyTaskInitS(looseScale=0.9),
            RallyTaskInitS(compressTime_s=0.0),
            RallyTaskInitS(staleTimeout_s=0.0),
            RallyTaskInitS(dt_s=0.0),
            RallyTaskInitS(loiter_radius_m=0.0),
            RallyTaskInitS(loiter_speed_min_mps=0.0),
            RallyTaskInitS(loiter_speed_min_mps=20.0, loiter_speed_max_mps=20.0),
        ]
        for cfg in invalid_cases:
            with self.subTest(cfg=cfg):
                with self.assertRaises(ValueError):
                    Rally().init(cfg)

    def test_remote_none_and_hold_write_expected_command(self) -> None:
        """验证 NONE/HOLD 遥控只更新阶段和目标队形。"""

        task = _rally_task(expected=())
        ctx = FormContextS()

        _task_step(task, ctx, remote=FormStageE.NONE)
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        self.assertEqual(ctx.cmd.pattern, 0)

        output = _task_step(task, ctx, remote=FormStageE.HOLD)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(ctx.cmd.pattern, 0)
        self.assertFalse(output.rallyCompleted)

    def test_remote_standby_keeps_task_out_of_rally_state_machine(self) -> None:
        """验证 STANDBY 遥控不会被 Rally 任务误解释成开始集结。"""

        task = _rally_task(expected=())
        ctx = FormContextS()

        output = _task_step(task, ctx, remote=FormStageE.STANDBY)

        self.assertEqual(ctx.cmd.stage, FormStageE.STANDBY)
        self.assertEqual(ctx.cmd.step, RallyPhaseE.JOINING)
        self.assertFalse(output.rallyCompleted)

    def test_remote_rally_from_standby_resets_phase_timers_only(self) -> None:
        """验证 STANDBY→RALLY 重置子阶段计时器，但保留已锁存协调计划。"""

        task = _rally_task(expected=("R02",), dt_s=0.1)
        ctx = FormContextS()
        ctx.cmd.stage = FormStageE.STANDBY
        ctx.cmd.step = RallyPhaseE.COMPRESS
        task._stable_timer = 8.0
        task._catchup_stable_timer = 7.0
        task._compress_elapsed = 6.0
        task._t_ref = 123.0
        task._plan_ready = True
        task._loop_counts = {"R01": 4, "R02": 2}

        output = _task_step(task, ctx, remote=FormStageE.RALLY, states=[], now_s=5.0)

        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(ctx.cmd.step, RallyPhaseE.JOINING)
        self.assertTrue(output.t_ref_valid)
        self.assertAlmostEqual(output.t_ref, 123.0)
        self.assertEqual(output.loopCounts, {"R01": 4, "R02": 2})
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

    def assert_common_time_is_inside_assigned_intervals(
        self,
        output: RallyTaskOutputS,
        path_lengths: dict[str, float],
        *,
        plan_start_s: float = 10.0,
    ) -> None:
        """验证公共相对到达时间落在每架飞机分配圈数的可达区间内。"""

        circumference_m = 2.0 * math.pi * 200.0
        duration_s = output.t_ref - plan_start_s
        for node_id, length_m in path_lengths.items():
            assigned_length_m = length_m + output.loopCounts[node_id] * circumference_m
            self.assertGreaterEqual(duration_s + 1e-9, assigned_length_m / 25.0)
            self.assertLessEqual(duration_s - 1e-9, assigned_length_m / 14.0)

    def assert_exact_assigned_intervals_have_common_time(
        self,
        loop_counts: dict[str, int],
        path_lengths: dict[str, float],
        task: Rally,
    ) -> Fraction:
        """用输入浮点数的精确有理值验证分配存在真实公共时刻。"""

        circumference = Fraction.from_float(task._loiter_circumference_m)
        speed_min = Fraction.from_float(task._speed_min)
        speed_max = Fraction.from_float(task._speed_max)
        lower_bounds: list[Fraction] = []
        upper_bounds: list[Fraction] = []
        for node_id, length_m in path_lengths.items():
            distance = Fraction.from_float(length_m) + loop_counts[node_id] * circumference
            lower_bounds.append(distance / speed_max)
            upper_bounds.append(distance / speed_min)
        common_time = max(lower_bounds)
        self.assertLessEqual(common_time, min(upper_bounds), msg="精确物理区间无公共时刻")
        return common_time

    def test_path_coordinator_assigns_zero_loops_when_base_intervals_intersect(self) -> None:
        """基础区间已有交集时不应增加盘旋圈。"""

        task = _rally_task(expected=("A02",), leader_id="A01")
        output = _step_with_paths(
            task,
            now_s=10.0,
            leader_path=2000.0,
            follower_paths={"A02": 1800.0},
        )

        self.assertTrue(output.t_ref_valid)
        self.assertEqual(output.loopCounts, {"A01": 0, "A02": 0})

    def test_path_coordinator_assigns_integer_loops_to_shorter_route(self) -> None:
        """基础区间无交集时应给短航程飞机分配完整圈。"""

        task = _rally_task(expected=("A02",), leader_id="A01")
        output = _step_with_paths(
            task,
            now_s=10.0,
            leader_path=3000.0,
            follower_paths={"A02": 500.0},
        )

        self.assertTrue(output.t_ref_valid)
        self.assertEqual(output.loopCounts["A01"], 0)
        self.assertGreater(output.loopCounts["A02"], 0)
        self.assert_common_time_is_inside_assigned_intervals(
            output,
            {"A01": 3000.0, "A02": 500.0},
        )

    def test_path_coordinator_handles_narrow_speed_interval_with_bounded_jump(self) -> None:
        """极窄合法速度区间应直接跳到最早公共时间，不受固定迭代次数限制。"""

        speed_min = 20.0
        speed_max = 20.0001
        radius_m = 200.0
        circumference_m = 2.0 * math.pi * radius_m
        task = _rally_task(
            expected=("A02",),
            leader_id="A01",
            loiter_radius_m=radius_m,
            loiter_speed_min_mps=speed_min,
            loiter_speed_max_mps=speed_max,
        )

        output = _step_with_paths(
            task,
            now_s=10.0,
            leader_path=0.0,
            follower_paths={"A02": 100.0},
        )

        relative_width = (speed_max - speed_min) / speed_max
        expected_loops = math.ceil((100.0 / relative_width - 100.0) / circumference_m)
        expected_duration_s = (100.0 + expected_loops * circumference_m) / speed_max
        self.assertTrue(output.t_ref_valid)
        self.assertEqual(output.loopCounts, {"A01": expected_loops, "A02": expected_loops})
        self.assertAlmostEqual(output.t_ref - 10.0, expected_duration_s, places=6)

    def test_path_coordinator_keeps_exact_interval_at_one_ulp_speed_width(self) -> None:
        """一 ULP 合法速度窗不得用放大的浮点容差接受真实区间外圈数。"""

        task = _rally_task(
            expected=("B",),
            leader_id="A",
            loiter_radius_m=200.0,
            loiter_speed_min_mps=math.nextafter(20.0, 0.0),
            loiter_speed_max_mps=20.0,
        )
        path_lengths = {"A": 0.0, "B": 600.0}

        duration_s, loop_counts = task._coordinate_paths(path_lengths)

        exact_common_time = self.assert_exact_assigned_intervals_have_common_time(
            loop_counts,
            path_lengths,
            task,
        )
        self.assertGreaterEqual(Fraction.from_float(duration_s), exact_common_time)
        self.assertEqual(loop_counts, {"A": 2687888034010621, "B": 2687888034010621})

    def test_path_coordinator_matches_exact_bounded_bruteforce(self) -> None:
        """四节点小规模场景应与精确有理数穷举得到的最早公共区间一致。"""

        task = _rally_task(
            expected=("B", "C", "D"),
            leader_id="A",
            loiter_radius_m=20.0,
            loiter_speed_min_mps=14.0,
            loiter_speed_max_mps=25.0,
        )
        path_lengths = {"A": 100.0, "B": 180.0, "C": 260.0, "D": 330.0}
        circumference = Fraction.from_float(task._loiter_circumference_m)
        speed_min = Fraction.from_float(task._speed_min)
        speed_max = Fraction.from_float(task._speed_max)
        exact_lengths = {node_id: Fraction.from_float(length) for node_id, length in path_lengths.items()}
        feasible: list[tuple[Fraction, dict[str, int]]] = []
        for loop_values in product(range(6), repeat=len(path_lengths)):
            loop_counts = dict(zip(path_lengths, loop_values, strict=True))
            distances = {
                node_id: exact_lengths[node_id] + loop_counts[node_id] * circumference
                for node_id in path_lengths
            }
            lower = max(distance / speed_max for distance in distances.values())
            upper = min(distance / speed_min for distance in distances.values())
            if lower <= upper:
                feasible.append((lower, loop_counts))
        expected_lower, expected_loops = min(
            feasible,
            key=lambda item: (item[0], sum(item[1].values()), tuple(item[1].values())),
        )

        duration_s, loop_counts = task._coordinate_paths(path_lengths)

        self.assertEqual(loop_counts, expected_loops)
        self.assertGreaterEqual(Fraction.from_float(duration_s), expected_lower)
        self.assert_exact_assigned_intervals_have_common_time(
            loop_counts,
            path_lengths,
            task,
        )

    def test_path_coordinator_rejects_invalid_path_length(self) -> None:
        """搜索边界应明确拒绝负值与非有限航程，不能返回非法公共时间。"""

        task = _rally_task(expected=(), leader_id="A01")
        for invalid_length in (-1.0, float("nan"), float("inf"), float("-inf")):
            with self.subTest(path_length=invalid_length):
                with self.assertRaises(ValueError):
                    task._coordinate_paths({"A01": invalid_length})

    def test_path_coordinator_reports_unrepresentable_common_time(self) -> None:
        """若最早公共时间超出有限浮点范围，搜索应明确失败而不是溢出或假收敛。"""

        radius_m = 1.0e307
        task = _rally_task(
            expected=("A02",),
            leader_id="A01",
            loiter_radius_m=radius_m,
            loiter_speed_min_mps=1.0,
            loiter_speed_max_mps=math.nextafter(1.0, math.inf),
        )
        half_circumference_m = task._loiter_circumference_m / 2.0

        with self.assertRaises(RuntimeError):
            task._coordinate_paths({"A01": 0.0, "A02": half_circumference_m})

    def test_path_plan_is_locked_after_first_valid_result(self) -> None:
        """后续航程回报变化和阶段推进不得移动已确认计划。"""

        task = _rally_task(expected=("A02",), leader_id="A01")
        ctx = FormContextS()
        first = _step_with_paths(
            task,
            ctx=ctx,
            now_s=10.0,
            leader_path=3000.0,
            follower_paths={"A02": 500.0},
        )

        for phase in (
            RallyPhaseE.JOINING,
            RallyPhaseE.CATCHUP,
            RallyPhaseE.LOOSE,
            RallyPhaseE.COMPRESS,
        ):
            ctx.cmd.stage = FormStageE.RALLY
            ctx.cmd.step = phase
            later = _step_with_paths(
                task,
                ctx=ctx,
                now_s=20.0,
                leader_path=100.0,
                follower_paths={"A02": 9000.0},
            )
            self.assertEqual(later.t_ref, first.t_ref)
            self.assertEqual(later.loopCounts, first.loopCounts)

        ctx.cmd.stage = FormStageE.HOLD
        hold = _step_with_paths(
            task,
            ctx=ctx,
            now_s=30.0,
            leader_path=100.0,
            follower_paths={"A02": 9000.0},
        )
        self.assertEqual(hold.t_ref, first.t_ref)
        self.assertEqual(hold.loopCounts, first.loopCounts)

    def test_started_task_rejects_reverse_standby_and_restart_until_reset(self) -> None:
        """任务进入 RALLY 后应拒绝反向 STANDBY 及 NONE 后重启，显式 reset 才开启新任务。"""

        task = _rally_task(expected=("A02",), leader_id="A01")
        ctx = FormContextS()
        first = _step_with_paths(
            task,
            ctx=ctx,
            now_s=10.0,
            leader_path=3000.0,
            follower_paths={"A02": 500.0},
        )
        locked_t_ref = first.t_ref
        locked_loops = first.loopCounts

        reverse_standby = _task_step(task, ctx, remote=FormStageE.STANDBY, now_s=20.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(reverse_standby.t_ref, locked_t_ref)
        self.assertEqual(reverse_standby.loopCounts, locked_loops)

        stopped = _task_step(task, ctx, remote=FormStageE.NONE, now_s=30.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        rejected_rally = _step_with_paths(
            task,
            ctx=ctx,
            now_s=40.0,
            leader_path=100.0,
            follower_paths={"A02": 9000.0},
        )
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        rejected_standby = _task_step(task, ctx, remote=FormStageE.STANDBY, now_s=45.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        for output in (stopped, rejected_rally, rejected_standby):
            self.assertTrue(output.t_ref_valid)
            self.assertEqual(output.t_ref, locked_t_ref)
            self.assertEqual(output.loopCounts, locked_loops)

        task.reset()
        reset_output = _task_step(task, ctx, remote=FormStageE.STANDBY, now_s=50.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.STANDBY)
        self.assertFalse(reset_output.t_ref_valid)
        self.assertEqual(reset_output.t_ref, 0.0)
        self.assertEqual(reset_output.loopCounts, {})

        replanned = _step_with_paths(
            task,
            ctx=ctx,
            now_s=60.0,
            leader_path=100.0,
            follower_paths={"A02": 9000.0},
        )
        self.assertTrue(replanned.t_ref_valid)
        self.assertNotEqual(replanned.t_ref, locked_t_ref)
        self.assertNotEqual(replanned.loopCounts, locked_loops)

    def test_path_plan_waits_for_every_expected_follower(self) -> None:
        """缺少任一期望僚机基础航程时计划必须保持无效。"""

        task = _rally_task(expected=("A02", "A03"), leader_id="A01")
        output = _step_with_paths(
            task,
            now_s=10.0,
            leader_path=3000.0,
            follower_paths={"A02": 500.0},
        )

        self.assertFalse(output.t_ref_valid)
        self.assertEqual(output.loopCounts, {})

    def test_path_plan_uses_all_paths_received_time_as_start(self) -> None:
        """固定计划应以全队航程收齐时刻为时间基准，而不是首次进入 RALLY 的时刻。"""

        task = _rally_task(expected=("A02", "A03"), leader_id="A01")
        ctx = FormContextS()
        path_lengths = {"A01": 3000.0, "A02": 500.0, "A03": 700.0}
        expected_duration_s, _ = task._coordinate_paths(path_lengths)

        incomplete = _step_with_paths(
            task,
            ctx=ctx,
            now_s=10.0,
            leader_path=path_lengths["A01"],
            follower_paths={"A02": path_lengths["A02"]},
        )
        self.assertFalse(incomplete.t_ref_valid)

        completed = _step_with_paths(
            task,
            ctx=ctx,
            now_s=30.0,
            leader_path=path_lengths["A01"],
            follower_paths={"A02": path_lengths["A02"], "A03": path_lengths["A03"]},
        )

        self.assertTrue(completed.t_ref_valid)
        self.assertAlmostEqual(completed.t_ref, 30.0 + expected_duration_s)

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
        self.assertFalse(first_compress.rallyCompleted)

        completed = _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertTrue(completed.rallyCompleted)

        next_frame = _task_step(task, ctx, remote=FormStageE.RALLY, states=ok, now_s=0.0)
        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertFalse(next_frame.rallyCompleted)

    def test_none_then_rally_is_rejected_after_completed_hold_until_reset(self) -> None:
        """完成 HOLD 后进入 NONE，未经显式 reset 的 RALLY/STANDBY 都不得重启任务。"""

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
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)
        _task_step(task, ctx, remote=FormStageE.STANDBY, states=[])
        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)

        task.reset()
        reset_output = _task_step(task, ctx, remote=FormStageE.STANDBY, states=[])
        self.assertEqual(ctx.cmd.stage, FormStageE.STANDBY)
        self.assertFalse(reset_output.t_ref_valid)
        self.assertEqual(reset_output.loopCounts, {})

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


class FollowerStatusTests(unittest.TestCase):
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

    def test_follower_status_round_trip_carries_planned_path_length(self) -> None:
        """僚机基础航程应通过现有状态 topic 原子写入长机黑板。"""

        outbound = FollowerBroadcast()
        outbound.init(FollowerBroadcastInitS(selfId="R02", leaderId="R01"))
        outbound_input = FollowerBroadcastInputS(
            selfState=_motion(east=1.0, north=2.0, h=3.0),
            selfCmd=_motion(east=4.0, north=5.0, h=6.0),
            planned_path_length_m=3210.5,
        )
        outbound_output = OutboundOutputS()
        outbound.step(outbound_input, outbound_output)

        assert outbound_output.outbox is not None
        payload = outbound_output.outbox[0].payload
        self.assertNotIn("eta_s", payload)
        self.assertNotIn("eta_min_s", payload)
        self.assertNotIn("eta_max_s", payload)
        self.assertEqual(payload["planned_path_length_m"], 3210.5)

        states: list[FollowerStateS] = []
        inbound = FollowerStatus()
        inbound.init(None)
        inbound.step(
            FollowerStatusInputS(inbox=outbound_output.outbox, now_s=5.0),
            FollowerStatusOutputS(followerStates=states),
        )

        self.assertEqual(len(states), 1)
        self.assertEqual(states[0].plannedPathLength_m, 3210.5)

    def test_follower_status_rejects_invalid_planned_path_length(self) -> None:
        """小于待规划哨兵值的航程不能覆盖上一份有效规划数据。"""

        states: list[FollowerStateS] = []
        inbound = FollowerStatus()
        inbound.init(None)
        valid = _follower_status_msg("R02")
        valid.payload["planned_path_length_m"] = 1000.0
        inbound.step(
            FollowerStatusInputS(inbox=[valid], now_s=5.0),
            FollowerStatusOutputS(followerStates=states),
        )
        invalid = _follower_status_msg("R02", pos_east=99.0)
        invalid.payload["planned_path_length_m"] = -2.0
        inbound.step(
            FollowerStatusInputS(inbox=[invalid], now_s=6.0),
            FollowerStatusOutputS(followerStates=states),
        )

        self.assertEqual(len(states), 1)
        self.assertEqual(states[0].plannedPathLength_m, 1000.0)
        self.assertAlmostEqual(states[0].pos.east, 0.0)
        self.assertAlmostEqual(states[0].lastUpdate_s, 5.0)

    def test_follower_status_accepts_unplanned_sentinel_during_standby(self) -> None:
        """STANDBY 的 -1 哨兵不应阻断其他状态字段更新。"""

        states: list[FollowerStateS] = []
        inbound = FollowerStatus()
        inbound.init(None)
        message = _follower_status_msg("R02", rally_state=RALLY_STATE_STANDBY, pos_east=8.0)
        message.payload["planned_path_length_m"] = -1.0
        inbound.step(
            FollowerStatusInputS(inbox=[message], now_s=5.0),
            FollowerStatusOutputS(followerStates=states),
        )

        self.assertEqual(states[0].plannedPathLength_m, -1.0)
        self.assertEqual(states[0].rally_state, RALLY_STATE_STANDBY)
        self.assertAlmostEqual(states[0].pos.east, 8.0)

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
            "planned_path_length_m": float("-inf"),
        }
        for field_name, invalid_value in invalid_fields.items():
            with self.subTest(field=field_name):
                baseline = _follower_state("R02", pos_err_m=4.0, arrived=1, valid=True, last_update_s=10.0)
                baseline.pos = _pos(1.0, 2.0, 3.0)
                baseline.headingErr_rad = 0.2
                baseline.plannedPathLength_m = 20.0
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

        new_invalid = _follower_status_msg("R03")
        new_invalid.payload["planned_path_length_m"] = float("inf")
        inbound.step(
            FollowerStatusInputS(inbox=[new_invalid], now_s=11.0),
            FollowerStatusOutputS(followerStates=states),
        )
        self.assertEqual([state.id for state in states], ["R02"])

    def test_follower_status_requires_bound_output_list(self) -> None:
        """验证输出列表端口未绑定时失败。"""

        with self.assertRaises(ValueError):
            FollowerStatus().step(FollowerStatusInputS(inbox=[]), FollowerStatusOutputS())


class RallyCommunicationTests(unittest.TestCase):
    """验证集结长机广播扩展和僚机解析。"""

    def test_leader_inbound_rejects_invalid_motion_atomically(self) -> None:
        """有效报文后若新报文的中途运动字段非法，应静默拒绝且保留完整旧快照。"""

        ctx = FormContextS()
        leader_cmd = MotionProfS()
        inbound = RallyLeaderFollower()
        parsed = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            leaderCmd=leader_cmd,
            cmd=ctx.cmd,
        )
        valid = _leader_msg(
            stage=FormStageE.HOLD,
            pattern=1,
            step=3,
            leader_state=_motion(east=1.0, north=2.0, h=3.0, v_east=4.0, v_north=5.0),
            t_ref=12.0,
            loop_counts={"R02": 1},
        )
        inbound.step(InboundInputS(inbox=[valid]), parsed)
        baseline = deepcopy((ctx.leaderState, leader_cmd, ctx.cmd, parsed.t_ref, parsed.t_ref_valid, parsed.loopCounts))

        malformed = _leader_msg(
            stage=FormStageE.RALLY,
            pattern=2,
            step=4,
            leader_state=_motion(east=91.0, north=92.0, h=93.0, v_east=94.0, v_north=95.0),
            t_ref=180.0,
            loop_counts={"R02": 2},
        )
        malformed.payload["leader_state"]["vd"]["vNorth"] = "非法速度"

        inbound.step(InboundInputS(inbox=[malformed]), parsed)

        self.assertEqual(
            (ctx.leaderState, leader_cmd, ctx.cmd, parsed.t_ref, parsed.t_ref_valid, parsed.loopCounts),
            baseline,
        )

    def test_leader_inbound_rejects_invalid_cmd_atomically(self) -> None:
        """有效报文后若新报文的 cmd 枚举非法，应静默拒绝且保留完整旧快照。"""

        ctx = FormContextS()
        leader_cmd = MotionProfS()
        inbound = RallyLeaderFollower()
        parsed = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            leaderCmd=leader_cmd,
            cmd=ctx.cmd,
        )
        valid = _leader_msg(
            stage=FormStageE.HOLD,
            pattern=1,
            step=3,
            leader_state=_motion(east=1.0, north=2.0, h=3.0, v_east=4.0),
            t_ref=12.0,
            loop_counts={"R02": 1},
        )
        inbound.step(InboundInputS(inbox=[valid]), parsed)
        baseline = deepcopy((ctx.leaderState, leader_cmd, ctx.cmd, parsed.t_ref, parsed.t_ref_valid, parsed.loopCounts))

        malformed = _leader_msg(
            stage=FormStageE.RALLY,
            pattern=2,
            step=4,
            leader_state=_motion(east=91.0, north=92.0, h=93.0, v_east=94.0),
            t_ref=180.0,
            loop_counts={"R02": 2},
        )
        malformed.payload["cmd"]["stage"] = 999

        inbound.step(InboundInputS(inbox=[malformed]), parsed)

        self.assertEqual(
            (ctx.leaderState, leader_cmd, ctx.cmd, parsed.t_ref, parsed.t_ref_valid, parsed.loopCounts),
            baseline,
        )

    def test_leader_inbound_reset_clears_latched_output(self) -> None:
        """入站 reset 应把全部绑定输出恢复默认值，不能保留上一份长机广播。"""

        ctx = FormContextS()
        leader_cmd = MotionProfS()
        inbound = RallyLeaderFollower()
        parsed = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            leaderCmd=leader_cmd,
            cmd=ctx.cmd,
        )
        inbound.step(
            InboundInputS(
                inbox=[
                    _leader_msg(
                        stage=FormStageE.RALLY,
                        pattern=2,
                        step=3,
                        leader_state=_motion(east=91.0, north=92.0, h=93.0, v_east=94.0),
                        t_ref=180.0,
                        loop_counts={"R02": 2},
                    )
                ]
            ),
            parsed,
        )

        inbound.reset()

        self.assertEqual(ctx.leaderState, MotionProfS())
        self.assertEqual(leader_cmd, MotionProfS())
        self.assertEqual(ctx.cmd, FormSnapshotS())
        self.assertEqual((parsed.t_ref, parsed.t_ref_valid, parsed.loopCounts), (0.0, False, {}))

    def test_leader_plan_round_trip_carries_fixed_loop_counts(self) -> None:
        """长机广播应原样传递固定 T_ref 与节点圈数映射。"""

        outbound = RallyLeaderBroadcast()
        outbound.init(OutboundInitS(selfId="R01", netWork=[NetWorkS("R01", "R02", CommDirE.DUPLEX)]))
        broadcast_input = RallyLeaderBroadcastInputS(
            cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=0),
            selfState=_motion(),
            t_ref=180.0,
            t_ref_valid=True,
            loop_counts={"A01": 0, "A02": 2},
        )
        broadcast_output = OutboundOutputS()
        outbound.step(broadcast_input, broadcast_output)

        payload = broadcast_output.outbox[0].payload
        self.assertEqual(payload.get("loop_counts"), {"A01": 0, "A02": 2})

        ctx = FormContextS()
        parsed = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
        )
        RallyLeaderFollower().step(InboundInputS(inbox=broadcast_output.outbox), parsed)

        self.assertTrue(parsed.t_ref_valid)
        self.assertEqual(parsed.t_ref, 180.0)
        self.assertEqual(parsed.loopCounts, {"A01": 0, "A02": 2})

    def test_leader_plan_rejects_invalid_loop_counts_atomically(self) -> None:
        """圈数映射包含非法键值时应拒绝整条入站消息，不提交任何计划字段。"""

        invalid_plans: list[tuple[str, dict[object, object]]] = [
            ("数值键", {2: 1}),
            ("布尔值", {"R02": True}),
            ("负整数", {"R02": -1}),
            ("浮点数", {"R02": 1.8}),
        ]
        for case_name, invalid_plan in invalid_plans:
            with self.subTest(case=case_name):
                ctx = FormContextS()
                ctx.cmd.stage = FormStageE.HOLD
                ctx.cmd.pattern = 3
                ctx.cmd.step = 7
                parsed = RallyLeaderFollowerOutputS(
                    leaderState=ctx.leaderState,
                    cmd=ctx.cmd,
                    t_ref=12.0,
                    t_ref_valid=True,
                    loopCounts={"BASE": 4},
                )
                message = _leader_msg(stage=FormStageE.RALLY, pattern=0, step=2, t_ref=180.0)
                message.payload["loop_counts"] = invalid_plan

                RallyLeaderFollower().step(InboundInputS(inbox=[message]), parsed)

                self.assertEqual((ctx.cmd.stage, ctx.cmd.pattern, ctx.cmd.step), (FormStageE.HOLD, 3, 7))
                self.assertEqual((parsed.t_ref, parsed.t_ref_valid), (12.0, True))
                self.assertEqual(parsed.loopCounts, {"BASE": 4})

    def test_leader_broadcast_rejects_invalid_loop_counts(self) -> None:
        """出站端口包含非法键值时应明确失败，且不得生成广播消息。"""

        invalid_plans: list[tuple[str, dict[object, object]]] = [
            ("数值键", {2: 1}),
            ("布尔值", {"R02": True}),
            ("负整数", {"R02": -1}),
            ("浮点数", {"R02": 1.8}),
        ]
        outbound = RallyLeaderBroadcast()
        outbound.init(OutboundInitS(selfId="R01", netWork=[NetWorkS("R01", "R02", CommDirE.DUPLEX)]))
        for case_name, invalid_plan in invalid_plans:
            with self.subTest(case=case_name):
                output = OutboundOutputS()
                broadcast_input = RallyLeaderBroadcastInputS(
                    cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=0),
                    selfState=_motion(),
                    t_ref=180.0,
                    t_ref_valid=True,
                    loop_counts=invalid_plan,  # type: ignore[arg-type]
                )

                with self.assertRaises(ValueError):
                    outbound.step(broadcast_input, output)
                self.assertEqual(output.outbox, [])

    def test_rally_leader_broadcast_omits_removed_slot_scale(self) -> None:
        """验证长机广播保留必要字段，不再发送已取消的缩放协议。"""

        outbound = RallyLeaderBroadcast()
        outbound.init(OutboundInitS(selfId="R01", netWork=[NetWorkS("R01", "R02", CommDirE.DUPLEX)]))
        output = OutboundOutputS()

        outbound.step(
            RallyLeaderBroadcastInputS(
                cmd=FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=2),
                selfState=_motion(east=1.0, north=2.0, h=3.0, v_east=4.0),
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
        self.assertNotIn("slot_scale", payload)
        self.assertEqual(payload["t_ref"], 12.0)
        self.assertTrue(payload["t_ref_valid"])

    def test_rally_leader_follower_parses_message_without_slot_scale(self) -> None:
        """验证僚机入站在无缩放字段的新协议下解析长机状态、指令和计划。"""

        ctx = FormContextS()
        inbound = RallyLeaderFollower()
        inbound_output = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
        )
        inbound.step(
            InboundInputS(inbox=[_leader_msg(step=2, t_ref=12.0, t_ref_valid=True)]),
            inbound_output,
        )

        self.assertEqual(ctx.cmd.stage, FormStageE.RALLY)
        self.assertEqual(ctx.cmd.step, 2)
        self.assertAlmostEqual(ctx.leaderState.pos.east, 100.0)
        self.assertAlmostEqual(inbound_output.t_ref, 12.0)
        self.assertTrue(inbound_output.t_ref_valid)

        old_format = _leader_msg()
        old_format.payload["slot_scale"] = {"scale": 9.0, "scale_rate": -4.0}
        del old_format.payload["t_ref_valid"]
        old_output = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
        )
        inbound.step(
            InboundInputS(inbox=[old_format]),
            old_output,
        )
        self.assertFalse(old_output.t_ref_valid)

    def test_invalid_t_ref_does_not_commit_partial_cmd_state(self) -> None:
        """t_ref 解析异常时不应提交本条消息中已解析的 cmd.stage/step，避免「新阶段 + 无效 T_ref」半截状态。"""
        ctx = FormContextS()
        inbound = RallyLeaderFollower()
        out = RallyLeaderFollowerOutputS(
            leaderState=ctx.leaderState,
            cmd=ctx.cmd,
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

    def test_non_finite_t_ref_is_rejected_atomically(self) -> None:
        """NaN 和正负 Inf 的 t_ref 应整条拒绝，不得提交任何同帧计划字段。"""

        for invalid_t_ref in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(t_ref=invalid_t_ref):
                ctx = FormContextS()
                ctx.leaderState = _motion(east=1.0, north=2.0, h=3.0)
                ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=3, step=7)
                out = RallyLeaderFollowerOutputS(
                    leaderState=ctx.leaderState,
                    cmd=ctx.cmd,
                    t_ref=12.0,
                    t_ref_valid=True,
                    loopCounts={"BASE": 4},
                )
                message = _leader_msg(
                    stage=FormStageE.RALLY,
                    pattern=0,
                    step=2,
                    leader_state=_motion(east=99.0, north=98.0, h=97.0),
                    t_ref=180.0,
                    t_ref_valid=True,
                    loop_counts={"R02": 2},
                )
                message.payload["t_ref"] = invalid_t_ref

                RallyLeaderFollower().step(InboundInputS(inbox=[message]), out)

                self.assertEqual(ctx.leaderState.pos, _pos(1.0, 2.0, 3.0))
                self.assertEqual((ctx.cmd.stage, ctx.cmd.pattern, ctx.cmd.step), (FormStageE.HOLD, 3, 7))
                self.assertEqual((out.t_ref, out.t_ref_valid), (12.0, True))
                self.assertEqual(out.loopCounts, {"BASE": 4})

    def test_rally_leader_follower_requires_all_output_ports(self) -> None:
        """验证三类输出端口必须同时绑定。"""

        with self.assertRaises(ValueError):
            RallyLeaderFollower().step(InboundInputS(inbox=[]), RallyLeaderFollowerOutputS())


class RallyLooseTargetTests(unittest.TestCase):
    """直接单测 rally_loose_target() 的 ENU 水平集结平面语义。"""

    def test_pure_forward_offset_at_zero_heading(self) -> None:
        """heading=0（正东）时，纯前向偏置（x 分量）应直接映射为东向偏置，北向不变。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 1.0, FormPosS("R02", 10.0, 5.0, 0.0))
        self.assertAlmostEqual(m_i.east, 10.0)
        self.assertAlmostEqual(m_i.north, 0.0)
        self.assertAlmostEqual(m_i.h, 105.0)

    def test_right_axis_sign_at_zero_heading(self) -> None:
        """heading=0（正东）时，纯右侧偏置（z 分量，正值=右）应映射为负的北向偏置（面向正东时右手边是正南）。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), 0.0, 1.0, FormPosS("R02", 0.0, 0.0, 10.0))
        self.assertAlmostEqual(m_i.east, 0.0)
        self.assertAlmostEqual(m_i.north, -10.0,
            msg="positive slot.z (right, facing east) must map to negative north (south), not positive")

    def test_rotates_forward_offset_with_heading(self) -> None:
        """heading=90°（正北）时，纯前向偏置应旋转成北向偏置，验证旋转矩阵方向而非仅测 heading=0。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import rally_loose_target

        m_i = rally_loose_target(_pos(0.0, 0.0, 100.0), math.pi / 2.0, 1.0, FormPosS("R02", 10.0, 0.0, 0.0))
        self.assertAlmostEqual(m_i.east, 0.0, places=6)
        self.assertAlmostEqual(m_i.north, 10.0, places=6)

    def test_looseScale_multiplies_horizontal_offset_only(self) -> None:
        """looseScale 应线性放大水平偏置（east/north），但高度偏置（slot.y）必须保持固定，不随 scale 扩展。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import rally_loose_target

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
        from src.algorithm.units.algo.pos_calc.rally_join_pos import rally_loose_target

        m_i = rally_loose_target(_pos(1000.0, 2000.0, 500.0), 0.0, 2.0, FormPosS("R02", 10.0, 0.0, 0.0))
        self.assertAlmostEqual(m_i.east, 1020.0)
        self.assertAlmostEqual(m_i.north, 2000.0)
        self.assertAlmostEqual(m_i.h, 500.0)

    def test_climbing_first_segment_still_uses_horizontal_rally_plane(self) -> None:
        """首航段有非零倾角时，M_i 仍是水平盘旋几何，不能误套三维 FUR 倾角旋转。"""
        from src.algorithm.entity.leader_follower_rally import rally_loose_target, route_heading_rad

        route = _route((0.0, 0.0, 100.0), (100.0, 0.0, 200.0))
        heading = route_heading_rad(route)
        m_i = rally_loose_target(route[0].pos, heading, 1.0, FormPosS("R02", 10.0, 5.0, 0.0))

        self.assertAlmostEqual(heading, 0.0)
        self.assertAlmostEqual(m_i.east, 10.0)
        self.assertAlmostEqual(m_i.north, 0.0)
        self.assertAlmostEqual(m_i.h, 105.0)


class RallyLoiterSpeedBoundsTests(unittest.TestCase):
    """直接单测 loiter_speed_bounds()：只显式配置 forwardMin/forwardMax 中的一侧时，与另一侧的默认
    兜底值反序的情形必须被显式拒绝，而不是静默产出 min>=max 的非法区间留给下游报 ERR_MODULE_INIT_FAILED。"""

    def test_only_forward_max_configured_below_default_min_rejected(self) -> None:
        """只配 forwardMax=10（< 默认 loiter_min=14）时，(14, 10) 是非法区间，必须拒绝。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import loiter_speed_bounds

        with self.assertRaises(ValueError):
            loiter_speed_bounds(VelCmdLimitS(forwardMax=10.0))

    def test_only_forward_min_configured_above_default_max_rejected(self) -> None:
        """只配 forwardMin=30（> 默认 loiter_max=25）时，(30, 25) 是非法区间，必须拒绝。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import loiter_speed_bounds

        with self.assertRaises(ValueError):
            loiter_speed_bounds(VelCmdLimitS(forwardMin=30.0))

    def test_both_unconfigured_uses_valid_defaults(self) -> None:
        """两侧都不配置时退回默认 (14, 25)，本身自洽，不应报错。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import loiter_speed_bounds

        loiter_min, loiter_max = loiter_speed_bounds(VelCmdLimitS())
        self.assertEqual((loiter_min, loiter_max), (14.0, 25.0))

    def test_both_explicitly_configured_and_consistent_passes_through(self) -> None:
        """两侧都显式配置且自洽时，原样透传，不受默认值影响。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import loiter_speed_bounds

        loiter_min, loiter_max = loiter_speed_bounds(VelCmdLimitS(forwardMin=18.0, forwardMax=22.0))
        self.assertEqual((loiter_min, loiter_max), (18.0, 22.0))


def _new_join_for_transit() -> RallyJoinPos:
    """构造用于公切线转移测试的 RallyJoinPos。"""
    join = RallyJoinPos()
    join.init(RallyJoinPosInitS(
        loose_slot=_pos(1000.0, 0.0, 560.0),
        approach_speed_mps=20.0,
        slow_radius_m=100.0,
        arrival_radius_m=20.0,
        loiter_radius_m=200.0,
        loiter_speed_min_mps=14.0,
        loiter_speed_max_mps=25.0,
        mission_heading_rad=0.0,
        mission_speed_mps=20.0,
        control_period_s=0.05,
        standby_altitude_m=560.0,
    ))
    return join


def _make_standby_join_for_transit() -> tuple[RallyJoinPos, MotionProfS, PosCalcOutputS]:
    """构造两个分离等半径圆，并把飞机放在待命圆顶部等待切出。"""

    join = _new_join_for_transit()
    output = PosCalcOutputS(selfCmd=MotionProfS())
    enter_state = _motion(
        east=0.0,
        north=0.0,
        h=560.0,
        v_east=20.0,
        vd=20.0,
        v_psi=0.0,
    )
    join.step(_rally_join_input(selfState=enter_state, standby=True), output)
    transit_state = _motion(
        east=0.0,
        north=400.0,
        h=560.0,
        v_east=-20.0,
        vd=20.0,
        v_psi=math.pi,
    )
    return join, transit_state, output


def _started_join_with_two_circles() -> tuple[RallyJoinPos, MotionProfS, PosCalcOutputS]:
    """启动分离两圆的公切线汇合路径。"""

    join, state, output = _make_standby_join_for_transit()
    join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)
    return join, state, output


def _started_join_with_point_fallback() -> tuple[RallyJoinPos, float]:
    """启动公切线无解时的点到集结圆退化路径，并返回独立计算的水平航程。"""

    join, state, output = _make_standby_join_for_transit()
    with patch(
        "src.algorithm.units.algo.pos_calc.rally_join_pos.common_tangent",
        return_value=None,
    ):
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)
    assert join._entry_point is not None
    line_length = math.hypot(
        join._entry_point.east - state.pos.east,
        join._entry_point.north - state.pos.north,
    )
    rally_arc = join._loiter_radius * ((join._theta_slot - join._theta_entry) % (2.0 * math.pi))
    return join, line_length + rally_arc


def _started_join_with_coincident_circles() -> tuple[RallyJoinPos, float]:
    """启动待命圆与集结圆重合的汇合路径，并返回独立计算的 CCW 圆弧。"""

    join = RallyJoinPos()
    join.init(RallyJoinPosInitS(
        loose_slot=_pos(0.0, 0.0, 560.0),
        approach_speed_mps=20.0,
        arrival_radius_m=20.0,
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
        _rally_join_input(
            selfState=_motion(east=0.0, north=0.0, h=560.0, v_east=20.0, vd=20.0),
            standby=True,
        ),
        output,
    )
    state = _motion(east=0.0, north=400.0, h=560.0, v_east=-20.0, vd=20.0, v_psi=math.pi)
    theta = math.atan2(state.pos.north - join._loiter_center_n, state.pos.east - join._loiter_center_e)
    expected_arc = join._loiter_radius * ((join._theta_slot - theta) % (2.0 * math.pi))
    join.step(_rally_join_input(selfState=state, standby=False, t_now=5.0), output)
    return join, expected_arc


def _join_at_transit_phase(phase: str) -> tuple[RallyJoinPos, MotionProfS, PosCalcOutputS]:
    """把汇合解算器推进到指定转移子阶段，并返回该阶段内的代表位置。"""

    join, state, output = _started_join_with_two_circles()
    if phase == "ARC_TO_TANGENT":
        theta = join._theta_local_exit - math.pi / 2.0
        state.pos.east = join._standby_center_e + join._loiter_radius * math.cos(theta)
        state.pos.north = join._standby_center_n + join._loiter_radius * math.sin(theta)
        state.v.vPsi = theta + math.pi / 2.0
        return join, state, output

    assert join._local_exit_point is not None
    assert join._entry_point is not None
    state.pos.east = join._local_exit_point.east
    state.pos.north = join._local_exit_point.north
    state.v.vPsi = join._theta_local_exit + math.pi / 2.0
    join.step(_rally_join_input(selfState=state, t_now=10.1), output)
    if phase == "LINE_TO_RALLY_ENTRY":
        state.pos.east = (join._local_exit_point.east + join._entry_point.east) / 2.0
        state.pos.north = (join._local_exit_point.north + join._entry_point.north) / 2.0
        return join, state, output

    if phase == "LOITERING":
        state.pos.east = join._entry_point.east
        state.pos.north = join._entry_point.north
        state.pos.h = join._entry_point.h
        join.step(_rally_join_input(selfState=state, t_now=10.2), output)
        theta = join._theta_slot - math.pi
        state.pos.east = join._loiter_center_e + join._loiter_radius * math.cos(theta)
        state.pos.north = join._loiter_center_n + join._loiter_radius * math.sin(theta)
        state.v.vPsi = theta + math.pi / 2.0
        return join, state, output

    raise ValueError(f"未知转移子阶段: {phase}")


def _join_loitering_with_plan(assigned_loops: int) -> tuple[RallyJoinPos, PosCalcOutputS]:
    """构造已锁存固定计划且位于 M_i 远角窗的盘旋状态。"""

    join = _new_join_for_transit()
    join._state = RALLY_STATE_LOITERING
    join._transit_phase = None
    join._theta_entry = join._theta_slot - math.pi
    output = PosCalcOutputS(selfCmd=MotionProfS())
    theta = join._theta_slot - math.pi
    state = _motion(
        east=join._loiter_center_e + join._loiter_radius * math.cos(theta),
        north=join._loiter_center_n + join._loiter_radius * math.sin(theta),
        h=join._slot.h,
        vd=20.0,
        v_psi=theta + math.pi / 2.0,
    )
    join.step(
        _rally_join_input(
            selfState=state,
            t_ref=200.0,
            t_ref_valid=True,
            t_now=100.0,
            assigned_loops=assigned_loops,
        ),
        output,
    )
    return join, output


def _rally_circle_state(join: RallyJoinPos, slot_remaining_angle: float) -> MotionProfS:
    """按到 M_i 的有向剩余角构造集结圆上的运动状态，负值表示刚越过 M_i。"""

    theta = join._theta_slot - slot_remaining_angle
    return _motion(
        east=join._loiter_center_e + join._loiter_radius * math.cos(theta),
        north=join._loiter_center_n + join._loiter_radius * math.sin(theta),
        h=join._slot.h,
        vd=20.0,
        v_psi=theta + math.pi / 2.0,
    )


def _cross_rally_slot(
    join: RallyJoinPos,
    output: PosCalcOutputS,
    *,
    assigned_loops: int,
    t_ref: float = 200.0,
    t_now: float = 100.0,
    t_ref_valid: bool = True,
) -> None:
    """先进入远角窗再真实经过 M_i，复用重复广播的同一圈数。"""

    far_theta = join._theta_slot - math.pi
    far_state = _motion(
        east=join._loiter_center_e + join._loiter_radius * math.cos(far_theta),
        north=join._loiter_center_n + join._loiter_radius * math.sin(far_theta),
        h=join._slot.h,
        vd=20.0,
        v_psi=far_theta + math.pi / 2.0,
    )
    join.step(
        _rally_join_input(
            selfState=far_state,
            t_ref=t_ref,
            t_ref_valid=t_ref_valid,
            t_now=t_now - 0.1,
            assigned_loops=assigned_loops,
        ),
        output,
    )
    near_state = _rally_circle_state(join, 0.1)
    join.step(
        _rally_join_input(
            selfState=near_state,
            t_ref=t_ref,
            t_ref_valid=t_ref_valid,
            t_now=t_now - 0.05,
            assigned_loops=assigned_loops,
        ),
        output,
    )
    crossed_state = _rally_circle_state(join, -0.1)
    join.step(
        _rally_join_input(
            selfState=crossed_state,
            t_ref=t_ref,
            t_ref_valid=t_ref_valid,
            t_now=t_now,
            assigned_loops=assigned_loops,
        ),
        output,
    )


def _drive_follower_across_rally_slot(
    follower: RallyFollowerEntity,
    *,
    start_now_s: float = 0.0,
) -> EntityOutputS:
    """从入圆开始驱动僚机完成远区布防、近窗确认和 M_i 跨零。"""

    join = _entity_rally_join(follower)
    states = (
        _rally_circle_state(join, 0.0),
        _rally_circle_state(join, math.pi),
        _rally_circle_state(join, 0.1),
        _rally_circle_state(join, -0.1),
    )
    output = EntityOutputS()
    for index, state in enumerate(states):
        output = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[_leader_msg(step=0, t_ref=1000.0, loop_counts={"R02": 0})],
                now_s=start_now_s + index * 0.1,
            ),
            output,
        )
    return output


class RallyJoinPosTests(unittest.TestCase):
    """验证集结专用位置解算单元。"""

    def _make_standby_join_for_transit(self) -> tuple[RallyJoinPos, MotionProfS, PosCalcOutputS]:
        """构造两个分离等半径圆，并把飞机放在待命圆顶部等待切出。"""

        return _make_standby_join_for_transit()

    def test_path_lengths_stay_unplanned_initially_during_standby_and_after_reset(self) -> None:
        """未开始规划、保持待命或复位后，基础航程及剩余航程都应保留未规划哨兵值。"""

        join = _new_join_for_transit()
        output = PosCalcOutputS(selfCmd=MotionProfS())
        self.assertEqual(join.planned_path_length_m, -1.0)
        self.assertEqual(join.remaining_path_length_m, -1.0)

        join.step(
            _rally_join_input(
                selfState=_motion(east=0.0, north=0.0, h=560.0, v_east=20.0, vd=20.0),
                standby=True,
            ),
            output,
        )

        self.assertEqual(join.state, RALLY_STATE_STANDBY)
        self.assertEqual(join.planned_path_length_m, -1.0)
        self.assertEqual(join.remaining_path_length_m, -1.0)

        join.reset()

        self.assertEqual(join.planned_path_length_m, -1.0)
        self.assertEqual(join.remaining_path_length_m, -1.0)

    def test_coordinated_speed_is_used_on_local_arc_tangent_line_and_rally_circle(self) -> None:
        """计划有效后完整剩余航程都应使用同一时间协调速度口径。"""

        for phase in ("ARC_TO_TANGENT", "LINE_TO_RALLY_ENTRY", "LOITERING"):
            with self.subTest(phase=phase):
                join, state, output = _join_at_transit_phase(phase)
                base_remaining = join._remaining_base_path_m(state.pos)
                circumference = 2.0 * math.pi * join._loiter_radius
                expected = max(14.0, min(25.0, (base_remaining + circumference) / 100.0))

                join.step(
                    _rally_join_input(
                        selfState=state,
                        t_ref=200.0,
                        t_ref_valid=True,
                        t_now=100.0,
                        assigned_loops=1,
                    ),
                    output,
                )

                self.assertAlmostEqual(output.selfCmd.v.vd, expected, places=6)

    def test_unplanned_transit_preserves_legacy_phase_speeds(self) -> None:
        """未锁存计划时待命圆、近场直飞和集结圆应保留既有速度行为。"""

        arc_join, arc_state, arc_output = _join_at_transit_phase("ARC_TO_TANGENT")
        arc_join._standby_speed = 22.0
        arc_join.step(_rally_join_input(selfState=arc_state, t_ref_valid=False), arc_output)
        self.assertAlmostEqual(arc_output.selfCmd.v.vd, 22.0)

        line_join, line_state, line_output = _join_at_transit_phase("LINE_TO_RALLY_ENTRY")
        assert line_join._entry_point is not None
        line_e = line_join._entry_point.east - line_state.pos.east
        line_n = line_join._entry_point.north - line_state.pos.north
        line_length = math.hypot(line_e, line_n)
        line_state.pos.east = line_join._entry_point.east - 50.0 * line_e / line_length
        line_state.pos.north = line_join._entry_point.north - 50.0 * line_n / line_length
        line_join.step(_rally_join_input(selfState=line_state, t_ref_valid=False), line_output)
        self.assertAlmostEqual(line_output.selfCmd.v.vd, 14.0)

        loiter_join, loiter_state, loiter_output = _join_at_transit_phase("LOITERING")
        loiter_join._loiter_speed = 18.0
        loiter_join.step(_rally_join_input(selfState=loiter_state, t_ref_valid=False), loiter_output)
        self.assertAlmostEqual(loiter_output.selfCmd.v.vd, 18.0)

    def test_slot_crossing_keeps_total_remaining_path_and_speed_continuous(self) -> None:
        """点前进入近窗不得扣圈，跨零后总剩余航程和协调速度只能按实际飞行量连续下降。"""

        join, output = _join_loitering_with_plan(assigned_loops=1)
        circumference = 2.0 * math.pi * join._loiter_radius
        samples: list[tuple[float, float]] = []
        for index, remaining_angle in enumerate((0.4, 0.2, -0.1)):
            state = _rally_circle_state(join, remaining_angle)
            t_now = 100.0 + index * 0.1
            join.step(
                _rally_join_input(
                    selfState=state,
                    t_ref=160.0,
                    t_ref_valid=True,
                    t_now=t_now,
                    assigned_loops=1,
                ),
                output,
            )
            total_remaining = join._remaining_base_path_m(state.pos) + join.remaining_loops * circumference
            expected_speed = max(14.0, min(25.0, total_remaining / (160.0 - t_now)))
            self.assertAlmostEqual(output.selfCmd.v.vd, expected_speed, places=6)
            samples.append((total_remaining, output.selfCmd.v.vd))

        self.assertAlmostEqual(samples[0][0] - samples[1][0], 0.2 * join._loiter_radius, places=6)
        self.assertAlmostEqual(samples[1][0] - samples[2][0], 0.3 * join._loiter_radius, places=6)
        self.assertLess(abs(samples[1][1] - samples[2][1]), 2.0)
        self.assertEqual(join.remaining_loops, 0)

    def test_slot_crossing_consumes_once_after_multiple_near_window_steps(self) -> None:
        """近窗内连续多拍不得提前扣圈，越点回绕只消费一次。"""

        join, output = _join_loitering_with_plan(assigned_loops=2)
        for index, remaining_angle in enumerate((0.3, 0.2, 0.1)):
            join.step(
                _rally_join_input(
                    selfState=_rally_circle_state(join, remaining_angle),
                    t_ref=200.0,
                    t_ref_valid=True,
                    t_now=100.1 + index * 0.1,
                    assigned_loops=2,
                ),
                output,
            )
            self.assertEqual(join.remaining_loops, 2)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, -0.05),
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.4,
                assigned_loops=2,
            ),
            output,
        )
        self.assertEqual(join.remaining_loops, 1)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, -0.1),
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.5,
                assigned_loops=2,
            ),
            output,
        )
        self.assertEqual(join.remaining_loops, 1)

    def test_single_tick_crossing_from_outside_near_window_exits_with_zero_loops(self) -> None:
        """零圈计划从 0.4rad 单拍推进到 -0.1rad 时，应识别真实跨零并立即切出。"""

        join, output = _join_loitering_with_plan(assigned_loops=0)
        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, 0.4),
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.1,
                assigned_loops=0,
            ),
            output,
        )
        self.assertEqual(join.state, RALLY_STATE_LOITERING)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, -0.1),
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.2,
                assigned_loops=0,
            ),
            output,
        )

        self.assertEqual(join.state, RALLY_STATE_EXITED)
        self.assertTrue(join.reached_slot_once)

    def test_single_tick_crossing_from_outside_near_window_consumes_one_loop(self) -> None:
        """多圈计划从 0.4rad 单拍跨到 -0.1rad 时，每拍只消费一圈。"""

        join, output = _join_loitering_with_plan(assigned_loops=2)
        for index, remaining_angle in enumerate((0.4, -0.1, -0.15)):
            join.step(
                _rally_join_input(
                    selfState=_rally_circle_state(join, remaining_angle),
                    t_ref=200.0,
                    t_ref_valid=True,
                    t_now=100.1 + index * 0.1,
                    assigned_loops=2,
                ),
                output,
            )

        self.assertEqual(join.state, RALLY_STATE_LOITERING)
        self.assertEqual(join.remaining_loops, 1)

    def test_slot_crossing_rejects_after_slot_entry_reverse_jump_and_near_noise(self) -> None:
        """点后近窗切入、反向跨零和近点抖动都不得伪造 CCW 真实越点事件。"""

        after_slot = _new_join_for_transit()
        after_slot._theta_entry = after_slot._theta_slot + 0.1
        after_slot._enter_arc()
        output = PosCalcOutputS(selfCmd=MotionProfS())
        for index, remaining_angle in enumerate((-0.1, -0.04, -0.08)):
            after_slot.step(
                _rally_join_input(
                    selfState=_rally_circle_state(after_slot, remaining_angle),
                    t_ref=200.0,
                    t_ref_valid=True,
                    t_now=100.0 + index * 0.1,
                    assigned_loops=1,
                ),
                output,
            )
        self.assertFalse(after_slot.reached_slot_once)
        self.assertEqual(after_slot.remaining_loops, 1)

        noise, noise_output = _join_loitering_with_plan(assigned_loops=1)
        for index, remaining_angle in enumerate((0.001, -0.001, 0.001)):
            noise.step(
                _rally_join_input(
                    selfState=_rally_circle_state(noise, remaining_angle),
                    t_ref=200.0,
                    t_ref_valid=True,
                    t_now=100.1 + index * 0.1,
                    assigned_loops=1,
                ),
                noise_output,
            )
        self.assertFalse(noise.reached_slot_once)
        self.assertEqual(noise.remaining_loops, 1)

        reverse, reverse_output = _join_loitering_with_plan(assigned_loops=2)
        for index, remaining_angle in enumerate((0.4, -0.1, 0.1)):
            reverse.step(
                _rally_join_input(
                    selfState=_rally_circle_state(reverse, remaining_angle),
                    t_ref=200.0,
                    t_ref_valid=True,
                    t_now=100.1 + index * 0.1,
                    assigned_loops=2,
                ),
                reverse_output,
            )
        self.assertEqual(reverse.remaining_loops, 1)
        self.assertEqual(reverse.state, RALLY_STATE_LOITERING)

    def test_assigned_loops_are_latched_once_and_consumed_once_per_slot_crossing(self) -> None:
        """重复广播不得重置圈数，每次 M_i 穿越只消耗一圈。"""

        join, output = _join_loitering_with_plan(assigned_loops=2)
        self.assertEqual(join.remaining_loops, 2)

        _cross_rally_slot(join, output, assigned_loops=2)
        self.assertEqual(join.remaining_loops, 1)
        self.assertEqual(join.state, RALLY_STATE_LOITERING)

        # 仍停留在近角窗的重复拍不构成第二次真实经过，也不能被重复计划重置。
        slot_state = _motion(east=join._slot.east, north=join._slot.north, h=join._slot.h)
        join.step(
            _rally_join_input(
                selfState=slot_state,
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.1,
                assigned_loops=2,
            ),
            output,
        )
        self.assertEqual(join.remaining_loops, 1)

        _cross_rally_slot(join, output, assigned_loops=2, t_now=101.0)
        self.assertEqual(join.remaining_loops, 0)
        self.assertEqual(join.state, RALLY_STATE_LOITERING)

        _cross_rally_slot(join, output, assigned_loops=2, t_now=102.0)
        self.assertEqual(join.state, RALLY_STATE_EXITED)

    def test_first_valid_plan_ignores_later_loop_assignment_changes(self) -> None:
        """首次有效计划锁存后，后续报文不得替换已分配圈数。"""

        join, output = _join_loitering_with_plan(assigned_loops=2)
        far_theta = join._theta_slot - math.pi
        far_state = _motion(
            east=join._loiter_center_e + join._loiter_radius * math.cos(far_theta),
            north=join._loiter_center_n + join._loiter_radius * math.sin(far_theta),
            h=join._slot.h,
        )
        join.step(
            _rally_join_input(
                selfState=far_state,
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.1,
                assigned_loops=7,
            ),
            output,
        )

        self.assertEqual(join.remaining_loops, 2)

    def test_final_slot_crossing_does_not_depend_on_dynamic_eta_freshness(self) -> None:
        """固定计划零圈时应在 M_i 可靠切出，不再依赖当前时刻是否越过 T_ref。"""

        join, output = _join_loitering_with_plan(assigned_loops=0)
        _cross_rally_slot(
            join,
            output,
            assigned_loops=0,
            t_ref=1000.0,
            t_now=100.2,
        )

        self.assertEqual(join.state, RALLY_STATE_EXITED)

    def test_slot_crossing_before_plan_does_not_consume_later_assignment(self) -> None:
        """计划生效前经过 M_i 不得提前消费之后下发的固定圈数。"""

        join, state, output = _join_at_transit_phase("LOITERING")
        join.step(_rally_join_input(selfState=state, t_ref_valid=False), output)
        _cross_rally_slot(join, output, assigned_loops=0, t_ref_valid=False)

        self.assertEqual(join.state, RALLY_STATE_LOITERING)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, math.pi),
                t_ref=200.0,
                t_ref_valid=True,
                t_now=100.0,
                assigned_loops=1,
            ),
            output,
        )

        self.assertEqual(join.remaining_loops, 1)
        _cross_rally_slot(join, output, assigned_loops=1, t_now=101.0)
        self.assertEqual(join.remaining_loops, 0)
        self.assertEqual(join.state, RALLY_STATE_LOITERING)

    def test_first_valid_plan_rejects_non_finite_times_without_latching(self) -> None:
        """首次有效计划要求 t_ref 和 t_now 都有限，失败后不得锁存圈数。"""

        invalid_times = (
            (float("nan"), 100.0),
            (float("inf"), 100.0),
            (float("-inf"), 100.0),
            (200.0, float("nan")),
            (200.0, float("inf")),
            (200.0, float("-inf")),
        )
        for t_ref, t_now in invalid_times:
            with self.subTest(t_ref=t_ref, t_now=t_now):
                join, state, output = _join_at_transit_phase("ARC_TO_TANGENT")
                with self.assertRaises(ValueError):
                    join.step(
                        _rally_join_input(
                            selfState=state,
                            t_ref=t_ref,
                            t_ref_valid=True,
                            t_now=t_now,
                            assigned_loops=2,
                        ),
                        output,
                    )
                self.assertEqual(join.remaining_loops, 0)

    def test_applied_plan_rejects_non_finite_times_on_every_coordinated_step(self) -> None:
        """计划生效后每次协调调速仍须拒绝非有限 t_ref 或 t_now。"""

        invalid_times = (
            (float("nan"), 100.1),
            (float("inf"), 100.1),
            (float("-inf"), 100.1),
            (200.0, float("nan")),
            (200.0, float("inf")),
            (200.0, float("-inf")),
        )
        for t_ref, t_now in invalid_times:
            with self.subTest(t_ref=t_ref, t_now=t_now):
                join, output = _join_loitering_with_plan(assigned_loops=2)
                with self.assertRaises(ValueError):
                    join.step(
                        _rally_join_input(
                            selfState=_rally_circle_state(join, math.pi),
                            t_ref=t_ref,
                            t_ref_valid=True,
                            t_now=t_now,
                            assigned_loops=2,
                        ),
                        output,
                    )
                self.assertEqual(join.remaining_loops, 2)

    def test_invalid_assigned_loops_are_rejected_by_first_valid_plan(self) -> None:
        """首次有效计划只接受非 bool 的非负整数圈数。"""

        for invalid_loops in (-1, 1.9, True):
            with self.subTest(assigned_loops=invalid_loops):
                join, state, output = _join_at_transit_phase("ARC_TO_TANGENT")
                with self.assertRaises(ValueError):
                    join.step(
                        _rally_join_input(
                            selfState=state,
                            t_ref=200.0,
                            t_ref_valid=True,
                            t_now=100.0,
                            assigned_loops=invalid_loops,  # type: ignore[arg-type]
                        ),
                        output,
                    )
                self.assertEqual(join.remaining_loops, 0)

    def test_remaining_path_length_updates_through_all_locked_transit_phases(self) -> None:
        """正常公切线路径应在三阶段保持非负剩余航程，并随沿途位置推进而更新。"""

        join, state, output = _started_join_with_two_circles()
        self.assertEqual(join._transit_phase, "ARC_TO_TANGENT")
        arc_start = join.remaining_path_length_m

        arc_theta = join._theta_local_exit - math.pi / 2.0
        state.pos.east = join._standby_center_e + join._loiter_radius * math.cos(arc_theta)
        state.pos.north = join._standby_center_n + join._loiter_radius * math.sin(arc_theta)
        state.v.vPsi = arc_theta + math.pi / 2.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.1), output)
        arc_progress = join.remaining_path_length_m

        assert join._local_exit_point is not None
        assert join._entry_point is not None
        state.pos.east = join._local_exit_point.east
        state.pos.north = join._local_exit_point.north
        state.v.vPsi = join._theta_local_exit + math.pi / 2.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.2), output)
        self.assertEqual(join._transit_phase, "LINE_TO_RALLY_ENTRY")
        line_start = join.remaining_path_length_m

        state.pos.east = (join._local_exit_point.east + join._entry_point.east) / 2.0
        state.pos.north = (join._local_exit_point.north + join._entry_point.north) / 2.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.3), output)
        line_progress = join.remaining_path_length_m

        state.pos.east = join._entry_point.east
        state.pos.north = join._entry_point.north
        state.pos.h = join._entry_point.h
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.4), output)
        self.assertEqual(join.state, RALLY_STATE_LOITERING)

        loiter_far_theta = join._theta_slot - math.pi
        state.pos.east = join._loiter_center_e + join._loiter_radius * math.cos(loiter_far_theta)
        state.pos.north = join._loiter_center_n + join._loiter_radius * math.sin(loiter_far_theta)
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.5), output)
        loiter_far = join.remaining_path_length_m

        loiter_near_theta = join._theta_slot - math.pi / 2.0
        state.pos.east = join._loiter_center_e + join._loiter_radius * math.cos(loiter_near_theta)
        state.pos.north = join._loiter_center_n + join._loiter_radius * math.sin(loiter_near_theta)
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.6), output)
        loiter_near = join.remaining_path_length_m

        for remaining in (arc_start, arc_progress, line_start, line_progress, loiter_far, loiter_near):
            self.assertGreaterEqual(remaining, 0.0)
        self.assertLess(arc_progress, arc_start)
        self.assertLess(line_progress, line_start)
        self.assertLess(loiter_near, loiter_far)

        join.reset()

        self.assertEqual(join.planned_path_length_m, -1.0)
        self.assertEqual(join.remaining_path_length_m, -1.0)

    def test_rally_join_pos_waits_for_local_tangent_before_flying_line(self) -> None:
        """验证开始集结后先沿待命圆飞向切出点，不从当前位置直接追集结圆。"""

        join, state, output = self._make_standby_join_for_transit()

        join.step(
            _rally_join_input(selfState=state, standby=False, t_now=10.0),
            output,
        )

        assert output.selfCmd is not None
        self.assertEqual(join.state, RALLY_STATE_FLYING)
        self.assertAlmostEqual(output.selfCmd.pos.east, 0.0)
        self.assertAlmostEqual(output.selfCmd.pos.north, 400.0)
        self.assertAlmostEqual(output.selfCmd.v.vEast, -20.0)
        self.assertAlmostEqual(output.selfCmd.v.vNorth, 0.0, places=6)
        self.assertAlmostEqual(output.selfCmd.v.dVPsi, 0.1)

    def test_planned_path_length_contains_local_arc_tangent_and_rally_arc(self) -> None:
        """两圆公切线方案应锁存完整基础水平航程。"""
        join, state, output = _started_join_with_two_circles()
        self.assertGreater(join.planned_path_length_m, join._tangent_length_m)
        self.assertAlmostEqual(join.remaining_path_length_m, join.planned_path_length_m, delta=1e-6)

    def test_planned_path_length_uses_point_to_circle_fallback(self) -> None:
        """公切线无解时基础航程应与实际点到圆退化路线一致。"""
        join, expected_length = _started_join_with_point_fallback()
        self.assertAlmostEqual(join.planned_path_length_m, expected_length, places=6)

    def test_planned_path_length_for_coincident_circles_is_rally_arc_only(self) -> None:
        """两圆重合时只保留当前位置到 M_i 的 CCW 圆弧。"""
        join, expected_arc = _started_join_with_coincident_circles()
        self.assertAlmostEqual(join.planned_path_length_m, expected_arc, places=6)

    def test_rally_join_pos_same_circle_enters_loitering_directly(self) -> None:
        """验证待命圆与集结圆重合时直接继续盘旋，不退化为直飞 M_i。"""

        join = RallyJoinPos()
        join.init(RallyJoinPosInitS(
            loose_slot=_pos(0.0, 0.0, 560.0),
            approach_speed_mps=20.0,
            arrival_radius_m=20.0,
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
            _rally_join_input(
                selfState=_motion(east=0.0, north=0.0, h=560.0, v_east=20.0, vd=20.0),
                standby=True,
            ),
            output,
        )
        same_circle_state = _motion(
            east=0.0,
            north=400.0,
            h=560.0,
            v_east=-20.0,
            vd=20.0,
            v_psi=math.pi,
        )

        join.step(
            _rally_join_input(selfState=same_circle_state, standby=False, t_now=5.0),
            output,
        )

        assert output.selfCmd is not None
        self.assertEqual(join.state, RALLY_STATE_LOITERING)
        self.assertIsNone(join._local_exit_point)
        self.assertAlmostEqual(output.selfCmd.pos.east, 0.0)
        self.assertAlmostEqual(output.selfCmd.pos.north, 400.0)
        self.assertAlmostEqual(output.selfCmd.v.dVPsi, 14.0 / 200.0)

    def test_rally_join_pos_common_tangent_is_tangent_and_locked(self) -> None:
        """验证两圆切点同时相切，且规划后不会随实时位置重新计算。"""

        join, state, output = self._make_standby_join_for_transit()
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)

        assert join._local_exit_point is not None
        assert join._entry_point is not None
        local_exit = join._local_exit_point
        rally_entry = join._entry_point
        line_e = rally_entry.east - local_exit.east
        line_n = rally_entry.north - local_exit.north
        local_radius_e = local_exit.east - join._standby_center_e
        local_radius_n = local_exit.north - join._standby_center_n
        rally_radius_e = rally_entry.east - join._loiter_center_e
        rally_radius_n = rally_entry.north - join._loiter_center_n
        self.assertAlmostEqual(math.hypot(local_radius_e, local_radius_n), 200.0)
        self.assertAlmostEqual(math.hypot(rally_radius_e, rally_radius_n), 200.0)
        self.assertAlmostEqual(line_e * local_radius_e + line_n * local_radius_n, 0.0)
        self.assertAlmostEqual(line_e * rally_radius_e + line_n * rally_radius_n, 0.0)
        locked = (local_exit.east, local_exit.north, rally_entry.east, rally_entry.north)

        state.pos.east += 50.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.1), output)

        assert join._local_exit_point is not None
        assert join._entry_point is not None
        self.assertEqual(locked, (
            join._local_exit_point.east,
            join._local_exit_point.north,
            join._entry_point.east,
            join._entry_point.north,
        ))

    def test_rally_join_pos_ten_degree_window_switches_to_locked_tangent(self) -> None:
        """验证进入待命圆切出点 10° 窗口后，同一拍切换到锁存公切线。"""

        join, state, output = self._make_standby_join_for_transit()
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)
        theta = math.radians(-95.0)
        state.pos.east = join._standby_center_e + 200.0 * math.cos(theta)
        state.pos.north = join._standby_center_n + 200.0 * math.sin(theta)
        state.v.vPsi = theta + math.pi / 2.0

        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.1), output)

        assert output.selfCmd is not None
        assert join._entry_point is not None
        self.assertEqual(join._transit_phase, "LINE_TO_RALLY_ENTRY")
        self.assertAlmostEqual(output.selfCmd.pos.east, join._entry_point.east)
        self.assertAlmostEqual(output.selfCmd.pos.north, join._entry_point.north)
        heading_jump = abs(math.atan2(
            math.sin(output.selfCmd.v.vPsi - state.v.vPsi),
            math.cos(output.selfCmd.v.vPsi - state.v.vPsi),
        ))
        self.assertLessEqual(heading_jump, math.radians(10.0))

    def test_rally_join_pos_crossing_local_tangent_does_not_wait_another_lap(self) -> None:
        """验证离散步进跨过切出点时仍立即转直线，不因漏过角度窗口多绕一圈。"""

        join, state, output = self._make_standby_join_for_transit()
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)

        before_exit = math.radians(-105.0)
        state.pos.east = join._standby_center_e + 200.0 * math.cos(before_exit)
        state.pos.north = join._standby_center_n + 200.0 * math.sin(before_exit)
        state.v.vPsi = before_exit + math.pi / 2.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.1), output)
        self.assertEqual(join._transit_phase, "ARC_TO_TANGENT")

        after_exit = math.radians(-75.0)
        state.pos.east = join._standby_center_e + 200.0 * math.cos(after_exit)
        state.pos.north = join._standby_center_n + 200.0 * math.sin(after_exit)
        state.v.vPsi = after_exit + math.pi / 2.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.2), output)

        assert output.selfCmd is not None
        assert join._entry_point is not None
        self.assertEqual(join._transit_phase, "LINE_TO_RALLY_ENTRY")
        self.assertAlmostEqual(output.selfCmd.pos.east, join._entry_point.east)
        self.assertAlmostEqual(output.selfCmd.pos.north, join._entry_point.north)

    def test_rally_join_pos_common_tangent_failure_uses_locked_point_to_circle_tangent(self) -> None:
        """验证两圆公切线无解时只在开始集结一拍计算当前点到集结圆切线。"""

        join, state, output = self._make_standby_join_for_transit()
        with patch(
            "src.algorithm.units.algo.pos_calc.rally_join_pos.common_tangent",
            return_value=None,
        ):
            join.step(_rally_join_input(selfState=state, standby=False, t_now=10.0), output)
        assert join._entry_point is not None
        entry = (join._entry_point.east, join._entry_point.north)
        self.assertEqual(join._transit_phase, "LINE_TO_RALLY_ENTRY")
        self.assertIsNone(join._local_exit_point)

        state.pos.east += 100.0
        join.step(_rally_join_input(selfState=state, standby=False, t_now=10.1), output)

        assert join._entry_point is not None
        self.assertEqual(entry, (join._entry_point.east, join._entry_point.north))

    def test_rally_join_pos_reset_clears_locked_transit_plan(self) -> None:
        """验证 reset 清除公切线和圈数计划，下一轮开始集结时能够重新规划。"""

        join, state, output = self._make_standby_join_for_transit()
        join.step(
            _rally_join_input(
                selfState=state,
                standby=False,
                t_ref=200.0,
                t_ref_valid=True,
                t_now=10.0,
                assigned_loops=2,
            ),
            output,
        )
        self.assertIsNotNone(join._local_exit_point)
        self.assertIsNotNone(join._entry_point)
        self.assertEqual(join.remaining_loops, 2)

        join.reset()

        self.assertIsNone(join._transit_phase)
        self.assertIsNone(join._local_exit_point)
        self.assertIsNone(join._entry_point)
        self.assertEqual(join._tangent_length_m, 0.0)
        self.assertIsNone(join._last_local_remaining_angle)
        self.assertEqual(join.remaining_loops, 0)

        join.step(
            _rally_join_input(
                selfState=state,
                t_ref=300.0,
                t_ref_valid=True,
                t_now=20.0,
                assigned_loops=1,
            ),
            output,
        )

        self.assertEqual(join.remaining_loops, 1)

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
            _rally_join_input(
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
            _rally_join_input(
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
                    _rally_join_input(
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

    def test_slot_geometry_uses_fixed_unscaled_offset(self) -> None:
        """验证取消缩放后，槽位位置和速度始终按固定队形解算。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
        ctx.selfState = _motion(east=80.0, north=210.0, h=500.0, v_east=20.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.RALLY, pattern=0, step=2)
        comm_init = _comm_init()
        slot = SlotGeometry()
        slot.init(SlotGeometryInitS("R02", comm_init.formPat, comm_init.formPos))

        slot.step(
            PosCalcInputS(
                selfState=ctx.selfState,
                leaderState=ctx.leaderState,
                cmd=ctx.cmd,
            ),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 90.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 205.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 500.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 20.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_slot_geometry_applies_fixed_three_dimensional_offset(self) -> None:
        """验证取消缩放后，水平和高度偏置均按固定槽位一次叠加。"""

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
        slot = SlotGeometry()
        slot.init(SlotGeometryInitS("R02", comm_init_alt.formPat, comm_init_alt.formPos))

        slot.step(
            PosCalcInputS(
                selfState=ctx.selfState,
                leaderState=ctx.leaderState,
                cmd=ctx.cmd,
            ),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.h, 530.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.east, 90.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 205.0)

    def test_slot_geometry_default_call_uses_unscaled_slot(self) -> None:
        """验证 SlotGeometry 默认按未缩放的固定槽位工作。"""

        comm_init = _comm_init()
        slot = SlotGeometry()
        out = PosCalcOutputS(selfCmd=MotionProfS())
        slot.init(SlotGeometryInitS("R02", comm_init.formPat, comm_init.formPos))
        slot.step(
            PosCalcInputS(
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
        join.step(_rally_join_input(selfState=_motion(east=2000.0, north=300.0, h=500.0), t_ref_valid=False), out)

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
            join.step(_rally_join_input(selfState=state, t_ref=1e9, t_ref_valid=False, t_now=t_now), out)
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
            join.step(_rally_join_input(selfState=state, t_ref=0.0, t_ref_valid=True, t_now=t_now), out)
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
            join.step(_rally_join_input(selfState=state, t_ref=0.0, t_ref_valid=True, t_now=t_now), out)
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
            join.step(_rally_join_input(selfState=state, t_ref_valid=False, t_now=t_now), out)
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
        join.step(_rally_join_input(selfState=_motion(east=-2000.0, north=0.0, h=500.0), t_ref_valid=False), out)
        # 强制切到 LOITERING，模拟飞机当前实际半径为 150m（偏离期望的 200m）而非精确在切入点上。
        join._state = RALLY_STATE_LOITERING
        join._away_from_slot = True
        theta = math.radians(30.0)
        actual_radius = 150.0
        pos_e = join._loiter_center_e + actual_radius * math.cos(theta)
        pos_n = join._loiter_center_n + actual_radius * math.sin(theta)
        join.step(_rally_join_input(selfState=_motion(east=pos_e, north=pos_n, h=500.0), t_ref_valid=False), out)

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
        join.step(_rally_join_input(selfState=_motion(east=10.0, north=20.0, h=500.0), t_ref_valid=False), out)

        self.assertEqual(join._entry_point.east, 10.0)
        self.assertEqual(join._entry_point.north, 20.0)

    def test_point_fallback_at_slot_exits_on_first_real_crossing(self) -> None:
        """点到圆退化到 M_i 时首拍不得误切，下一拍真实跨零应立即按零圈计划切出。"""

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
        output = PosCalcOutputS(selfCmd=MotionProfS())
        join.step(
            _rally_join_input(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                t_ref=100.0,
                t_ref_valid=True,
                t_now=0.0,
                assigned_loops=0,
            ),
            output,
        )

        assert join._entry_point is not None
        self.assertEqual(join._entry_point, join._slot)
        self.assertEqual(join.state, RALLY_STATE_LOITERING)
        self.assertFalse(join.reached_slot_once)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, -0.1),
                t_ref=100.0,
                t_ref_valid=True,
                t_now=0.1,
                assigned_loops=0,
            ),
            output,
        )

        self.assertEqual(join.state, RALLY_STATE_EXITED)
        self.assertTrue(join.reached_slot_once)

    def test_point_before_near_window_exits_on_first_real_crossing(self) -> None:
        """点前约 0.5rad 切入时应保留 away 布防，并在首次真实跨零按零圈计划切出。"""

        join = _new_join_for_transit()
        join._theta_entry = join._theta_slot - 0.5
        join._enter_arc()
        output = PosCalcOutputS(selfCmd=MotionProfS())

        for index, remaining_angle in enumerate((0.5, 0.2)):
            join.step(
                _rally_join_input(
                    selfState=_rally_circle_state(join, remaining_angle),
                    t_ref=100.0,
                    t_ref_valid=True,
                    t_now=index * 0.1,
                    assigned_loops=0,
                ),
                output,
            )
            self.assertEqual(join.state, RALLY_STATE_LOITERING)
            self.assertFalse(join.reached_slot_once)

        join.step(
            _rally_join_input(
                selfState=_rally_circle_state(join, -0.05),
                t_ref=100.0,
                t_ref_valid=True,
                t_now=0.2,
                assigned_loops=0,
            ),
            output,
        )

        self.assertEqual(join.state, RALLY_STATE_EXITED)
        self.assertTrue(join.reached_slot_once)


class RallyEntityTests(unittest.TestCase):
    """验证集结长机和僚机实体的主链路。"""

    def test_follower_reset_does_not_restore_plan_from_empty_inbox(self) -> None:
        """有效计划后复位僚机实体，下一拍空 inbox 不得回灌旧 T_ref 和圈数。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        state = _motion(east=10.0, north=20.0, h=500.0, v_east=20.0)
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[_leader_msg(t_ref=180.0, t_ref_valid=True, loop_counts={"R02": 2})],
            ),
            EntityOutputS(),
        )
        self.assertTrue(follower.cxt.rally_t_ref_valid)
        self.assertEqual(follower.cxt.rally_loop_counts.get("R02"), 2)

        follower.reset()
        follower.step(EntityInputS(selfState=state, inbox=[]), EntityOutputS())

        self.assertEqual(follower.cxt.rally_t_ref, 0.0)
        self.assertFalse(follower.cxt.rally_t_ref_valid)
        self.assertEqual(follower.cxt.rally_loop_counts, {})

    def test_five_aircraft_entities_lock_plan_consume_loops_and_enter_catchup(self) -> None:
        """五实体应经真实消息链路锁存计划、按圈切出并用回传推进 CATCHUP。"""

        node_ids = ("A01", "A02", "A03", "A04", "A05")
        comm_init = _comm_init_five()
        route = _route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0))
        rally_cfg = _rally_cfg(expected=node_ids[1:], dt_s=0.1)
        states = {
            "A01": _motion(east=0.0, north=0.0, h=500.0, v_east=20.0, vd=20.0, v_psi=0.0),
            "A02": _motion(east=-500.0, north=100.0, h=500.0, v_east=20.0, vd=20.0, v_psi=0.0),
            "A03": _motion(east=-1500.0, north=-100.0, h=500.0, v_east=20.0, vd=20.0, v_psi=0.0),
            "A04": _motion(east=-2500.0, north=200.0, h=500.0, v_east=20.0, vd=20.0, v_psi=0.0),
            "A05": _motion(east=-3500.0, north=-200.0, h=500.0, v_east=20.0, vd=20.0, v_psi=0.0),
        }
        leader = RallyLeaderEntity()
        leader.init(EntityInitS(
            selfInit=FormSelfInitS("A01"), commInit=comm_init, route=route, rally_cfg=rally_cfg,
        ))
        followers: dict[str, RallyFollowerEntity] = {}
        for node_id in node_ids[1:]:
            follower = RallyFollowerEntity()
            follower.init(EntityInitS(
                selfInit=FormSelfInitS(node_id), commInit=comm_init, route=route,
                rally_cfg=rally_cfg, rally_leader_id="A01",
            ))
            followers[node_id] = follower

        leader.step(EntityInputS(
            selfState=states["A01"], remote=RemoteCmdS(FormStageE.STANDBY), now_s=0.0,
        ), EntityOutputS())
        for node_id, follower in followers.items():
            follower.step(EntityInputS(
                selfState=states[node_id], remote=RemoteCmdS(FormStageE.STANDBY), now_s=0.0,
            ), EntityOutputS())

        leader_join = EntityOutputS()
        leader.step(EntityInputS(
            selfState=states["A01"], remote=RemoteCmdS(FormStageE.RALLY), now_s=0.1,
        ), leader_join)
        follower_reports = []
        for node_id, follower in followers.items():
            output = EntityOutputS()
            follower.step(EntityInputS(
                selfState=states[node_id], inbox=list(leader_join.outbox), now_s=0.1,
            ), output)
            follower_reports.extend(output.outbox)
            self.assertGreaterEqual(_entity_rally_join(follower).planned_path_length_m, 0.0, msg=node_id)

        plan_output = EntityOutputS()
        leader.step(EntityInputS(
            selfState=states["A01"], inbox=follower_reports,
            remote=RemoteCmdS(FormStageE.RALLY), now_s=0.2,
        ), plan_output)
        self.assertEqual(len(plan_output.outbox), 1)
        plan_message = list(plan_output.outbox)
        plan_payload = plan_message[0].payload
        self.assertTrue(plan_payload["t_ref_valid"])
        self.assertEqual(plan_payload["loop_counts"], {"A01": 3, "A02": 1, "A03": 0, "A04": 0, "A05": 0})
        self.assertIsNot(leader._outbound_u.loop_counts, leader._task_y.loopCounts)
        locked_t_ref = plan_payload["t_ref"]
        locked_loop_counts = dict(plan_payload["loop_counts"])

        follower_exit_reports = []
        for index, node_id in enumerate(node_ids[1:]):
            follower = followers[node_id]
            join = _entity_rally_join(follower)
            assert join._entry_point is not None
            assert join._local_exit_point is not None
            follower.step(EntityInputS(
                selfState=_motion(
                    east=join._local_exit_point.east, north=join._local_exit_point.north,
                    h=join._local_exit_point.h, v_east=20.0, vd=20.0,
                    v_psi=join._theta_local_exit + math.pi / 2.0,
                ),
                inbox=plan_message,
                now_s=0.9 + index * 10.0,
            ), EntityOutputS())
            entry_output = EntityOutputS()
            follower.step(EntityInputS(
                selfState=_motion(
                    east=join._entry_point.east, north=join._entry_point.north, h=join._entry_point.h,
                    v_east=20.0, vd=20.0,
                ),
                inbox=plan_message,
                now_s=1.0 + index * 10.0,
            ), entry_output)
            self.assertTrue(follower.cxt.rally_t_ref_valid, msg=node_id)
            self.assertEqual(join.remaining_loops, locked_loop_counts[node_id], msg=node_id)
            self.assertEqual(join.state, RALLY_STATE_LOITERING, msg=node_id)

            for crossing in range(locked_loop_counts[node_id] + 1):
                now_s = 1.1 + index * 10.0 + crossing
                for offset, angle in ((0.0, math.pi), (0.1, 0.1), (0.2, -0.1)):
                    crossing_output = EntityOutputS()
                    follower.step(EntityInputS(
                        selfState=_rally_circle_state(join, angle), inbox=plan_message, now_s=now_s + offset,
                    ), crossing_output)
                expected_remaining = locked_loop_counts[node_id] - crossing - 1
                if expected_remaining >= 0:
                    self.assertEqual(join.remaining_loops, expected_remaining, msg=node_id)
                    self.assertEqual(join.state, RALLY_STATE_LOITERING, msg=node_id)
                else:
                    self.assertEqual(join.state, RALLY_STATE_EXITED, msg=node_id)
                    self.assertEqual(crossing_output.outbox[0].payload["rally_state"], RALLY_STATE_EXITED)
            follower_exit_reports.extend(crossing_output.outbox)

        leader_join_state = _entity_rally_join(leader)
        self.assertEqual(leader_join_state.state, RALLY_STATE_LOITERING)
        for crossing in range(locked_loop_counts["A01"] + 1):
            now_s = 50.0 + crossing
            for offset, angle in ((0.0, math.pi), (0.1, 0.1), (0.2, -0.1)):
                leader_crossing = EntityOutputS()
                leader.step(EntityInputS(
                    selfState=_rally_circle_state(leader_join_state, angle), inbox=follower_exit_reports,
                    remote=RemoteCmdS(FormStageE.RALLY), now_s=now_s + offset,
                ), leader_crossing)
            expected_remaining = locked_loop_counts["A01"] - crossing - 1
            if expected_remaining >= 0:
                self.assertEqual(leader_join_state.remaining_loops, expected_remaining)
                self.assertEqual(leader_join_state.state, RALLY_STATE_LOITERING)
            else:
                self.assertEqual(leader_join_state.state, RALLY_STATE_EXITED)

        completed = EntityOutputS()
        leader.step(EntityInputS(
            selfState=_rally_circle_state(leader_join_state, 0.0), inbox=follower_exit_reports,
            remote=RemoteCmdS(FormStageE.RALLY), now_s=60.0,
        ), completed)
        self.assertEqual(leader.cxt.cmd.step, RallyPhaseE.CATCHUP)
        self.assertTrue(completed.outbox[0].payload["t_ref_valid"])
        self.assertEqual(completed.outbox[0].payload["t_ref"], locked_t_ref)
        self.assertEqual(completed.outbox[0].payload["loop_counts"], locked_loop_counts)

    def test_follower_does_not_accept_plan_without_own_loop_assignment(self) -> None:
        """广播缺少本机圈数时僚机不得把计划视为可执行。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        leader_plan = _leader_msg(t_ref=180.0, t_ref_valid=True)
        leader_plan.payload["loop_counts"] = {"R01": 0}

        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[leader_plan],
            ),
            EntityOutputS(),
        )

        self.assertFalse(follower.cxt.rally_t_ref_valid)
        self.assertNotIn("R02", follower.cxt.rally_loop_counts)

    def test_follower_receives_own_fixed_loop_assignment(self) -> None:
        """僚机仅在收到本机非负圈数时启用同步计划并接线到位置解算。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        leader_plan = _leader_msg(t_ref=180.0, t_ref_valid=True)
        leader_plan.payload["loop_counts"] = {"R01": 0, "R02": 2}

        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[leader_plan],
            ),
            EntityOutputS(),
        )

        self.assertTrue(follower.cxt.rally_t_ref_valid)
        self.assertEqual(follower.cxt.rally_loop_counts.get("R02"), 2)

    def test_rally_follower_joining_uses_speed_only_forward_control(self) -> None:
        """JOINING 前向通道只跟踪协调速度，进入 CATCHUP 后恢复位置跟踪。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        state = _motion(
            east=-2000.0,
            north=0.0,
            h=500.0,
            v_east=20.0,
            vd=20.0,
            v_psi=0.0,
        )

        joining = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[_leader_msg(t_ref=500.0, loop_counts={"R02": 0})],
                now_s=0.0,
            ),
            joining,
        )

        assert joining.selfCmd is not None
        assert joining.selfAccCmd is not None
        joining_heading = joining.selfCmd.v.vPsi
        joining_acc_forward = (
            joining.selfAccCmd.accEast * math.cos(joining_heading)
            + joining.selfAccCmd.accNorth * math.sin(joining_heading)
        )
        self.assertLess(joining.selfCmd.v.vd, state.v.vd)
        self.assertLess(joining_acc_forward, 0.0)

        catchup = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[
                    _leader_msg(
                        step=int(RallyPhaseE.CATCHUP),
                        leader_state=_motion(east=100.0, north=0.0, h=500.0, v_east=20.0),
                    )
                ],
                now_s=0.1,
            ),
            catchup,
        )

        assert catchup.selfCmd is not None
        assert catchup.selfAccCmd is not None
        catchup_heading = catchup.selfCmd.v.vPsi
        catchup_acc_forward = (
            catchup.selfAccCmd.accEast * math.cos(catchup_heading)
            + catchup.selfAccCmd.accNorth * math.sin(catchup_heading)
        )
        self.assertGreater(catchup_acc_forward, 0.0)

    def test_rally_follower_standby_loiters_and_broadcasts_from_entity_layer(self) -> None:
        """验证僚机实体在 STANDBY 阶段自行本地盘旋，并继续发送待命回报。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
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
        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_STANDBY)
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
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
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
        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_STANDBY)

    def test_rally_follower_catchup_keeps_layered_altitude_before_loose(self) -> None:
        """验证 CATCHUP 阶段仍保持分层高度，直到进入 LOOSE 后再收敛到正常槽位高度。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
                rally_layer_altitude_m=560.0,
            )
        )

        catchup = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=1, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            catchup,
        )
        assert catchup.selfCmd is not None
        catchup_height_m = catchup.selfCmd.pos.h
        loose = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=500.0, v_east=20.0),
                inbox=[_leader_msg(step=2, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            loose,
        )

        assert loose.selfCmd is not None
        self.assertAlmostEqual(catchup_height_m, 560.0)
        self.assertAlmostEqual(loose.selfCmd.pos.h, 500.0)

    def test_rally_follower_latches_arrival_and_uses_fixed_slot_after_step_one(self) -> None:
        """验证僚机到达松散点后进入 EXITED 上报，并在后续阶段使用固定槽位。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )

        first = _drive_follower_across_rally_slot(follower)
        # 僚机真实越过目标点且固定计划为零圈 → RallyJoinPos 进入 EXITED，上报 arrived=1。
        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_EXITED)
        self.assertEqual(first.outbox[0].payload["arrived"], 1)
        self.assertEqual(first.outbox[0].payload["rally_state"], RALLY_STATE_EXITED)

        second = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=10.0, north=20.0, h=490.0, v_east=20.0),
                inbox=[_leader_msg(step=1, leader_state=_motion(east=100.0, north=200.0, h=500.0, v_east=20.0))],
            ),
            second,
        )

        assert second.selfCmd is not None
        # R02 固定槽位偏置 (x=-10,z=-5) 在东向航迹下投影为 (east=-10,north=5)。
        self.assertAlmostEqual(second.selfCmd.pos.east, 90.0)
        self.assertAlmostEqual(second.selfCmd.pos.north, 205.0)
        # 长机沿东向以 20 m/s 直飞（无偏航角速率），槽位只透传自身速度前馈；
        # CATCHUP 不得再按位置误差额外调速，追赶速度修正由 PidCompose 前向外环生成。
        self.assertAlmostEqual(second.selfCmd.v.vEast, 20.0)
        self.assertAlmostEqual(second.selfCmd.v.vd, 20.0)
        self.assertAlmostEqual(second.selfCmd.v.vPsi, 0.0)
        # CATCHUP 门控统一使用到真实槽位的三维距离，高度误差也必须计入 pos_err_m。
        self.assertAlmostEqual(second.outbox[0].payload["pos_err_m"], math.sqrt(80.0**2 + 185.0**2 + 10.0**2))
        self.assertEqual(second.outbox[0].target, "R01")

    def test_rally_follower_waits_when_t_ref_is_not_valid_at_cold_start(self) -> None:
        """验证冷启动尚无有效 T_ref 时，已到目标点的僚机进入盘旋而不是直接切出。"""
        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
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

        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_LOITERING)
        self.assertEqual(output.outbox[0].payload["arrived"], 0)

    def test_rally_follower_none_clears_join_state_for_compatible_stop(self) -> None:
        """验证僚机进入兼容 NONE 停控时清除 EXITED 位置状态，但不声明新任务生命周期。"""
        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
                rally_leader_id="R01",
            )
        )
        _drive_follower_across_rally_slot(follower)
        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_EXITED)

        state = _motion(east=10.0, north=20.0, h=500.0, v_east=20.0)
        none_output = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=state,
                inbox=[_leader_msg(stage=FormStageE.NONE, pattern=0)],
            ),
            none_output,
        )

        self.assertEqual(follower.cxt.posCalcStatus.rally_state, RALLY_STATE_FLYING)
        self.assertEqual(none_output.outbox[0].payload["arrived"], 0)

    def test_rally_follower_none_outputs_current_position_zero_velocity(self) -> None:
        """验证僚机收到 NONE 时输出当前位置零速并清到达上报。"""

        follower = RallyFollowerEntity()
        follower.init(
            EntityInitS(
                selfInit=FormSelfInitS("R02"),
                commInit=_comm_init(),
                route=_route((40.0, 5.0, 500.0), (100.0, 5.0, 500.0)),
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
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",), dt_s=0.1),
            )
        )
        # 本用例只验证完成分析输出，直接把汇合子状态置为已切出，避免固定位置夹具无法模拟完整盘旋轨迹。
        _entity_rally_join(leader)._state = RALLY_STATE_EXITED
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

    def test_rally_leader_none_clears_join_and_completion_latches(self) -> None:
        """验证长机进入兼容 NONE 停控时清理位置与完成显示锁存。"""
        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
                rally_cfg=_rally_cfg(expected=("R02",)),
            )
        )
        leader.cxt.cmd.stage = FormStageE.HOLD
        _entity_rally_join(leader)._state = RALLY_STATE_EXITED
        leader._rally_completed = True

        leader.step(
            EntityInputS(
                selfState=_motion(east=100.0, north=0.0, h=500.0, v_east=20.0),
                remote=RemoteCmdS(FormStageE.NONE),
                now_s=1.0,
            ),
            EntityOutputS(),
        )

        self.assertEqual(leader.cxt.posCalcStatus.rally_state, RALLY_STATE_FLYING)
        self.assertFalse(leader._rally_completed)

    def test_rally_leader_standby_parses_follower_status_and_broadcasts(self) -> None:
        """验证长机待命阶段正常解析僚机回报，并继续广播 STANDBY 阶段。"""

        leader = RallyLeaderEntity()
        leader.init(
            EntityInitS(
                selfInit=FormSelfInitS("R01"),
                commInit=_comm_init(),
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
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
        self.assertEqual(leader.cxt.posCalcStatus.rally_state, RALLY_STATE_STANDBY)
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
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
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
                route=_route((0.0, 0.0, 500.0), (200.0, 0.0, 500.0)),
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
        self.assertEqual(leader.cxt.posCalcStatus.rally_state, RALLY_STATE_STANDBY)
        self.assertEqual([state.id for state in leader.cxt.followerStates], ["R02"])

    def test_rally_leader_init_rejects_empty_route_list(self) -> None:
        """验证 route=[] 时抛出 ValueError 而非 IndexError。"""
        with self.assertRaises(ValueError):
            RallyLeaderEntity().init(
                EntityInitS(
                    selfInit=FormSelfInitS("R01"),
                    commInit=_comm_init(),
                    route=[],  # 空列表，不是 None，守卫应捕获
                    rally_cfg=_rally_cfg(expected=()),
                )
            )

    def test_route_heading_rejects_horizontally_degenerate_first_segment(self) -> None:
        """回归用例：A/A1 水平坐标重合（仅高度不同也算）时必须显式报错，不能静默按 atan2(0,0) 退化为正东。"""
        from src.algorithm.units.algo.pos_calc.rally_join_pos import route_heading_rad

        with self.assertRaises(ValueError):
            route_heading_rad(_route((0.0, 0.0, 500.0), (0.0, 0.0, 520.0)))

    def test_rally_leader_init_rejects_horizontally_degenerate_route(self) -> None:
        """验证长机 init 时拒绝水平退化的统一 route 第一航段。"""
        with self.assertRaises(ValueError):
            RallyLeaderEntity().init(
                EntityInitS(
                    selfInit=FormSelfInitS("R01"),
                    commInit=_comm_init(),
                    route=_route((0.0, 0.0, 500.0), (0.0, 0.0, 520.0)),
                    rally_cfg=_rally_cfg(expected=()),
                )
            )


if __name__ == "__main__":
    unittest.main()
