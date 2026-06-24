"""Low-level tests for the formation algorithm package."""

from __future__ import annotations

import math
import unittest

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    AccInEarthS,
    CommDirE,
    FormCommInitS,
    FormPatE,
    FormPosS,
    FormSelfInitS,
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
    RemoteCmdS,
    RouteS,
    VdInEarthS,
    WayLineS,
    WayPointS,
)
from src.algorithm.entity.leader_follower_hold.follower import FollowerEntity
from src.algorithm.entity.leader_follower_hold.leader import LeaderEntity
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.ctrl.pid import Pid
from src.algorithm.units.algo.formation_math import clamp, enu_to_track, horizontal_track_to_enu, track_to_enu
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS, RouteInterpInputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS
from src.algorithm.units.process.formation_task.base import FormationTaskInputS, FormationTaskOutputS
from src.algorithm.units.process.formation_task.hold import Hold
from src.algorithm.units.process.inbound.base import InboundInputS, InboundOutputS
from src.algorithm.units.process.inbound.leader_follower import LeaderFollower
from src.algorithm.units.process.outbound.base import OutboundInputS, OutboundOutputS
from src.algorithm.units.process.outbound.leader_broadcast import LeaderBroadcast, OutboundInitS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute, LeaderRouteInitS
from src.algorithm.units.process.tra_plan.noop import Noop
from src.common.envelope import MessageEnvelope


def _motion(
    east: float = 0.0,
    north: float = 0.0,
    h: float = 0.0,
    v_east: float = 0.0,
    v_north: float = 0.0,
    v_up: float = 0.0,
    d_vpsi: float = 0.0,
) -> MotionProfS:
    return MotionProfS(
        pos=PosInEarthS(east=east, north=north, h=h),
        v=VdInEarthS(
            vEast=v_east,
            vNorth=v_north,
            vUp=v_up,
            vd=(v_east * v_east + v_north * v_north) ** 0.5,
            dVPsi=d_vpsi,
        ),
    )


class FormationMathTests(unittest.TestCase):
    def test_clamp_and_track_transforms_round_trip(self) -> None:
        """验证限幅函数和东北天/航迹系坐标变换的基础几何关系。"""

        self.assertEqual(clamp(3.0, -1.0, 2.0), 2.0)

        state = _motion(v_east=10.0, v_north=0.0)
        self.assertEqual(enu_to_track((1.0, 2.0, 3.0), state), (1.0, 3.0, -2.0))
        self.assertEqual(track_to_enu((1.0, 3.0, -2.0), state), (1.0, 2.0, 3.0))

        northbound = _motion(v_east=0.0, v_north=10.0)
        self.assertAlmostEqual(enu_to_track((0.0, 2.0, 3.0), northbound)[0], 2.0)
        self.assertAlmostEqual(enu_to_track((0.0, 2.0, 3.0), northbound)[1], 3.0)
        self.assertAlmostEqual(track_to_enu((2.0, 3.0, 0.0), northbound)[1], 2.0)

    def test_horizontal_track_to_enu_ignores_vertical_velocity(self) -> None:
        """验证水平队形槽位只随水平航迹旋转，不被长机爬升/下降角耦合。"""

        northbound_climb = _motion(v_east=0.0, v_north=10.0, v_up=10.0)

        east, north = horizontal_track_to_enu((-54.0, -58.0), northbound_climb)

        self.assertAlmostEqual(east, -58.0)
        self.assertAlmostEqual(north, -54.0)


class CtrlPidTests(unittest.TestCase):
    def test_pid_p_i_d_limits_and_reset(self) -> None:
        """验证单轴 PID 的 P/I/D 输出、积分限幅、输出限幅和 reset 清零。"""

        pid = Pid()
        pid.init(CtrlInitS(kp=2.0, ki=1.0, kd=0.5, dt=0.1, iMax=0.15, outMax=10.0))

        self.assertAlmostEqual(pid.step(1.0, 2.0), 3.1)
        self.assertAlmostEqual(pid.step(1.0, 0.0), 2.15)
        self.assertAlmostEqual(pid.step(100.0, 0.0), 10.0)

        pid.reset()
        self.assertAlmostEqual(pid.step(0.0, 0.0), 0.0)


class PosCalcTests(unittest.TestCase):
    def test_route_interp_projects_to_line_and_sets_speed(self) -> None:
        """验证长机航线插值把当前位置投影到直线航段并生成沿航段速度指令。"""

        ctx = FormContextS()
        ctx.selfState = _motion(east=3.0, north=4.0, h=5.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0)),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
            vdCmd=7.0,
        )
        u = RouteInterpInputS(selfState=ctx.selfState, wayLine=ctx.wayLine)
        y = PosCalcOutputS(selfCmd=ctx.selfCmd)

        RouteInterp().step(u, y)

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 3.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 0.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 5.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 7.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_route_interp_look_ahead_moves_target_forward_on_line(self) -> None:
        """验证启用 L1 前视距离时，长机目标点沿当前航段前移。"""

        ctx = FormContextS()
        ctx.selfState = _motion(east=3.0, north=4.0, h=5.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0)),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
            vdCmd=7.0,
        )
        route = RouteInterp()
        route.init(RouteInterpInitS(lookAheadDistance=2.0))

        route.step(
            RouteInterpInputS(selfState=ctx.selfState, wayLine=ctx.wayLine),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 5.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 0.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 5.0)

    def test_route_interp_projects_to_reversed_diagonal_segment(self) -> None:
        """验证反向对角航段投影：selfState=(5,0,5) 应投影到航段中点。"""

        ctx = FormContextS()
        ctx.selfState = _motion(east=5.0, north=0.0, h=5.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(5.0, 5.0, 5.0)),
            end=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0)),
            vdCmd=10.0,
        )
        u = RouteInterpInputS(selfState=ctx.selfState, wayLine=ctx.wayLine)
        y = PosCalcOutputS(selfCmd=ctx.selfCmd)

        RouteInterp().step(u, y)

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 2.5)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 2.5)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 5.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, -10.0 / (2.0 ** 0.5))
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, -10.0 / (2.0 ** 0.5))

    def test_route_interp_extends_after_segment_end(self) -> None:
        """验证单航段过终点后沿切向延拓目标点，避免长机长期追身后的终点。"""

        ctx = FormContextS()
        ctx.selfState = _motion(east=15.0, north=3.0, h=5.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0)),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
            vdCmd=7.0,
        )
        u = RouteInterpInputS(selfState=ctx.selfState, wayLine=ctx.wayLine)
        y = PosCalcOutputS(selfCmd=ctx.selfCmd)

        RouteInterp().step(u, y)

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 15.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 0.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 5.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 7.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_route_interp_keeps_ground_speed_on_climbing_segment(self) -> None:
        """验证爬升航段上 vdCmd 按地速分解：水平合速度恰为 vdCmd，天向速度由航迹角带出。"""

        ctx = FormContextS()
        # 起点放在航段起点，专注校验速度分解；航段水平 3-4-5、爬升 30m。
        ctx.selfState = _motion(east=0.0, north=0.0, h=1000.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 1000.0)),
            end=WayPointS(pos=PosInEarthS(30.0, 40.0, 1030.0)),
            vdCmd=50.0,
        )
        u = RouteInterpInputS(selfState=ctx.selfState, wayLine=ctx.wayLine)
        y = PosCalcOutputS(selfCmd=ctx.selfCmd)

        RouteInterp().step(u, y)

        # hlen=50: vEast=50*30/50, vNorth=50*40/50, vUp=50*30/50。
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 30.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 40.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vUp, 30.0)
        # 核心回归点：地速恒等于 vdCmd，不被爬升角的 cosγ 压小。
        self.assertAlmostEqual(ctx.selfCmd.v.vd, 50.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vTheta, math.atan2(30.0, 50.0))
        self.assertAlmostEqual(ctx.selfCmd.v.vPsi, math.atan2(40.0, 30.0))

    def test_route_interp_rejects_pure_vertical_segment(self) -> None:
        """验证纯垂直航段（水平长度为零）被显式拒绝，固定翼地速在该航段无定义。"""

        u = RouteInterpInputS(
            selfState=_motion(h=1000.0),
            wayLine=WayLineS(
                start=WayPointS(pos=PosInEarthS(0.0, 0.0, 1000.0)),
                end=WayPointS(pos=PosInEarthS(0.0, 0.0, 1100.0)),
                vdCmd=50.0,
            ),
        )
        with self.assertRaisesRegex(ValueError, "horizontal"):
            RouteInterp().step(u, PosCalcOutputS(selfCmd=MotionProfS()))

    def test_route_interp_rejects_curve_segment(self) -> None:
        """验证本轮未实现的曲线航段会显式报错，避免静默给出错误目标。"""

        route = RouteInterp()
        u = RouteInterpInputS(
            selfState=_motion(),
            wayLine=WayLineS(radius=10.0),
        )
        with self.assertRaisesRegex(NotImplementedError, "curve"):
            route.step(u, PosCalcOutputS(selfCmd=MotionProfS()))

    def test_slot_geometry_uses_pattern_lookup_and_self_id(self) -> None:
        """验证僚机槽位按 pattern 反查队形行、按 selfId 查槽位，而不是按枚举值或数组位置取。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=12.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A02",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 20.0, -5.0)]],
            )
        )
        ctx.selfState = _motion(east=70.0, north=220.0, h=995.0, v_east=12.0)

        slot.step(
            SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 70.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 220.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 995.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 12.0)

    def test_slot_geometry_adds_along_track_catchup_speed(self) -> None:
        """验证僚机落后于前向槽位时，速度指令会沿长机航迹方向增加以收敛待飞距。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=8.0)
        ctx.selfState = _motion(east=16.0, north=258.0, h=1000.0, v_east=8.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A02",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -54.0, 58.0, 0.0)]],
            )
        )

        slot.step(
            SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 46.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 258.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 10.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_slot_geometry_rotates_slot_offsets_with_leader_track(self) -> None:
        """验证转弯后槽位随长机航迹旋转，A03 不会继续使用固定 ENU 偏移。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=6000.0, north=200.0, h=1000.0, v_east=0.0, v_north=35.0)
        ctx.selfState = _motion(east=6058.0, north=146.0, h=1000.0, v_east=0.0, v_north=35.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A03",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A03", -54.0, -58.0, 0.0)]],
            )
        )

        slot.step(
            SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 6058.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 146.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.h, 1000.0)

    def test_slot_geometry_falls_back_to_enu_offset_when_leader_track_is_undefined(self) -> None:
        """验证长机水平速度为 0 时槽位回退固定 ENU 偏移，不因航迹未定义而崩溃。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0)
        ctx.selfState = _motion(east=70.0, north=220.0, h=1000.0, v_east=5.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A02",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 20.0, 0.0)]],
            )
        )

        slot.step(
            SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 70.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 220.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 0.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_slot_geometry_does_not_add_lateral_position_error_to_velocity(self) -> None:
        """验证槽位单元不把侧向位置误差重复注入速度，侧向收敛交给位置跟踪环。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=8.0)
        ctx.selfState = _motion(east=46.0, north=250.0, h=1000.0, v_east=8.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A02",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -54.0, 58.0, 0.0)]],
            )
        )

        slot.step(
            SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 46.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 258.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 8.0)
        self.assertAlmostEqual(ctx.selfCmd.v.vNorth, 0.0)

    def test_slot_geometry_feeds_forward_turn_speed(self) -> None:
        """验证转弯时槽位速度前馈：外侧槽位沿航迹加速、内侧减速，并补后方槽位的横扫分量。"""

        # 长机向东 vd=30，左转 ω=+0.1 rad/s；僚机恰在槽位上(无待飞距 trim)。
        omega = 0.1
        for self_id, lateral, expect_along in (("A02", 58.0, 30.0 - 58.0 * omega), ("A03", -58.0, 30.0 + 58.0 * omega)):
            ctx = FormContextS()
            ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=30.0, d_vpsi=omega)
            ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
            slot = SlotGeometry()
            slot.init(
                SlotGeometryInitS(
                    selfId=self_id,
                    formPat=[FormPatE.TRIANGLE],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS(self_id, -54.0, lateral, 0.0)]],
                )
            )
            # 槽位偏移按东向航迹旋转：东=前向，左(北)为正侧偏。
            ctx.selfState = _motion(east=46.0, north=200.0 + lateral, h=1000.0, v_east=30.0)

            slot.step(
                SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
                PosCalcOutputS(selfCmd=ctx.selfCmd),
            )

            # 沿航迹速度 = vd - c·ω(外侧加速、内侧减速)；横扫 = a·ω 投到左向(此处为北向分量)。
            self.assertAlmostEqual(ctx.selfCmd.v.vEast, expect_along)
            self.assertAlmostEqual(ctx.selfCmd.v.vNorth, -54.0 * omega)


class PosTrackTests(unittest.TestCase):
    def test_pid_compose_ignores_forward_position_and_uses_speed_pi(self) -> None:
        """验证 PID 组合跟踪中前向只做速度 PI，法向/侧向按苏联系轴序生成加速度。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=2.0, ki=1.0, kd=100.0, dt=0.1),
                gainLateral=CtrlInitS(kp=0.5, ki=0.0, kd=0.0, dt=0.1),
                gainVertical=CtrlInitS(kp=0.25, ki=0.0, kd=0.0, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=1000.0, v_east=10.0)
        ctx.selfCmd = _motion(east=50.0, north=4.0, h=1008.0, v_east=12.0)

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 4.2)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 2.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 2.0)

    def test_pid_compose_forward_speed_pi_uses_scalar_speed_error(self) -> None:
        """验证前向速度环按地速标量误差控制，不把目标速度投影到当前航向后再相减。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=1.0, ki=0.0, kd=0.0, dt=0.1, outMax=20.0),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(v_east=0.0, v_north=10.0)
        ctx.selfCmd = _motion(v_east=20.0, v_north=0.0)

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 0.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 10.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 0.0)

    def test_pid_compose_rejects_low_speed_without_overwriting_output(self) -> None:
        """验证低于 vMin 时航迹系奇异状态会 fail-fast，且不会覆盖已有输出。"""

        tracker = PidCompose()
        tracker.init(PidComposeInitS(vMin=3.0))
        out = AccInEarthS(accEast=1.0, accNorth=2.0, accUp=3.0)

        with self.assertRaisesRegex(ValueError, "vMin"):
            tracker.step(
                PosTrackInputS(selfCmd=_motion(), selfState=_motion(v_east=1.0)),
                PosTrackOutputS(accCmd=out),
            )

        self.assertEqual((out.accEast, out.accNorth, out.accUp), (1.0, 2.0, 3.0))


class ProcessUnitTests(unittest.TestCase):
    def test_hold_writes_hold_triangle(self) -> None:
        """验证本轮 Hold 编排固定输出编队保持和三角队形。"""

        ctx = FormContextS()
        Hold().step(
            FormationTaskInputS(remote=RemoteCmdS(stage=FormStageE.RECONFIG), cmd=ctx.cmd),
            FormationTaskOutputS(cmd=ctx.cmd),
        )

        self.assertEqual(ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(ctx.cmd.pattern, FormPatE.TRIANGLE)

    def test_leader_route_selects_current_segment_from_route(self) -> None:
        """验证长机轨迹规划持有整条航线，每拍只向黑板写当前航段。"""

        ctx = FormContextS()
        planner = LeaderRoute()
        planner.init(
            LeaderRouteInitS(
                RouteS(
                    lines=[
                        WayLineS(
                            idx=0,
                            start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
                            end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            vdCmd=8.0,
                        ),
                        WayLineS(
                            idx=1,
                            start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            end=WayPointS(idx=2, pos=PosInEarthS(200.0, 0.0, 1000.0)),
                            vdCmd=8.0,
                        ),
                    ]
                )
            )
        )
        ctx.selfState = _motion(east=50.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 0)

        ctx.selfState = _motion(east=120.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 1)
        original_end = ctx.wayLine.end.pos.east

        Noop().step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )

        self.assertEqual(ctx.wayLine.end.pos.east, original_end)

    def test_leader_route_switches_by_20deg_turn_radius(self) -> None:
        """验证多航段交接按 20deg 坡度转弯半径乘 1.2 裕度提前切到下一航段。"""

        ctx = FormContextS()
        planner = LeaderRoute()
        planner.init(
            LeaderRouteInitS(
                RouteS(
                    lines=[
                        WayLineS(
                            idx=0,
                            start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
                            end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            vdCmd=10.0,
                        ),
                        WayLineS(
                            idx=1,
                            start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            end=WayPointS(idx=2, pos=PosInEarthS(100.0, 100.0, 1000.0)),
                            vdCmd=10.0,
                        ),
                    ]
                )
            )
        )

        ctx.selfState = _motion(east=65.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 0)

        ctx.selfState = _motion(east=73.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 1)

    def test_leader_route_switch_distance_scales_with_heading_change(self) -> None:
        """验证非 90deg 转弯按 R*tan(delta_psi/2) 提前切段，避免浅转弯过早切角。"""

        ctx = FormContextS()
        planner = LeaderRoute()
        planner.init(
            LeaderRouteInitS(
                RouteS(
                    lines=[
                        WayLineS(
                            idx=0,
                            start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
                            end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            vdCmd=10.0,
                        ),
                        WayLineS(
                            idx=1,
                            start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                            end=WayPointS(idx=2, pos=PosInEarthS(186.6, 50.0, 1000.0)),
                            vdCmd=10.0,
                        ),
                    ]
                )
            )
        )

        ctx.selfState = _motion(east=80.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 0)

        ctx.selfState = _motion(east=93.0, h=1000.0)
        planner.step(
            TraPlanInputS(cmd=ctx.cmd, wayLine=ctx.wayLine, selfState=ctx.selfState),
            TraPlanOutputS(wayLine=ctx.wayLine),
        )
        self.assertEqual(ctx.wayLine.idx, 1)

    def test_leader_broadcast_targets_topology_and_inbound_parses_latest(self) -> None:
        """验证长机广播按拓扑生成多播目标，僚机收消息解析长机状态和编队指令。"""

        ctx = FormContextS()
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE, step=2)
        ctx.selfState = _motion(east=1.0, north=2.0, h=3.0, v_east=4.0, v_north=5.0)
        outbound = LeaderBroadcast()
        outbound.init(
            OutboundInitS(
                selfId="A01",
                netWork=[
                    NetWorkS("A01", "A02", CommDirE.DUPLEX),
                    NetWorkS("A03", "A01", CommDirE.DUPLEX),
                    NetWorkS("A04", "A01", CommDirE.SIMPLEX),
                ],
            )
        )
        out = OutboundOutputS()

        outbound.step(OutboundInputS(cmd=ctx.cmd, selfState=ctx.selfState), out)

        self.assertEqual(len(out.outbox), 1)
        self.assertEqual(out.outbox[0].source, "A01")
        self.assertEqual(out.outbox[0].target, ["A02", "A03"])

        follower_ctx = FormContextS()
        inbound = LeaderFollower()
        inbound.step(InboundInputS(inbox=out.outbox), InboundOutputS(follower_ctx.leaderState, follower_ctx.cmd))

        self.assertEqual(follower_ctx.cmd.stage, FormStageE.HOLD)
        self.assertEqual(follower_ctx.cmd.pattern, FormPatE.TRIANGLE)
        self.assertAlmostEqual(follower_ctx.leaderState.pos.east, 1.0)

        inbound.step(InboundInputS(inbox=[]), InboundOutputS(follower_ctx.leaderState, follower_ctx.cmd))
        self.assertAlmostEqual(follower_ctx.leaderState.pos.east, 1.0)

    def test_inbound_skips_non_leader_follower_messages(self) -> None:
        """验证收消息单元忽略非长机编队广播，避免其它 topic 污染编队黑板。"""

        ctx = FormContextS()
        msg = MessageEnvelope("node.status", "A99", "A02", 0.0, {"health": "normal"})

        LeaderFollower().step(InboundInputS(inbox=[msg]), InboundOutputS(ctx.leaderState, ctx.cmd))

        self.assertEqual(ctx.cmd.stage, FormStageE.NONE)


class EntityTests(unittest.TestCase):
    def test_leader_and_follower_ports_share_context_and_run_one_frame(self) -> None:
        """验证长机/僚机实体完成端口绑定，并能通过一帧 outbox/inbox 串起领航跟随数据流。"""

        comm = FormCommInitS(
            netWork=[NetWorkS("A01", "A02", CommDirE.DUPLEX)],
            formPat=[FormPatE.TRIANGLE],
            formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 20.0, 0.0)]],
        )
        leader = LeaderEntity()
        follower = FollowerEntity()
        leader.init(EntityInitS(selfInit=FormSelfInitS("A01"), commInit=comm))
        follower.init(EntityInitS(selfInit=FormSelfInitS("A02"), commInit=comm))

        leader_state = _motion(east=5.0, north=0.0, h=1000.0, v_east=8.0)
        leader_out = EntityOutputS()
        leader.step(EntityInputS(selfState=leader_state, remote=RemoteCmdS(FormStageE.HOLD)), leader_out)

        self.assertIs(leader._pos_calc_y.selfCmd, leader.cxt.selfCmd)
        self.assertEqual(len(leader_out.outbox), 1)

        follower_out = EntityOutputS()
        follower.step(
            EntityInputS(
                selfState=_motion(east=-30.0, north=15.0, h=1000.0, v_east=8.0),
                inbox=leader_out.outbox,
            ),
            follower_out,
        )

        self.assertIs(follower._inbound_y.leaderState, follower.cxt.leaderState)
        self.assertEqual(follower_out.outbox, [])
        self.assertAlmostEqual(follower.cxt.selfCmd.pos.east, -25.0)
        self.assertAlmostEqual(follower.cxt.selfCmd.pos.north, 20.0)

    def test_entity_reset_clears_context_and_boundary_buffers(self) -> None:
        """验证实体 reset 会原地复位 Context、边界缓存和子单元状态。"""

        comm = FormCommInitS(
            netWork=[NetWorkS("A01", "A02", CommDirE.DUPLEX)],
            formPat=[FormPatE.TRIANGLE],
            formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 20.0, 0.0)]],
        )
        leader = LeaderEntity()
        follower = FollowerEntity()
        leader.init(EntityInitS(selfInit=FormSelfInitS("A01"), commInit=comm))
        follower.init(EntityInitS(selfInit=FormSelfInitS("A02"), commInit=comm))

        leader.step(
            EntityInputS(
                selfState=_motion(east=5.0, h=1000.0, v_east=8.0),
                remote=RemoteCmdS(FormStageE.HOLD),
            ),
            EntityOutputS(),
        )
        follower.step(
            EntityInputS(
                selfState=_motion(east=-30.0, h=1000.0, v_east=8.0),
                inbox=list(leader._outbox),
            ),
            EntityOutputS(),
        )

        leader.reset()
        follower.reset()

        self.assertEqual(leader.cxt.cmd.stage, FormStageE.NONE)
        self.assertEqual(leader.cxt.selfState.pos.h, 0.0)
        self.assertEqual(leader._outbox, [])
        self.assertIs(leader._task_u.remote, leader._remote)
        self.assertEqual(follower.cxt.cmd.stage, FormStageE.NONE)
        self.assertEqual(follower.cxt.leaderState.pos.h, 0.0)
        self.assertEqual(follower._inbox, [])
        self.assertIs(follower._inbound_y.leaderState, follower.cxt.leaderState)


if __name__ == "__main__":
    unittest.main()
