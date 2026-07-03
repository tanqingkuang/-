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
    PosTrackDiagS,
    RemoteCmdS,
    VdInEarthS,
    WayLineS,
    WayPointS,
)
from src.algorithm.entity.leader_follower_hold.follower import FollowerEntity
from src.algorithm.entity.leader_follower_hold.leader import LeaderEntity, _follower_tracker_init
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.ctrl.pid import Pid
from src.algorithm.units.algo.ctrl.ppi import PPI, PPIInitS
from src.algorithm.units.algo.formation_math import clamp, enu_to_track, horizontal_track_to_enu, track_to_enu
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInitS, RouteInterpInputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.lateral_track_angle import LateralTrackAngle, LateralTrackAngleInitS
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
from src.environment.model import AircraftState, ModelIterator, PointMass3DoFModel


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

        # step(posErr, velFf, velActual)，并联式内部取 velErr=velFf-velActual；此处 velActual=0 故 velErr=velFf。
        self.assertAlmostEqual(pid.step(1.0, 2.0, 0.0), 3.1)
        self.assertAlmostEqual(pid.step(1.0, 0.0, 0.0), 2.15)
        self.assertAlmostEqual(pid.step(100.0, 0.0, 0.0), 10.0)

        pid.reset()
        self.assertAlmostEqual(pid.step(0.0, 0.0, 0.0), 0.0)

    def test_pid_velocity_integral_accumulates_and_resets(self) -> None:
        """验证速度积分通道(kiv)按速度误差累积、与位置通道独立，reset 清零速度积分。"""

        # kp=ki=0(位置通道关闭)，输出 = kd·velErr + kiv·∫velErr。
        pid = Pid()
        pid.init(CtrlInitS(kp=0.0, ki=0.0, kd=0.5, kiv=2.0, dt=0.1))

        # 第一拍：∫velErr = 1.0·0.1 = 0.1，输出 = 0.5·1 + 2.0·0.1 = 0.7。
        self.assertAlmostEqual(pid.step(0.0, 1.0, 0.0), 0.7)
        # 第二拍：∫velErr 累积到 0.2，输出 = 0.5·1 + 2.0·0.2 = 0.9。位置误差非零也不应影响(ki=0)。
        self.assertAlmostEqual(pid.step(100.0, 1.0, 0.0), 0.9)

        pid.reset()
        # reset 后速度积分清零，本拍重新从 0 累积，输出同首拍 0.7(未清零则为 0.5+2.0·0.3=1.1)。
        self.assertAlmostEqual(pid.step(0.0, 1.0, 0.0), 0.7)

    def test_pid_velocity_integral_clamped_by_imaxvel(self) -> None:
        """验证速度积分受 iMaxVel 限幅，正负两侧都被钳住。"""

        pid = Pid()
        pid.init(CtrlInitS(kp=0.0, ki=0.0, kd=0.0, kiv=1.0, dt=0.1, iMaxVel=0.15))

        # ∫velErr 累积 0.2 但被钳到 0.15，输出 = 1.0·0.15。
        self.assertAlmostEqual(pid.step(0.0, 2.0, 0.0), 0.15)
        # 大负误差把速度积分拉到下限 -0.15。
        self.assertAlmostEqual(pid.step(0.0, -100.0, 0.0), -0.15)

    def test_pid_rejects_dual_integrators(self) -> None:
        """验证位置积分与速度积分互斥：ki 与 kiv 同时非零时 init 报错。"""

        pid = Pid()
        with self.assertRaises(ValueError):
            pid.init(CtrlInitS(ki=1.0, kiv=1.0, dt=0.1))


class CtrlPpiTests(unittest.TestCase):
    def test_ppi_cascade_equivals_parallel_pid_without_limits(self) -> None:
        """验证无限幅时串级 P+PI(纯比例) 与并联式 PID 代数等价：kpPos·kpVel=kp、kpVel=kd。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=0.5, kpVel=2.0, kiVel=0.0, dt=0.1))
        # vel_cmd = 3.0 + 0.5·1.0 = 3.5; vel_err = 3.5 - 0 = 3.5; acc = 2.0·3.5 = 7.0。
        # 等价并联式 kp=1.0,kd=2.0: 1.0·1.0 + 2.0·3.0 = 7.0。
        self.assertAlmostEqual(ppi.step(1.0, 3.0, 0.0), 7.0)

    def test_ppi_velocity_command_clamp_is_asymmetric(self) -> None:
        """验证外环速度指令按 vCmdMin/vCmdMax 非对称限幅，限的是速度而非加速度。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=1.0, kpVel=1.0, kiVel=0.0, dt=0.1, vCmdMin=0.0, vCmdMax=5.0))
        # vel_cmd = 2.0 + 1.0·10 = 12.0 -> 夹到 5.0; vel_err = 5.0 - 1.0 = 4.0; acc = 4.0。
        self.assertAlmostEqual(ppi.step(10.0, 2.0, 1.0), 4.0)
        # vel_cmd = 2.0 - 10 = -8.0 -> 夹到下限 0.0; vel_err = 0 - 1.0 = -1.0; acc = -1.0。
        self.assertAlmostEqual(ppi.step(-10.0, 2.0, 1.0), -1.0)

    def test_ppi_acceleration_clamp_is_asymmetric(self) -> None:
        """验证内环输出加速度按 accMin/accMax 非对称限幅。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=0.0, kpVel=10.0, kiVel=0.0, dt=0.1, accMin=-1.0, accMax=2.0))
        self.assertAlmostEqual(ppi.step(0.0, 5.0, 0.0), 2.0)  # acc=50 -> 上限 2.0
        self.assertAlmostEqual(ppi.step(0.0, -5.0, 0.0), -1.0)  # acc=-50 -> 下限 -1.0

    def test_ppi_integral_clamped_by_iout_max(self) -> None:
        """验证内环积分(以加速度贡献量存储)受 iOutMax 限幅，正负两侧都被钳住。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=0.0, kpVel=0.0, kiVel=1.0, dt=0.1, iOutMax=0.15))
        # 积分累积 1.0·2.0·0.1=0.2 但被钳到 0.15，acc = 0 + 0.15。
        self.assertAlmostEqual(ppi.step(0.0, 2.0, 0.0), 0.15)
        # 大负误差把积分拉到下限 -0.15。
        self.assertAlmostEqual(ppi.step(0.0, -100.0, 0.0), -0.15)

    def test_ppi_anti_windup_back_calculation_recovers_immediately(self) -> None:
        """验证输出饱和时反算法回退积分：饱和期积分被钉住不绕死，误差反向后立即退出饱和。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=0.0, kpVel=0.0, kiVel=1.0, dt=1.0, accMin=-1.0, accMax=1.0))
        # 第一拍：积分 +2.0=2.0，acc=2.0 夹到 1.0，反算法回退 (2.0-1.0) -> 积分钉在 1.0。
        self.assertAlmostEqual(ppi.step(0.0, 2.0, 0.0), 1.0)
        # 第二拍：再饱和，积分仍被钉在 1.0(无 windup)。
        self.assertAlmostEqual(ppi.step(0.0, 2.0, 0.0), 1.0)
        # 误差反向：积分 1.0 + (-0.5) = 0.5，acc=0.5 不再饱和，立即恢复。
        # 若无抗饱和，积分会绕到 4.0 以上，此拍仍被钉在上限 1.0。
        self.assertAlmostEqual(ppi.step(0.0, -0.5, 0.0), 0.5)

    def test_ppi_reset_clears_integral(self) -> None:
        """验证 reset 清零内环积分。"""

        ppi = PPI()
        ppi.init(PPIInitS(kpPos=0.0, kpVel=0.0, kiVel=2.0, dt=0.1))
        ppi.step(0.0, 1.0, 0.0)  # 积分累积 0.2
        ppi.reset()
        self.assertAlmostEqual(ppi.step(0.0, 1.0, 0.0), 0.2)  # 复位后从 0 重新累积

    def test_ppi_init_rejects_inverted_limits(self) -> None:
        """验证 init 拦截非法限幅区间(下限>上限)。"""

        with self.assertRaises(ValueError):
            PPI().init(PPIInitS(vCmdMin=5.0, vCmdMax=1.0, dt=0.1))
        with self.assertRaises(ValueError):
            PPI().init(PPIInitS(accMin=2.0, accMax=-2.0, dt=0.1))


class PosCalcTests(unittest.TestCase):
    def test_route_interp_projects_to_line_and_sets_speed(self) -> None:
        """验证长机航线插值把当前位置投影到直线航段并生成沿航段速度指令。"""

        ctx = FormContextS()
        ctx.selfState = _motion(east=3.0, north=4.0, h=5.0)
        ctx.wayLine = WayLineS(
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0), vdCmd=7.0),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
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
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0), vdCmd=7.0),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
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
            start=WayPointS(pos=PosInEarthS(5.0, 5.0, 5.0), vdCmd=10.0),
            end=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0)),
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
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 5.0), vdCmd=7.0),
            end=WayPointS(pos=PosInEarthS(10.0, 0.0, 5.0)),
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
            start=WayPointS(pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=50.0),
            end=WayPointS(pos=PosInEarthS(30.0, 40.0, 1030.0)),
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
                start=WayPointS(pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=50.0),
                end=WayPointS(pos=PosInEarthS(0.0, 0.0, 1100.0)),
            ),
        )
        with self.assertRaisesRegex(ValueError, "horizontal"):
            RouteInterp().step(u, PosCalcOutputS(selfCmd=MotionProfS()))

    def test_route_interp_tracks_arc_segment_with_curvature_ff(self) -> None:
        """验证圆弧航段：目标点投影到弧上、速度沿切向、曲率前馈 dVPsi=vd·κ(右转为负)。"""

        # 东->南右转圆弧，R=400：切入(1600,0)、切出(2000,-400)、圆心(1600,-400)、turnSign=-1。
        line = WayLineS(
            start=WayPointS(
                pos=PosInEarthS(1600.0, 0.0, 1000.0),
                vdCmd=20.0,
                turnSign=-1.0,
                center=PosInEarthS(1600.0, -400.0, 1000.0),
            ),
            end=WayPointS(pos=PosInEarthS(2000.0, -400.0, 1000.0)),
        )
        # 飞机恰在弧中点(进度 0.5)，航向东南(-45°)。
        mid_e = 1600.0 + 400.0 * math.cos(math.pi / 4.0)
        mid_n = -400.0 + 400.0 * math.sin(math.pi / 4.0)
        self_state = _motion(
            east=mid_e, north=mid_n, h=1000.0,
            v_east=20.0 * math.cos(-math.pi / 4.0), v_north=20.0 * math.sin(-math.pi / 4.0),
        )
        route = RouteInterp()
        route.init(RouteInterpInitS(leadTimeS=0.5))  # σ=0.5s
        cmd = MotionProfS()

        route.step(RouteInterpInputS(selfState=self_state, wayLine=line), PosCalcOutputS(selfCmd=cmd))

        # 目标点在弧上(到圆心距离=R)，且为该在弧点的投影(=自身)。
        self.assertAlmostEqual(math.hypot(cmd.pos.east - 1600.0, cmd.pos.north + 400.0), 400.0, places=3)
        self.assertAlmostEqual(cmd.pos.east, mid_e, places=3)
        self.assertAlmostEqual(cmd.pos.north, mid_n, places=3)
        # 速度沿切向、地速=vdCmd、航向 -45°。
        self.assertAlmostEqual(cmd.v.vd, 20.0, places=6)
        self.assertAlmostEqual(cmd.v.vPsi, -math.pi / 4.0, places=4)
        # 曲率前馈 dVPsi = vd·κ = 20·(-1/400) = -0.05 rad/s。
        self.assertAlmostEqual(cmd.v.dVPsi, 20.0 * (-1.0 / 400.0), places=4)

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
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, -5.0, -20.0)]],
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

    def test_slot_geometry_leaves_forward_catchup_to_position_track(self) -> None:
        """验证僚机落后于前向槽位时，槽位单元只给纯速度前馈、不再注入待飞距追赶速度(已下沉到 PidCompose 前向位置环)。"""

        ctx = FormContextS()
        ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=8.0)
        ctx.selfState = _motion(east=16.0, north=258.0, h=1000.0, v_east=8.0)
        ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
        slot = SlotGeometry()
        slot.init(
            SlotGeometryInitS(
                selfId="A02",
                formPat=[FormPatE.TRIANGLE],
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -54.0, 0.0, -58.0)]],
            )
        )

        slot.step(
            SlotGeometryInputS(leaderState=ctx.leaderState, cmd=ctx.cmd),
            PosCalcOutputS(selfCmd=ctx.selfCmd),
        )

        self.assertAlmostEqual(ctx.selfCmd.pos.east, 46.0)
        self.assertAlmostEqual(ctx.selfCmd.pos.north, 258.0)
        # 落后 30m 的待飞距不再叠加到速度上，速度只剩长机直线前馈。
        self.assertAlmostEqual(ctx.selfCmd.v.vEast, 8.0)
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
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A03", -54.0, 0.0, 58.0)]],
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
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 0.0, -20.0)]],
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
                formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -54.0, 0.0, -58.0)]],
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
        """验证转弯时槽位速度前馈：沿航迹分量按 b·ω 增减(外/内侧由 b 与 ω 符号共定)，并补后方槽位横扫。"""

        # 长机向东 vd=30，左转 ω=+0.1 rad/s；僚机恰在槽位上(无待飞距 trim)。
        omega = 0.1
        for self_id, right_offset, expect_along in (("A02", -58.0, 30.0 - 58.0 * omega), ("A03", 58.0, 30.0 + 58.0 * omega)):
            ctx = FormContextS()
            ctx.leaderState = _motion(east=100.0, north=200.0, h=1000.0, v_east=30.0, d_vpsi=omega)
            ctx.cmd = FormSnapshotS(stage=FormStageE.HOLD, pattern=FormPatE.TRIANGLE)
            slot = SlotGeometry()
            slot.init(
                SlotGeometryInitS(
                    selfId=self_id,
                    formPat=[FormPatE.TRIANGLE],
                    formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS(self_id, -54.0, 0.0, right_offset)]],
                )
            )
            # 槽位偏移按东向航迹旋转：东=前向，南=右侧向。
            ctx.selfState = _motion(east=46.0, north=200.0 - right_offset, h=1000.0, v_east=30.0)

            slot.step(
                SlotGeometryInputS(selfState=ctx.selfState, leaderState=ctx.leaderState, cmd=ctx.cmd),
                PosCalcOutputS(selfCmd=ctx.selfCmd),
            )

            # 沿航迹速度 = vd + b·ω；本例左转 ω>0，故 b<0(左/内侧)减速、b>0(右/外侧)加速。横扫 = a·ω 投到左向(此处为北向分量)。
            self.assertAlmostEqual(ctx.selfCmd.v.vEast, expect_along)
            self.assertAlmostEqual(ctx.selfCmd.v.vNorth, -54.0 * omega)


class PosTrackTests(unittest.TestCase):
    def test_pid_compose_forward_closes_position_and_velocity(self) -> None:
        """验证僚机型前向位置环：前向加速度同时受待飞距(kp)与速度误差(kd)驱动，法向/侧向按苏联系轴序生成加速度。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=0.1, ki=0.0, kd=0.5, dt=0.1),
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

        # 前向(东向)：kp·前向位置误差 + kd·前向速度误差 = 0.1·50 + 0.5·2 = 6.0。
        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 6.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 2.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 2.0)

    def test_pid_compose_forward_speed_p_uses_scalar_speed_error(self) -> None:
        """验证长机型前向速度环按地速标量误差控制(纯 P，速度比例走 kd、kp=ki=kiv=0)。

        误差幅值仍是地速标量差 vd_cmd-vd_self=20-10=10(与航向无关)；1.1 后误差在
        "目标速度系"分解/还原，故这 10 的前向加速度沿**目标航向**(此处东向)输出，而非本机航向(北向)。
        """

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=0.0, ki=0.0, kd=1.0, dt=0.1, outMax=20.0),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(v_east=0.0, v_north=10.0)
        ctx.selfCmd = _motion(v_east=20.0, v_north=0.0)

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        # 沿目标航向(东)输出标量速度误差 10；本机航向(北)分量为 0。
        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 10.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 0.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 0.0)

    def test_pid_compose_decomposes_error_in_target_velocity_frame(self) -> None:
        """1.1 核心：误差按目标速度系分解。目标在本机航向(北)正右后方 100m，但沿目标航向(东)是纯前向偏置，
        应主要走前向通道(kp_fwd·100=10)，而非被本机航迹系判成大侧偏去转弯(那样会是 kp_lat·100=50)。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=0.1, ki=0.0, kd=0.0, dt=0.1),
                gainLateral=CtrlInitS(kp=0.5, ki=0.0, kd=0.0, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=0.0, v_north=10.0)  # 机头朝北
        ctx.selfCmd = _motion(east=100.0, north=0.0, h=0.0, v_east=10.0)   # 目标航向朝东，在正东 100m

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        # 目标系(东)下 100m 为纯前向误差 → 前向加速度 0.1·100=10 沿东；侧偏为 0，无转弯指令。
        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 10.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 0.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 0.0)

    def test_pid_compose_falls_back_to_self_frame_when_target_hovers(self) -> None:
        """目标水平速度低于 vMin(悬停/集结起步)时航向无定义，应退回本机航迹系兜底，不因建基奇异而抛错。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=0.5,
                gainLateral=CtrlInitS(kp=0.5, ki=0.0, kd=0.0, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=0.0, v_east=10.0)  # 机头朝东，可飞
        ctx.selfCmd = _motion(east=0.0, north=50.0, h=0.0)               # 目标在正北 50m，且悬停(零速)

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        # 退回自身系(东)：北向 50m 为侧偏，lateral_right=(0,-1,0) → 侧偏 -50 → 侧向加速度 -25 落在北向 +25。
        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 0.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 25.0)
        self.assertAlmostEqual(ctx.selfAccCmd.accUp, 0.0)

    def test_pid_compose_centripetal_ff_uses_self_speed_not_target_speed(self) -> None:
        """向心前馈须按本机自身地速换算(a_lat=dVPsi·V_self)，而非目标速度。

        本机东向 10、目标东向 20(dVPsi=0.1)、位置零误差、各轴增益关：侧向仅剩前馈
        lateral_ff=-dVPsi·V_self=-1.0，目标系(东)侧向轴=(0,-1,0) → accNorth=+1.0(取本机速度 10)。
        若误用目标速度 20 则会得到 2.0。
        """
        tracker = PidCompose()
        tracker.init(PidComposeInitS(vMin=3.0))  # 三轴增益缺省为零，隔离出纯前馈
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=0.0, v_east=10.0)
        ctx.selfCmd = _motion(east=0.0, north=0.0, h=0.0, v_east=20.0, d_vpsi=0.1)

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd),
        )

        self.assertAlmostEqual(ctx.selfAccCmd.accNorth, 1.0)  # dVPsi·V_self = 0.1·10
        self.assertAlmostEqual(ctx.selfAccCmd.accEast, 0.0)

    def test_pid_compose_writes_control_diagnostics(self) -> None:
        """验证 PosTrack 通过 diag 输出目标指令和控制误差，不把诊断量写入 Context。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=0.1, ki=0.0, kd=0.5, dt=0.1),
                gainLateral=CtrlInitS(kp=0.5, ki=0.0, kd=0.25, dt=0.1),
                gainVertical=CtrlInitS(kp=0.25, ki=0.0, kd=0.75, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=1000.0, v_east=10.0)
        ctx.selfCmd = _motion(east=50.0, north=4.0, h=1008.0, v_east=12.0)
        diag = PosTrackDiagS()

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd, diag=diag),
        )

        self.assertAlmostEqual(diag.cmd_pos_east_m, 50.0)
        self.assertAlmostEqual(diag.cmd_pos_north_m, 4.0)
        self.assertAlmostEqual(diag.cmd_pos_h_m, 1008.0)
        self.assertAlmostEqual(diag.cmd_vel_east_mps, 12.0)
        self.assertAlmostEqual(diag.pos_err_east_m, 50.0)
        self.assertAlmostEqual(diag.pos_err_north_m, 4.0)
        self.assertAlmostEqual(diag.pos_err_h_m, 8.0)
        self.assertAlmostEqual(diag.vel_err_east_mps, 2.0)
        self.assertAlmostEqual(diag.track_pos_err_x_m, 50.0)
        self.assertAlmostEqual(diag.track_pos_err_y_m, 8.0)
        self.assertAlmostEqual(diag.track_pos_err_z_m, -4.0)
        self.assertAlmostEqual(diag.track_vel_err_x_mps, 2.0)

    def test_pid_compose_masks_unused_control_error_diagnostics(self) -> None:
        """未被控制参数使用的航迹系误差诊断应输出 0，避免把无效通道纳入效果分析。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=PPIInitS(kpPos=0.0, kpVel=1.0, kiVel=0.0, dt=0.1),
                gainLateral=CtrlInitS(kp=0.5, ki=0.0, kd=0.25, dt=0.1),
                gainVertical=CtrlInitS(kp=0.25, ki=0.0, kd=0.75, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, north=0.0, h=1000.0, v_east=10.0)
        ctx.selfCmd = _motion(east=50.0, north=4.0, h=1008.0, v_east=12.0, v_north=-3.0, v_up=5.0)
        diag = PosTrackDiagS()

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd, diag=diag),
        )

        # 1.1 后误差在"目标速度系"(selfCmd 航迹系，含其 v_north/-3、v_up/5 造成的航向与倾角)分解；
        # 数值随之从旧的自身系(东向平飞)结果改变。前向位置(PPI kpPos=0)仍被屏蔽为 0。
        self.assertAlmostEqual(diag.track_pos_err_x_m, 0.0)
        self.assertAlmostEqual(diag.track_pos_err_y_m, -10.39828, places=5)
        self.assertAlmostEqual(diag.track_pos_err_z_m, -16.00735, places=5)
        # 前向速度误差是地速标量差(与系无关)：hypot(12,-3)-10。
        self.assertAlmostEqual(diag.track_vel_err_x_mps, math.hypot(12.0, -3.0) - 10.0)
        self.assertAlmostEqual(diag.track_vel_err_y_mps, 3.63576, places=5)
        self.assertAlmostEqual(diag.track_vel_err_z_mps, 2.42536, places=5)

    def test_pid_compose_masks_unused_velocity_error_diagnostics(self) -> None:
        """未配置速度环增益时，航迹系速度误差诊断应输出 0。"""

        tracker = PidCompose()
        tracker.init(
            PidComposeInitS(
                vMin=3.0,
                gainForward=CtrlInitS(kp=0.1, ki=0.0, kd=0.0, kiv=0.0, dt=0.1),
            )
        )
        ctx = FormContextS()
        ctx.selfState = _motion(east=0.0, h=1000.0, v_east=10.0)
        ctx.selfCmd = _motion(east=50.0, h=1000.0, v_east=12.0)
        diag = PosTrackDiagS()

        tracker.step(
            PosTrackInputS(selfCmd=ctx.selfCmd, selfState=ctx.selfState),
            PosTrackOutputS(accCmd=ctx.selfAccCmd, diag=diag),
        )

        self.assertAlmostEqual(diag.track_pos_err_x_m, 50.0)
        self.assertAlmostEqual(diag.track_vel_err_x_mps, 0.0)

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


class LateralTrackAngleTests(unittest.TestCase):
    """横侧向串级 + 航迹角变限幅控制律。"""

    def _cfg(self, **kw: float) -> LateralTrackAngleInitS:
        base = dict(
            kp=0.02, kd=0.12, ki=0.0, dt=0.05, rollMaxRad=math.radians(40.0),
            gammaMaxRad=math.radians(30.0), floorRad=math.radians(7.0), margin=1.2,
        )
        base.update(kw)
        return LateralTrackAngleInitS(**base)

    def test_unsaturated_matches_parallel_pid(self) -> None:
        """无饱和 + ki=0 时，串级输出严格等于旧并联式 kp·dZ + kd·velErr(平滑迁移的等价保证)。"""
        ctrl = LateralTrackAngle()
        ctrl.init(self._cfg())
        dz, vel_err, v = 1.0, 0.5, 20.0  # 小侧偏，不触发变限幅饱和
        got = ctrl.step(dz, vel_err, v)
        self.assertAlmostEqual(got, 0.02 * dz + 0.12 * vel_err)
        self.assertAlmostEqual(got, 0.08)

    def test_variable_track_angle_limit(self) -> None:
        """限幅曲线：|dZ|≥R→90°；中段→asin(|dZ|/R)；极小→地板 7°。"""
        ctrl = LateralTrackAngle()
        ctrl.init(self._cfg())
        v = 20.0
        radius = v * v / (9.80665 * math.sin(math.radians(30.0))) * 1.2
        self.assertAlmostEqual(ctrl.track_angle_limit_rad(10.0 * radius, v), math.pi / 2)
        self.assertAlmostEqual(ctrl.track_angle_limit_rad(0.5 * radius, v), math.asin(0.5))
        self.assertAlmostEqual(ctrl.track_angle_limit_rad(0.0, v), math.radians(7.0))

    def test_large_cross_track_bounds_lateral_accel(self) -> None:
        """大侧偏下侧向加速度饱和到 kd·V·sin(90°)，且不随侧偏继续增大——这是防"持续滚转→转圈"的本质。"""
        ctrl = LateralTrackAngle()
        ctrl.init(self._cfg())
        v = 20.0
        a1 = ctrl.step(1_000.0, 0.0, v)
        a2 = ctrl.step(1_000_000.0, 0.0, v)
        self.assertAlmostEqual(a1, 0.12 * v * math.sin(math.pi / 2))  # = 2.4，对应 90° 垂直切入
        self.assertAlmostEqual(a1, a2)  # 侧偏放大 1000 倍指令不变：有界拦截，不会越滚越紧
        self.assertGreater(a1, 0.0)     # dZ>0(目标在右) → 向右(正)修正

    def test_roll_limit_clamps_lateral_accel(self) -> None:
        """执行层限幅用滚转角：侧向加速度被夹到 g·tan(rollMax)(而非旧的固定加速度 4.0)。"""
        ctrl = LateralTrackAngle()
        ctrl.init(self._cfg(rollMaxRad=math.radians(40.0)))
        # 大 velErr 使内环需求 acc=kd·(velErr-velErr_cmd) 远超上限，触发夹幅。
        out = ctrl.step(1_000.0, 100.0, 20.0)
        self.assertAlmostEqual(out, 9.80665 * math.tan(math.radians(40.0)))  # ≈8.23 m/s²

    def test_rejects_zero_kd(self) -> None:
        """kd=0 时串级 K1=-kp/kd 与内环比例都退化，应在 init 拦截。"""
        with self.assertRaisesRegex(ValueError, "kd"):
            LateralTrackAngle().init(self._cfg(kd=0.0))


class PosTrackClosedLoopTests(unittest.TestCase):
    """PidCompose(僚机增益) + 三自由度质点模型闭环回归：复现"目标在机头后方偏右→转圈"原始故障。"""

    def test_follower_behind_right_slot_tracks_without_circling(self) -> None:
        """槽位随编队东向 20m/s 移动，僚机初始在槽位**前方 40m、右侧 80m**(即槽位落在其机头右后方)。

        旧实现(本机航迹系度量误差)：右后方目标→右滚消侧偏，但目标在后、侧偏反增，飞机转整圈追踪。
        新实现(1.1 目标速度系 + 1.2 航迹角变限幅)：应靠降速让槽位追上、有界右滚(不越 90°)平滑切入并收敛，
        航向相对编队航向的最大偏离远小于半圈。此用例即该故障的闭环回归护栏。
        """
        model = PointMass3DoFModel(ModelIterator._default_config())
        tracker = PidCompose()
        tracker.init(_follower_tracker_init(0.05))
        state = AircraftState(
            node_id="F", x_m=40.0, y_m=80.0, altitude_m=1000.0, speed_mps=20.0,
            theta_rad=0.0, psi_rad=0.0, ax_mps2=0.0, ay_mps2=0.0, az_mps2=0.0,
            ax_rate_mps3=0.0, ay_rate_mps3=0.0, az_rate_mps3=0.0,
            nx=0.0, nz=0.0, phi_rad=0.0, psi_dot_deg_s=0.0,
        )
        slot_e, slot_n, slot_h, slot_v = 0.0, 0.0, 1000.0, 20.0  # 槽位起于原点、沿东向匀速
        dt = 0.05
        self_cmd, self_state, acc = MotionProfS(), MotionProfS(), AccInEarthS()
        d0 = math.hypot(state.x_m - slot_e, state.y_m - slot_n)
        max_heading_dev = 0.0

        for _ in range(int(40.0 / dt)):
            self_state.pos = PosInEarthS(state.x_m, state.y_m, state.altitude_m)
            self_state.v = VdInEarthS(
                vEast=state.vx_mps, vNorth=state.vy_mps, vUp=state.vz_mps,
                vd=math.hypot(state.vx_mps, state.vy_mps), vPsi=state.psi_rad,
            )
            self_cmd.pos = PosInEarthS(slot_e, slot_n, slot_h)
            self_cmd.v = VdInEarthS(vEast=slot_v, vNorth=0.0, vUp=0.0, vd=slot_v, vPsi=0.0, dVPsi=0.0)
            tracker.step(
                PosTrackInputS(selfCmd=self_cmd, selfState=self_state),
                PosTrackOutputS(accCmd=acc),
            )
            state.update_from_vector(
                model.step(state.as_vector(), (acc.accEast, acc.accNorth, acc.accUp), (0.0, 0.0, 0.0), dt)
            )
            slot_e += slot_v * dt  # 槽位随编队前移
            max_heading_dev = max(max_heading_dev, abs(math.atan2(math.sin(state.psi_rad), math.cos(state.psi_rad))))

        final = math.hypot(state.x_m - slot_e, state.y_m - slot_n)
        # 不转圈：航向相对编队航向(东)的最大偏离远小于半圈(转一圈会扫过 ≥180°)。
        self.assertLess(max_heading_dev, math.radians(90.0))
        # 收敛到槽位：既绝对收敛，也相对初始 89m 大幅收敛。
        self.assertLess(final, 5.0)
        self.assertLess(final, 0.1 * d0)

    def test_follower_lateral_offset_converges_without_sustained_overshoot(self) -> None:
        """僚机纯横偏(150m)切入随编队东向匀速的槽位：应收敛且不大幅过冲到对侧、末段不再持续摆动。

        换到目标速度系度量误差后，丢了自身航迹系随机头旋转的纯追踪前置阻尼；若侧向阻尼(kd)照搬旧自身系
        并联式的偏小值就会欠阻尼——过冲到对侧、持续摆动(base.json 僚机切入曾复现)。kd 整定到 0.30 后收敛。
        本用例用真实僚机增益 `_follower_tracker_init`，是该整定的回归护栏(若把 kd 调回欠阻尼即失败)。
        """
        model = PointMass3DoFModel(ModelIterator._default_config())
        tracker = PidCompose()
        tracker.init(_follower_tracker_init(0.05))
        state = AircraftState(
            node_id="F", x_m=0.0, y_m=150.0, altitude_m=1000.0, speed_mps=20.0,
            theta_rad=0.0, psi_rad=0.0, ax_mps2=0.0, ay_mps2=0.0, az_mps2=0.0,
            ax_rate_mps3=0.0, ay_rate_mps3=0.0, az_rate_mps3=0.0,
            nx=0.0, nz=0.0, phi_rad=0.0, psi_dot_deg_s=0.0,
        )
        slot_e, slot_n, slot_h, slot_v = 0.0, 0.0, 1000.0, 20.0  # 槽位在东向直线上匀速
        dt = 0.05
        self_cmd, self_state, acc = MotionProfS(), MotionProfS(), AccInEarthS()
        crosses: list[float] = []

        for i in range(int(60.0 / dt)):
            self_state.pos = PosInEarthS(state.x_m, state.y_m, state.altitude_m)
            self_state.v = VdInEarthS(
                vEast=state.vx_mps, vNorth=state.vy_mps, vUp=state.vz_mps,
                vd=math.hypot(state.vx_mps, state.vy_mps), vPsi=state.psi_rad,
            )
            self_cmd.pos = PosInEarthS(slot_e, slot_n, slot_h)
            self_cmd.v = VdInEarthS(vEast=slot_v, vNorth=0.0, vUp=0.0, vd=slot_v, vPsi=0.0, dVPsi=0.0)
            tracker.step(
                PosTrackInputS(selfCmd=self_cmd, selfState=self_state),
                PosTrackOutputS(accCmd=acc),
            )
            state.update_from_vector(
                model.step(state.as_vector(), (acc.accEast, acc.accNorth, acc.accUp), (0.0, 0.0, 0.0), dt)
            )
            slot_e += slot_v * dt
            if i % 20 == 0:  # 每 1s 记一次横偏(北向偏移=相对东向槽位线的横偏)
                crosses.append(state.y_m - slot_n)

        overshoot = max(0.0, -min(crosses))  # 从北侧(+150)切入，冲到南侧(负)的最大幅值
        tail = crosses[-10:]
        self.assertLess(overshoot, 5.0)               # 不大幅过冲到对侧(欠阻尼时会到 -20~-34)
        self.assertLess(abs(crosses[-1]), 3.0)        # 收敛到槽位
        self.assertLess(max(tail) - min(tail), 3.0)   # 末段不再持续摆动


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
                [
                    WayLineS(
                        idx=0,
                        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=8.0),
                        end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                    ),
                    WayLineS(
                        idx=1,
                        start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=8.0),
                        end=WayPointS(idx=2, pos=PosInEarthS(200.0, 0.0, 1000.0)),
                    ),
                ]
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
                [
                    WayLineS(
                        idx=0,
                        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=10.0),
                        end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                    ),
                    WayLineS(
                        idx=1,
                        start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=10.0),
                        end=WayPointS(idx=2, pos=PosInEarthS(100.0, 100.0, 1000.0)),
                    ),
                ]
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
                [
                    WayLineS(
                        idx=0,
                        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=10.0),
                        end=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0)),
                    ),
                    WayLineS(
                        idx=1,
                        start=WayPointS(idx=1, pos=PosInEarthS(100.0, 0.0, 1000.0), vdCmd=10.0),
                        end=WayPointS(idx=2, pos=PosInEarthS(186.6, 50.0, 1000.0)),
                    ),
                ]
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
            formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 0.0, -20.0)]],
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
        self.assertIsNotNone(leader_out.selfCmd)
        self.assertIsNotNone(leader_out.controlDiag)
        assert leader_out.selfCmd is not None
        assert leader_out.controlDiag is not None
        self.assertAlmostEqual(leader_out.selfCmd.pos.h, leader.cxt.selfCmd.pos.h)
        self.assertAlmostEqual(leader_out.controlDiag.cmd_pos_h_m, leader.cxt.selfCmd.pos.h)

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
        self.assertIsNotNone(follower_out.selfCmd)
        self.assertIsNotNone(follower_out.controlDiag)
        self.assertAlmostEqual(follower.cxt.selfCmd.pos.east, -25.0)
        self.assertAlmostEqual(follower.cxt.selfCmd.pos.north, 20.0)

    def test_entity_reset_clears_context_and_boundary_buffers(self) -> None:
        """验证实体 reset 会原地复位 Context、边界缓存和子单元状态。"""

        comm = FormCommInitS(
            netWork=[NetWorkS("A01", "A02", CommDirE.DUPLEX)],
            formPat=[FormPatE.TRIANGLE],
            formPos=[[FormPosS("A01", 0.0, 0.0, 0.0), FormPosS("A02", -30.0, 0.0, -20.0)]],
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
