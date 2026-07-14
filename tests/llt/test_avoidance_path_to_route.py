"""Tests for the avoidance exit translation: LOS simplification + arc route building."""

from __future__ import annotations

import math
import unittest

from src.algorithm.units.algo.arc_path import arc_radius, segment_length
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.process.tra_plan.avoidance.astar import plan_path
from src.algorithm.units.process.tra_plan.avoidance.obstacle import (
    blocked,
    make_circle,
)
from src.algorithm.context.leaf_types import PosInEarthS, WayLineS, WayPointInputS, WayPointS
from src.algorithm.units.algo.arc_path import arc_swept_rad
from src.algorithm.units.process.tra_plan.avoidance.path_to_route import (
    assign_transition_radius,
    bake_obstacle_hug_arcs,
    bake_transition_arcs,
    line_of_sight_clear,
    points_to_route,
    simplify_path,
    simplify_path_with_causes,
)


class LineOfSightTests(unittest.TestCase):
    def test_clear_when_no_obstacles(self) -> None:
        self.assertTrue(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), []))

    def test_blocked_when_segment_crosses_obstacle(self) -> None:
        obstacles = [make_circle("C", 50.0, 0.0, 20.0)]
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles))

    def test_clear_when_segment_passes_beside_obstacle(self) -> None:
        obstacles = [make_circle("C", 50.0, 0.0, 20.0)]
        self.assertTrue(line_of_sight_clear((0.0, 100.0), (100.0, 100.0), obstacles))

    def test_clearance_widens_blocking(self) -> None:
        obstacles = [make_circle("C", 50.0, 40.0, 20.0)]
        self.assertTrue(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles))
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles, clearance=30.0))

    def test_sample_step_is_spacing_upper_bound_not_floored(self) -> None:
        # length=19、sample_step=10：int 向下取整只采两端点会漏掉中间细障碍；ceil 必须采到 (9.5,0)。
        obstacles = [make_circle("C", 9.5, 0.0, 2.0)]
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (19.0, 0.0), obstacles, sample_step=10.0))

    def test_invalid_sample_step_raises(self) -> None:
        obstacles = [make_circle("C", 5.0, 0.0, 1.0)]
        with self.assertRaises(ValueError):
            line_of_sight_clear((0.0, 0.0), (10.0, 0.0), obstacles, sample_step=0.0)
        with self.assertRaises(ValueError):
            line_of_sight_clear((0.0, 0.0), (10.0, 0.0), obstacles, sample_step=-1.0)


class SimplifyPathTests(unittest.TestCase):
    def test_collinear_points_reduced_to_endpoints(self) -> None:
        points = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0), (40.0, 0.0)]
        self.assertEqual(simplify_path(points, []), [(0.0, 0.0), (40.0, 0.0)])

    def test_short_path_returned_as_is(self) -> None:
        self.assertEqual(simplify_path([(0.0, 0.0), (10.0, 0.0)], []), [(0.0, 0.0), (10.0, 0.0)])

    def test_staircase_without_obstacles_straightens(self) -> None:
        # 无障碍时锯齿应被拉直为两端点。
        points = [(0.0, 0.0), (10.0, 10.0), (20.0, 20.0), (30.0, 30.0)]
        self.assertEqual(simplify_path(points, []), [(0.0, 0.0), (30.0, 30.0)])

    def test_simplified_segments_do_not_cross_obstacle(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        raw = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(raw)
        simplified = simplify_path(raw, obstacles, clearance=clearance)
        # 去冗余后拐点数显著少于原始格点。
        self.assertLess(len(simplified), len(raw))
        self.assertGreaterEqual(len(simplified), 2)
        # 端点保持不变。
        self.assertEqual(simplified[0], raw[0])
        self.assertEqual(simplified[-1], raw[-1])
        # 每条拉直后的腿都不得穿过障碍。
        for a, b in zip(simplified, simplified[1:]):
            self.assertTrue(line_of_sight_clear(a, b, obstacles, clearance=clearance), f"leg {a}-{b} crosses obstacle")


def _route_with_radius(points, *, turn_radius_m, speed_mps, **kw):
    """生产端真实顺序：摆点(r=0) → 补交接半径。"""
    wpi = points_to_route(points, speed_mps=speed_mps, **kw)
    assign_transition_radius(wpi, turn_radius_m)
    return wpi


class PointsToRouteTests(unittest.TestCase):
    def test_two_points_make_wpi_list(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (100.0, 0.0)], speed_mps=20.0)
        self.assertEqual(len(wpi), 2)
        self.assertEqual(wpi[0].r, 0.0)
        self.assertEqual(wpi[1].r, 0.0)
        self.assertEqual((wpi[0].pos.east, wpi[0].pos.north), (0.0, 0.0))
        self.assertEqual((wpi[1].pos.east, wpi[1].pos.north), (100.0, 0.0))
        self.assertEqual(wpi[0].vdCmd, 20.0)

    def test_only_places_points_r_always_zero(self) -> None:
        # points_to_route 不决策 r，内部拐点也保持 0（由 assign_transition_radius 后补）。
        wpi = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], speed_mps=20.0)
        self.assertTrue(all(w.r == 0.0 for w in wpi))
        self.assertTrue(all(w.turnSign == 0.0 for w in wpi))

    def test_corner_arc_geometry_via_conversion(self) -> None:
        # L 形拐点，R 足够小可装进两腿 → 补 R 后转换得到直线+圆弧+直线段。
        wpi = _route_with_radius(
            [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0
        )
        lines = waypoint_inputs_to_waylines(wpi)
        arcs = [ln for ln in lines if ln.start.turnSign != 0.0]
        self.assertEqual(len(arcs), 1)
        arc = arcs[0]
        # 左转（东→北）turnSign 应为 +1，且圆心到两切点距离均为 R。
        self.assertEqual(arc.start.turnSign, 1.0)
        r = arc_radius(arc)
        self.assertAlmostEqual(r, 200.0)
        r_end = math.hypot(arc.end.pos.east - arc.start.center.east, arc.end.pos.north - arc.start.center.north)
        self.assertAlmostEqual(r_end, 200.0, places=6)
        # 圆弧段长 = R·|扫掠角|，90° 转弯 ≈ R·pi/2。
        self.assertAlmostEqual(segment_length(arc), 200.0 * math.pi / 2.0, places=3)

    def test_oversized_corner_radius_falls_back_to_straight_lines(self) -> None:
        """圆弧切点超出相邻腿长时，应退回原始折线而不是插入腿外圆弧。"""

        wpi = _route_with_radius(
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], turn_radius_m=100.0, speed_mps=20.0
        )
        lines = waypoint_inputs_to_waylines(wpi)

        self.assertEqual(len(lines), 2)
        self.assertTrue(all(line.start.turnSign == 0.0 for line in lines))
        self.assertAlmostEqual(lines[0].end.pos.east, 10.0)
        self.assertAlmostEqual(lines[0].end.pos.north, 0.0)
        self.assertAlmostEqual(lines[1].start.pos.east, 10.0)
        self.assertAlmostEqual(lines[1].start.pos.north, 0.0)

    def test_with_radius_inserts_arc_via_conversion(self) -> None:
        wpi = _route_with_radius([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0)
        lines = waypoint_inputs_to_waylines(wpi)
        self.assertTrue(any(ln.start.turnSign != 0.0 for ln in lines))

    def test_endpoints_have_correct_pos(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0), (1000.0, 500.0)], speed_mps=18.0
        )
        self.assertEqual((wpi[0].pos.east, wpi[0].pos.north), (0.0, 0.0))
        self.assertEqual((wpi[-1].pos.east, wpi[-1].pos.north), (1000.0, 500.0))

    def test_altitude_applied(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (100.0, 0.0)], speed_mps=20.0, altitude_m=1000.0)
        self.assertEqual(wpi[0].pos.h, 1000.0)
        self.assertEqual(wpi[1].pos.h, 1000.0)

    def test_per_point_altitudes_applied(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (100.0, 0.0)], speed_mps=20.0, altitudes=[1000.0, 1200.0]
        )
        self.assertEqual(wpi[0].pos.h, 1000.0)
        self.assertEqual(wpi[1].pos.h, 1200.0)

    def test_altitudes_length_mismatch_raises(self) -> None:
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0), (100.0, 0.0)], speed_mps=20.0, altitudes=[1000.0])

    def test_invalid_inputs_raise(self) -> None:
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0)], speed_mps=20.0)
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0), (1.0, 0.0)], speed_mps=-1.0)


class AssignTransitionRadiusTests(unittest.TestCase):
    """补充函数：可飞性校验后，给两侧均为直线段的内部拐点补交接半径 R。"""

    @staticmethod
    def _wpi(east: float, north: float, *, turn_sign: float = 0.0) -> WayPointInputS:
        return WayPointInputS(pos=PosInEarthS(east, north, 0.0), turnSign=turn_sign)

    def test_straight_straight_interior_gets_radius(self) -> None:
        inputs = [self._wpi(0.0, 0.0), self._wpi(1000.0, 0.0), self._wpi(1000.0, 1000.0)]
        assign_transition_radius(inputs, 400.0)
        self.assertEqual(inputs[0].r, 0.0)  # 首点不补
        self.assertEqual(inputs[1].r, 400.0)  # 直-直内部拐点补 R
        self.assertEqual(inputs[2].r, 0.0)  # 末点不补

    def test_endpoints_never_get_radius(self) -> None:
        inputs = [self._wpi(0.0, 0.0), self._wpi(100.0, 0.0)]
        assign_transition_radius(inputs, 300.0)
        self.assertEqual(inputs[0].r, 0.0)
        self.assertEqual(inputs[1].r, 0.0)

    def test_interior_next_to_incoming_curve_stays_zero(self) -> None:
        # 入段(0->1)为曲线(turnSign!=0) → 拐点 1 不补 R，交给曲线自身。
        inputs = [self._wpi(0.0, 0.0, turn_sign=1.0), self._wpi(1000.0, 0.0), self._wpi(1000.0, 1000.0)]
        assign_transition_radius(inputs, 400.0)
        self.assertEqual(inputs[1].r, 0.0)

    def test_interior_next_to_outgoing_curve_stays_zero(self) -> None:
        # 出段(1->2)为曲线 → 拐点 1 不补 R。
        inputs = [self._wpi(0.0, 0.0), self._wpi(1000.0, 0.0, turn_sign=-1.0), self._wpi(1000.0, 1000.0)]
        assign_transition_radius(inputs, 400.0)
        self.assertEqual(inputs[1].r, 0.0)

    def test_overwrites_preexisting_radius(self) -> None:
        # 邻段变曲线时，旧的 r 必须被清掉，避免残留。
        inputs = [self._wpi(0.0, 0.0), self._wpi(1000.0, 0.0, turn_sign=1.0), self._wpi(1000.0, 1000.0)]
        inputs[1].r = 999.0
        assign_transition_radius(inputs, 400.0)
        self.assertEqual(inputs[1].r, 0.0)

    def test_negative_radius_raises(self) -> None:
        with self.assertRaises(ValueError):
            assign_transition_radius([self._wpi(0.0, 0.0), self._wpi(1.0, 0.0)], -1.0)


class BakeTransitionArcsTests(unittest.TestCase):
    """allow_arc=True 时把带交接半径 r 的直线-直线拐点烘焙成相切圆弧段(turnSign!=0)。"""

    def _route(self, pts, r):
        wpi = points_to_route(pts, speed_mps=20.0)
        assign_transition_radius(wpi, r)
        return wpi

    def test_straight_corner_baked_to_arc(self) -> None:
        baked = bake_transition_arcs(self._route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], 200.0))
        # 一个拐点 → 两切点替换，航点数 3→4。
        self.assertEqual(len(baked), 4)
        arc_nodes = [w for w in baked if w.turnSign != 0.0]
        self.assertEqual(len(arc_nodes), 1)
        self.assertEqual(arc_nodes[0].turnSign, 1.0)  # 东→北 左转
        # 烘焙后不再留 r(转弯信息已变成航段曲率)。
        self.assertTrue(all(w.r == 0.0 for w in baked))

    def test_oversized_radius_stays_sharp(self) -> None:
        # 半径过大、切点越出腿长 → 保持尖角，不烘焙。
        baked = bake_transition_arcs(self._route([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], 100.0))
        self.assertEqual(len(baked), 3)
        self.assertTrue(all(w.turnSign == 0.0 for w in baked))

    def test_zero_radius_untouched(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], speed_mps=20.0)  # 全 r=0
        baked = bake_transition_arcs(wpi)
        self.assertEqual(len(baked), 3)
        self.assertTrue(all(w.turnSign == 0.0 for w in baked))


class SimplifyPathCausesTests(unittest.TestCase):
    def test_causes_tag_blocking_circle(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        raw = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(raw)
        pts, causes = simplify_path_with_causes(raw, obstacles, clearance=clearance)
        self.assertEqual(len(pts), len(causes))
        self.assertIsNone(causes[0])  # 首点无来源
        self.assertIsNone(causes[-1])  # 末点无来源
        # 绕障保留的内部拐点应标注为被 C 逼出。
        self.assertTrue(any(c is not None and c.id == "C" for c in causes))

    def test_no_obstacle_all_causes_none(self) -> None:
        pts, causes = simplify_path_with_causes([(0.0, 0.0), (10.0, 10.0), (20.0, 20.0)], [])
        self.assertTrue(all(c is None for c in causes))


class BakeObstacleHugArcsTests(unittest.TestCase):
    """连续贴同一圆的拐点串应折叠成一段沿膨胀圆的真圆弧(圆心=障碍中心、半径=r+间距)。"""

    @staticmethod
    def _wpi(east: float, north: float) -> WayPointInputS:
        return WayPointInputS(pos=PosInEarthS(east, north, 0.0), vdCmd=20.0)

    def _hug_route(self):
        obstacle = make_circle("C", 500.0, 0.0, 150.0)
        hug_clearance = 100.0
        r_inf = obstacle.radius + hug_clearance  # 250
        on = lambda deg: self._wpi(
            500.0 + r_inf * math.cos(math.radians(deg)), r_inf * math.sin(math.radians(deg))
        )
        # 自由点落在首/尾顶点的切线上(合法擦入位置)；中间三点贴在膨胀圆底部、极角单调递增(CCW)。
        route = [self._wpi(273.5, -183.7), on(250.0), on(270.0), on(290.0), self._wpi(726.5, -183.7)]
        causes = [None, obstacle, obstacle, obstacle, None]
        return route, causes, obstacle, hug_clearance

    def test_consecutive_hug_collapsed_to_single_arc(self) -> None:
        route, causes, obstacle, hug_clearance = self._hug_route()
        out = bake_obstacle_hug_arcs(
            route, causes, [obstacle], turn_radius_m=150.0, hug_clearance=hug_clearance
        )
        # 三个贴障顶点 → 一段弧(两切点)：5 → 4。
        self.assertEqual(len(out), 4)
        arc_nodes = [w for w in out if w.turnSign != 0.0]
        self.assertEqual(len(arc_nodes), 1)
        arc = arc_nodes[0]
        self.assertEqual(arc.turnSign, 1.0)  # 底部从左向右贴 = CCW
        self.assertAlmostEqual(arc.center.east, 500.0, places=6)
        self.assertAlmostEqual(arc.center.north, 0.0, places=6)
        radius = math.hypot(arc.pos.east - arc.center.east, arc.pos.north - arc.center.north)
        self.assertAlmostEqual(radius, 250.0, places=6)  # 半径=障碍膨胀半径，非最小转弯半径
        # 扫掠角应约等于顶点跨度(250°→290°≈40°)的底部局部弧，而非绕大半圈。
        idx = out.index(arc)
        line = WayLineS(
            start=WayPointS(pos=arc.pos, turnSign=arc.turnSign, center=arc.center),
            end=WayPointS(pos=out[idx + 1].pos),
        )
        self.assertAlmostEqual(math.degrees(arc_swept_rad(line)), 40.0, delta=5.0)

    def test_hug_arc_does_not_wrap_long_way(self) -> None:
        # 回归 PR#83 检视意见1：~40° 的底部贴障串(250/270/290)绝不能被折成绕大半圈的弧。
        # 自由点远在圆下方、不在合法切线擦入位置时，宁可不折叠也不能产出 >180° 的怪弧。
        obstacle = make_circle("C", 500.0, 0.0, 150.0)
        on = lambda deg: self._wpi(
            500.0 + 250.0 * math.cos(math.radians(deg)), 250.0 * math.sin(math.radians(deg))
        )
        route = [self._wpi(100.0, -400.0), on(250.0), on(270.0), on(290.0), self._wpi(900.0, -400.0)]
        causes = [None, obstacle, obstacle, obstacle, None]
        out = bake_obstacle_hug_arcs(route, causes, [obstacle], turn_radius_m=150.0, hug_clearance=100.0)
        for node, nxt in zip(out, out[1:]):
            if node.turnSign != 0.0:
                line = WayLineS(
                    start=WayPointS(pos=node.pos, turnSign=node.turnSign, center=node.center),
                    end=WayPointS(pos=nxt.pos),
                )
                self.assertLessEqual(
                    abs(math.degrees(arc_swept_rad(line))), 180.0, "贴障弧不应绕大半圈"
                )

    def test_inflated_radius_smaller_than_turn_radius_not_collapsed(self) -> None:
        route, causes, obstacle, hug_clearance = self._hug_route()
        # R=300 > 膨胀半径 250：贴不住，保持原顶点不折叠。
        out = bake_obstacle_hug_arcs(
            route, causes, [obstacle], turn_radius_m=300.0, hug_clearance=hug_clearance
        )
        self.assertEqual(len(out), len(route))
        self.assertTrue(all(w.turnSign == 0.0 for w in out))

    def test_single_hug_vertex_not_collapsed(self) -> None:
        obstacle = make_circle("C", 500.0, 0.0, 150.0)
        route = [self._wpi(100.0, -400.0), self._wpi(500.0, -250.0), self._wpi(900.0, -400.0)]
        causes = [None, obstacle, None]  # 仅单顶点擦边
        out = bake_obstacle_hug_arcs(route, causes, [obstacle], turn_radius_m=150.0, hug_clearance=100.0)
        self.assertEqual(len(out), 3)
        self.assertTrue(all(w.turnSign == 0.0 for w in out))

    def test_adjacent_circles_connected_by_common_tangent(self) -> None:
        # 两障碍被连续贴(中间无自由顶点)，应得两段大弧 + 一条同时相切两圆的衔接直线。
        c1 = make_circle("C1", 0.0, 0.0, 100.0)
        c2 = make_circle("C2", 900.0, 0.0, 100.0)
        hug = 100.0
        r_inf = 200.0  # 100 + 100
        on1 = lambda deg: self._wpi(r_inf * math.cos(math.radians(deg)), r_inf * math.sin(math.radians(deg)))
        on2 = lambda deg: self._wpi(900.0 + r_inf * math.cos(math.radians(deg)), r_inf * math.sin(math.radians(deg)))
        # 都贴顶部、从左向右(CW, sign=-1)，极角单调递减：C1 150→120→90，C2 90→60→30。
        # 自由端点落在首/尾顶点的切线上(擦入 C1@150°、擦出 C2@30°)，保证弧覆盖整个顶点跨度。
        route = [
            self._wpi(-298.2, -116.5),  # 自由起点(C1@150° 切线上)
            on1(150.0), on1(120.0), on1(90.0),  # 贴 C1 顶
            on2(90.0), on2(60.0), on2(30.0),  # 贴 C2 顶（与 C1 段相邻，无自由顶点间隔）
            self._wpi(1198.2, -116.5),  # 自由终点(C2@30° 切线上)
        ]
        causes = [None, c1, c1, c1, c2, c2, c2, None]
        out = bake_obstacle_hug_arcs(route, causes, [c1, c2], turn_radius_m=150.0, hug_clearance=hug)
        arcs = [w for w in out if w.turnSign != 0.0]
        self.assertEqual(len(arcs), 2)  # 两段贴障弧
        centers = {(round(a.center.east), round(a.center.north)) for a in arcs}
        self.assertEqual(centers, {(0, 0), (900, 0)})
        # 找到 C1 出切点 → C2 入切点这条衔接直线，验证它同时相切两圆(到两圆心距离=膨胀半径)。
        # 弧节点后紧跟其出切点节点(turnSign==0)；C1 出切点是 arcs[0] 之后的节点。
        idx1 = out.index(arcs[0])
        t_out1 = out[idx1 + 1].pos  # C1 出切点
        t_in2 = arcs[1].pos  # C2 入切点(弧起点)
        d1 = math.hypot(t_out1.east - 0.0, t_out1.north - 0.0)
        d2 = math.hypot(t_in2.east - 900.0, t_in2.north - 0.0)
        self.assertAlmostEqual(d1, r_inf, places=3)
        self.assertAlmostEqual(d2, r_inf, places=3)
        # 衔接直线方向应分别与两端半径垂直(相切)。
        seg = (t_in2.east - t_out1.east, t_in2.north - t_out1.north)
        rad1 = (t_out1.east - 0.0, t_out1.north - 0.0)
        rad2 = (t_in2.east - 900.0, t_in2.north - 0.0)
        self.assertAlmostEqual(seg[0] * rad1[0] + seg[1] * rad1[1], 0.0, places=2)
        self.assertAlmostEqual(seg[0] * rad2[0] + seg[1] * rad2[1], 0.0, places=2)


class AStarToRouteIntegrationTests(unittest.TestCase):
    def test_full_chain_plan_simplify_route(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        raw = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=20.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(raw)
        simplified = simplify_path(raw, obstacles, clearance=clearance)
        wpi = _route_with_radius(simplified, turn_radius_m=80.0, speed_mps=20.0, altitude_m=1000.0)
        lines = waypoint_inputs_to_waylines(wpi)
        self.assertGreaterEqual(len(lines), 1)
        # 起终点与规划一致。
        self.assertAlmostEqual(lines[0].start.pos.east, 0.0)
        self.assertAlmostEqual(lines[-1].end.pos.east, 1000.0)
        # 直线段端点应在障碍外（圆弧外凸是否触障由步骤4 校验，这里只查直线骨架）。
        for line in lines:
            if line.start.turnSign == 0.0:
                self.assertFalse(blocked(obstacles, line.start.pos.east, line.start.pos.north, clearance))
                self.assertFalse(blocked(obstacles, line.end.pos.east, line.end.pos.north, clearance))


if __name__ == "__main__":
    unittest.main()
