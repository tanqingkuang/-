"""公共坐标变换 LLT。注意：锁定 ENU 与前-上-右 FUR 的轴向、手性和往返关系。"""

from __future__ import annotations

import math
import unittest

from src.common.coordinates import (
    enu_to_fur,
    fur_basis_from_angles,
    fur_basis_from_velocity,
    fur_to_enu,
)


def _cross(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    """返回三维叉积，供测试验证 F×U=R。"""

    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


class CoordinateTransformTests(unittest.TestCase):
    """验证公共 ENU/FUR 变换。"""

    def test_level_cardinal_headings_keep_right_axis_on_aircraft_right(self) -> None:
        """四个基本航向下，FUR 的 z 正方向都必须落在飞机右侧。"""

        cases = (
            (0.0, (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
            (math.pi / 2.0, (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
            (math.pi, (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            (-math.pi / 2.0, (0.0, -1.0, 0.0), (-1.0, 0.0, 0.0)),
        )
        for psi, expected_forward, expected_right in cases:
            with self.subTest(psi=psi):
                forward, up_normal, right = fur_basis_from_angles(0.0, psi)
                for actual, expected in zip(forward, expected_forward, strict=True):
                    self.assertAlmostEqual(actual, expected, places=12)
                self.assertEqual(up_normal, (0.0, 0.0, 1.0))
                for actual, expected in zip(right, expected_right, strict=True):
                    self.assertAlmostEqual(actual, expected, places=12)

    def test_climbing_basis_is_orthonormal_and_right_handed(self) -> None:
        """爬升状态仍须保持单位正交基，且满足 F×U=R。"""

        basis = fur_basis_from_angles(math.radians(30.0), math.radians(40.0))
        forward, up_normal, right = basis
        for axis in basis:
            self.assertAlmostEqual(sum(value * value for value in axis), 1.0, places=12)
        self.assertAlmostEqual(sum(a * b for a, b in zip(forward, up_normal, strict=True)), 0.0, places=12)
        self.assertAlmostEqual(sum(a * b for a, b in zip(forward, right, strict=True)), 0.0, places=12)
        self.assertAlmostEqual(sum(a * b for a, b in zip(up_normal, right, strict=True)), 0.0, places=12)
        for actual, expected in zip(_cross(forward, up_normal), right, strict=True):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_velocity_basis_and_round_trip_match_angle_basis(self) -> None:
        """速度建基与角度建基必须一致，任意向量 ENU→FUR→ENU 后保持不变。"""

        theta = math.radians(-20.0)
        psi = math.radians(135.0)
        speed = 37.0
        velocity = (
            speed * math.cos(theta) * math.cos(psi),
            speed * math.cos(theta) * math.sin(psi),
            speed * math.sin(theta),
        )
        basis_from_angles = fur_basis_from_angles(theta, psi)
        basis_from_velocity = fur_basis_from_velocity(velocity)
        for actual_axis, expected_axis in zip(basis_from_velocity, basis_from_angles, strict=True):
            for actual, expected in zip(actual_axis, expected_axis, strict=True):
                self.assertAlmostEqual(actual, expected, places=12)

        vector_enu = (12.5, -3.25, 7.75)
        vector_fur = enu_to_fur(vector_enu, basis_from_velocity)
        round_trip = fur_to_enu(vector_fur, basis_from_velocity)
        for actual, expected in zip(round_trip, vector_enu, strict=True):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_velocity_basis_rejects_undefined_horizontal_heading(self) -> None:
        """零速和纯垂直速度都无法定义前-上-右航迹系，应显式失败。"""

        for velocity in ((0.0, 0.0, 0.0), (0.0, 0.0, 10.0)):
            with self.subTest(velocity=velocity), self.assertRaisesRegex(ValueError, "horizontal"):
                fur_basis_from_velocity(velocity)

    def test_basis_rejects_non_finite_angles_and_velocity(self) -> None:
        """坐标基不能把 NaN/Inf 扩散到后续控制链。"""

        with self.assertRaisesRegex(ValueError, "angles"):
            fur_basis_from_angles(float("nan"), 0.0)
        with self.assertRaisesRegex(ValueError, "velocity"):
            fur_basis_from_velocity((1.0, float("inf"), 0.0))


if __name__ == "__main__":
    unittest.main()
