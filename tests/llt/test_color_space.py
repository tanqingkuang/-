"""3D 显示颜色空间工具测试。"""

from __future__ import annotations

import unittest

import numpy as np

from src.ui.gui.situation3d.color_space import srgb_to_linear


class ColorSpaceTests(unittest.TestCase):
    """锁定标量与数组共用的 sRGB 转换公式。"""

    def test_scalar_uses_standard_piecewise_threshold(self) -> None:
        """标量在阈值两侧分别走线性段和幂函数段。"""

        self.assertAlmostEqual(srgb_to_linear(0.04045), 0.04045 / 12.92)
        self.assertAlmostEqual(srgb_to_linear(1.0), 1.0)

    def test_array_matches_scalar_conversion_elementwise(self) -> None:
        """数组输出形状保持不变，且逐元素结果与标量调用一致。"""

        values = np.array([[0.0, 0.02, 0.5, 1.0]], dtype=np.float32)
        converted = srgb_to_linear(values)

        self.assertIsInstance(converted, np.ndarray)
        self.assertEqual(converted.shape, values.shape)
        np.testing.assert_allclose(
            converted,
            np.array([[srgb_to_linear(float(value)) for value in values[0]]]),
            rtol=1e-6,
            atol=1e-7,
        )


if __name__ == "__main__":
    unittest.main()
