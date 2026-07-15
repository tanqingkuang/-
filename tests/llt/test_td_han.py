"""Han 跟踪微分器与相对槽位 TD 软化的单元测试。"""

from __future__ import annotations

import unittest

from src.algorithm.context.leaf_types import (
    FormPosS,
    FormSnapshotS,
    MotionProfS,
    PosInEarthS,
    VdInEarthS,
)
from src.algorithm.units.algo.pos_calc.base import PosCalcInputS, PosCalcOutputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS
from src.algorithm.units.algo.td_han import TdHan, TdHanInitS

_DT = 0.05


def _td(r: float, h: float = _DT) -> TdHan:
    td = TdHan()
    td.init(TdHanInitS(r=r, h=h))
    return td


class TdHanTests(unittest.TestCase):
    """Han TD 核心性质：无超调收敛、加速度受 r 限、播种对齐、小偏差近似透传。"""

    def test_step_converges_without_overshoot(self) -> None:
        """从 0 阶跃到 10：单调逼近、不超调、稳态到位。"""
        td = _td(r=5.0)
        target, peak, last = 10.0, 0.0, 0.0
        for _ in range(600):  # 30s
            x1, _x2 = td.step(target)
            peak = max(peak, x1)
            last = x1
        self.assertLessEqual(peak, target + 0.05)  # 不超调
        self.assertAlmostEqual(last, target, delta=0.05)  # 到位

    def test_reference_acceleration_bounded_by_r(self) -> None:
        """参考加速度(x2 每拍变化/h)不超过 r。"""
        r = 5.0
        td = _td(r=r)
        prev_v = 0.0
        max_acc = 0.0
        for _ in range(600):
            _x1, x2 = td.step(10.0)
            max_acc = max(max_acc, abs(x2 - prev_v) / _DT)
            prev_v = x2
        self.assertLessEqual(max_acc, r + 1e-6)

    def test_seed_aligns_initial_reference(self) -> None:
        """播种到当前值后，目标即为该值时保持不动(TD 休眠)；目标不同则从种子起步。"""
        td = _td(r=5.0)
        td.seed(3.0, 0.0)
        x1, x2 = td.step(3.0)  # 目标=种子 → 应基本不动
        self.assertAlmostEqual(x1, 3.0, delta=1e-6)
        self.assertAlmostEqual(x2, 0.0, delta=1e-6)

        td2 = _td(r=5.0)
        td2.seed(3.0, 0.0)
        x1b, _ = td2.step(10.0)  # 目标≠种子 → 参考从种子 3 附近起步(不是从 0)
        self.assertAlmostEqual(x1b, 3.0, delta=0.2)

    def test_velocity_bounded_by_vmax(self) -> None:
        """开启 vMax 后，参考速度 x2 不超过 vMax，且仍无超调收敛(梯形速度剖面)。"""
        td = TdHan()
        td.init(TdHanInitS(r=5.0, h=_DT, vMax=3.0))
        target, peak, last, max_v = 60.0, 0.0, 0.0, 0.0
        for _ in range(1000):  # 50s，60m/3≈20s+
            x1, x2 = td.step(target)
            peak = max(peak, x1)
            last = x1
            max_v = max(max_v, abs(x2))
        self.assertLessEqual(max_v, 3.0 + 1e-9)  # 速度受 vMax 限
        self.assertLessEqual(peak, target + 0.05)  # 仍不超调
        self.assertAlmostEqual(last, target, delta=0.1)  # 到位

    def test_small_step_reaches_quickly(self) -> None:
        """小偏差近似透传：0.1 的小阶跃在很少拍内即到位。"""
        td = _td(r=5.0)
        reached = None
        for k in range(200):
            x1, _ = td.step(0.1)
            if abs(x1 - 0.1) < 1e-3 and reached is None:
                reached = k
                break
        self.assertIsNotNone(reached)
        self.assertLess(reached, 40)  # <2s


def _leader_east(vd: float = 20.0, h: float = 1000.0) -> MotionProfS:
    """长机沿正东匀速平飞，航迹系 前向=东、右向=南。"""
    s = MotionProfS()
    s.pos = PosInEarthS(east=0.0, north=0.0, h=h)
    s.v = VdInEarthS(vEast=vd, vNorth=0.0, vUp=0.0, vd=vd, vPsi=0.0, dVPsi=0.0)
    return s


def _two_pattern_init(control_period_s: float) -> SlotGeometryInitS:
    """两队形：pattern0 右偏 +50m，pattern1 右偏 -50m(横向 100m 重构)。"""
    form_pos = [
        [FormPosS(id="F", x=0.0, y=0.0, z=50.0)],
        [FormPosS(id="F", x=0.0, y=0.0, z=-50.0)],
    ]
    return SlotGeometryInitS(
        selfId="F", formPat=["a", "b"], formPos=form_pos, control_period_s=control_period_s
    )


class SlotGeometryTdTests(unittest.TestCase):
    """相对槽位 TD 在 SlotGeometry 中的行为：关闭=旧行为，开启=软化重构阶跃并补速度前馈。"""

    def _run(self, control_period_s: float) -> None:
        self.geo = SlotGeometry()
        self.geo.init(_two_pattern_init(control_period_s))
        self.leader = _leader_east()
        # 僚机初始恰在 pattern0 槽位(东0,北-50)，播种后 TD 休眠。
        self.self_state = MotionProfS()
        self.self_state.pos = PosInEarthS(east=0.0, north=-50.0, h=1000.0)
        self.self_state.v = VdInEarthS(vEast=20.0, vNorth=0.0, vUp=0.0, vd=20.0, vPsi=0.0)
        self.cmd = FormSnapshotS(pattern=0)
        self.out = PosCalcOutputS(selfCmd=MotionProfS())

    def _step(self) -> MotionProfS:
        u = PosCalcInputS(leaderState=self.leader, cmd=self.cmd, selfState=self.self_state)
        self.geo.step(u, self.out)
        return self.out.selfCmd

    def test_disabled_matches_raw_and_jumps_instantly(self) -> None:
        """关闭 TD：pattern 切换后目标位置立即跳到新槽位(旧行为)。"""
        self._run(control_period_s=0.0)
        cmd = self._step()
        self.assertAlmostEqual(cmd.pos.north, -50.0, delta=1e-6)  # 槽位 z=+50 → 北 -50
        self.cmd.pattern = 1  # 重构：z 翻到 -50 → 北 +50
        cmd = self._step()
        self.assertAlmostEqual(cmd.pos.north, 50.0, delta=1e-6)  # 立即跳满 100m

    def test_enabled_softens_reconfig_step(self) -> None:
        """开启 TD：pattern 切换后第一拍只移动一小段，随后平滑收敛到新槽位。"""
        self._run(control_period_s=_DT)
        for _ in range(5):  # 稳态(播种后休眠)
            cmd = self._step()
        self.assertAlmostEqual(cmd.pos.north, -50.0, delta=0.1)

        self.cmd.pattern = 1  # 触发 100m 横向重构
        cmd = self._step()
        # 第一拍远未跳满：仍靠近旧槽位 -50，而不是新槽位 +50。
        self.assertLess(cmd.pos.north, -40.0)
        # 充分收敛到新槽位（位置软化默认开启，速度前馈默认关）。
        for _ in range(460):  # 23s
            cmd = self._step()
        self.assertAlmostEqual(cmd.pos.north, 50.0, delta=1.0)

    def test_slot_velff_flag_emits_feedforward(self) -> None:
        """开启 slotVelFf 后，重构过渡中相对槽位速度(北向前馈)由 TD 的 x2 补出、非零。"""
        self.geo = SlotGeometry()
        init = _two_pattern_init(_DT)
        init.slotVelFf = True
        self.geo.init(init)
        self.leader = _leader_east()
        self.self_state = MotionProfS()
        self.self_state.pos = PosInEarthS(east=0.0, north=-50.0, h=1000.0)
        self.self_state.v = VdInEarthS(vEast=20.0, vNorth=0.0, vUp=0.0, vd=20.0, vPsi=0.0)
        self.cmd = FormSnapshotS(pattern=0)
        self.out = PosCalcOutputS(selfCmd=MotionProfS())
        for _ in range(5):
            self._step()
        self.cmd.pattern = 1
        mid_vnorth = 0.0
        for _ in range(60):
            cmd = self._step()
            mid_vnorth = max(mid_vnorth, abs(cmd.v.vNorth))
        self.assertGreater(mid_vnorth, 0.5)


if __name__ == "__main__":
    unittest.main()
