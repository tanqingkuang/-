"""编队跟随对比分析脚本 LLT。"""

from __future__ import annotations

import unittest

from scripts.analyze_following_schemes import _profile_for_pos_calc
from src.algorithm.context.leaf_types import FormStageE, PosCalcStrategyE, RallyPhaseE
from src.algorithm.entity.leader_follower import FOLLOWER_PROFILE


class FollowingAnalysisTests(unittest.TestCase):
    """验证离线对比只替换临时 Profile，不污染正式配置。"""

    def test_space_profile_replaces_only_formation_pos_calc(self) -> None:
        """空间对照 Profile 应使用 SLOT_GEOMETRY，正式 Profile 仍保持 ROUTE_FORMATION。"""

        profile = _profile_for_pos_calc(PosCalcStrategyE.SLOT_GEOMETRY)

        hold = profile.require_strategies(FormStageE.HOLD, RallyPhaseE.JOINING)
        production_hold = FOLLOWER_PROFILE.require_strategies(
            FormStageE.HOLD, RallyPhaseE.JOINING
        )
        self.assertEqual(hold.pos_calc, PosCalcStrategyE.SLOT_GEOMETRY)
        self.assertEqual(hold.pos_track, production_hold.pos_track)
        self.assertEqual(production_hold.pos_calc, PosCalcStrategyE.ROUTE_FORMATION)

    def test_route_profile_keeps_route_formation_strategy(self) -> None:
        """航线里程对比 Profile 应可独立构造并保持正式策略。"""

        profile = _profile_for_pos_calc(PosCalcStrategyE.ROUTE_FORMATION)

        hold = profile.require_strategies(FormStageE.HOLD, RallyPhaseE.JOINING)
        self.assertEqual(hold.pos_calc, PosCalcStrategyE.ROUTE_FORMATION)


if __name__ == "__main__":
    unittest.main()
