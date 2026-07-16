"""编队入站处理包。注意：Rally 实体统一使用 FormationInbound。"""

from src.algorithm.units.process.inbound.formation import (
    FormationInbound,
    FormationInboundInitS,
)

__all__ = [
    "FormationInbound",
    "FormationInboundInitS",
]
