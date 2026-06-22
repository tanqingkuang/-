"""Base API for formation entities."""

from __future__ import annotations

from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS


class EntityBase:
    def init(self, cfg: EntityInitS) -> None:
        raise NotImplementedError

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
