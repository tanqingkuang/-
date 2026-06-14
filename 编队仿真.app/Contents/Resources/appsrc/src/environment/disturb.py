"""Disturbance manager.

Centralizes uncertainty index handling, stochastic configuration dispatch, and
runtime dynamic disturbance injection.
"""


class DisturbanceManager:
    """Manage stochastic and dynamic disturbances."""

    def tick(self, dt: float) -> None:
        """Advance dynamic disturbances and push injections to model/comm."""
        raise NotImplementedError

