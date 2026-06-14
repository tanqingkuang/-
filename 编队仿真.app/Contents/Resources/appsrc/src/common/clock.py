"""Simulation clock utilities."""


class SimulationClock:
    """Track simulation time."""

    def __init__(self) -> None:
        self.time = 0.0

    def tick(self, dt: float) -> float:
        """Advance and return current simulation time."""
        self.time += dt
        return self.time

