"""Simulation controller.

Owns lifecycle, scheduling, disturbance ticks, data persistence, and UI data
push according to the architecture design.
"""


class SimulationController:
    """Top-level simulation orchestration facade."""

    def start(self, config: object) -> None:
        """Start a simulation run from a loaded configuration object."""
        raise NotImplementedError

