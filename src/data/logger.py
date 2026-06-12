"""Key simulation data logging.

The architecture targets HDF5 for time-series persistence.
"""


class SimulationLogger:
    """Persist key simulation variables."""

    def write(self, record: dict[str, object]) -> None:
        """Write one simulation record."""
        raise NotImplementedError

