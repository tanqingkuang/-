"""Aircraft model iteration and dynamics integration."""


class ModelIterator:
    """Advance aircraft dynamics and accept model-side disturbance injections."""

    def tick(self, dt: float) -> None:
        """Advance all aircraft models by one simulation step."""
        raise NotImplementedError

