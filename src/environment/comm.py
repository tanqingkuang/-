"""Communication channel simulation."""


class CommunicationChannel:
    """Route messages by topology and QoS configuration."""

    def tick(self, dt: float) -> None:
        """Advance in-flight messages and link-side stochastic behavior."""
        raise NotImplementedError

