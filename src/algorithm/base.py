"""Algorithm base types and message schema declaration API."""


class AlgorithmBase:
    """Base class for coordination and node algorithm plugins."""

    def declare_message_schema(self) -> dict[str, object]:
        """Return message schema metadata required by the communication layer."""
        raise NotImplementedError

