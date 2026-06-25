class LamportClock:
    """
    Minimal Lamport logical clock for a single process.

    Rules:
      • tick()   — internal event: increment and return new time.
      • update() — receive event: set time = max(local, received) + 1.
      • time     — read current value without advancing.
    """

    def __init__(self) -> None:
        self._t: int = 0

    def tick(self) -> int:
        """Advance clock for an internal or send event."""
        self._t += 1
        return self._t

    def update(self, received: int) -> int:
        """Advance clock on message receive (Lamport receive rule)."""
        self._t = max(self._t, received) + 1
        return self._t

    @property
    def time(self) -> int:
        return self._t
