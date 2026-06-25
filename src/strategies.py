"""
Abstract strategy interfaces for leader-election policy customisation.

Three independent policy axes can be swapped without modifying the core
:class:`ElectionMixin` lifecycle:

1. **Anomaly Detection** — *How* are splits / merges detected?
2. **Invitation** — *Who* starts the election and *how* are peers invited?
3. **Reconciliation** — *Who* becomes leader and *how* is state collected?

Developers implement one or more of these ABCs and set them as class-level
attributes on the UAV protocol class (see ``strategies_default.py`` for
the built-in implementations).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, Optional, Set


# ── Value objects returned by strategies ───────────────────────────────────


@dataclass
class AnomalyResult:
    """Outcome of a single anomaly-detection check.

    Attributes:
        lost_peers: Peer IDs that should be removed from the known set.
        new_peers:  Peer IDs that appeared since the last check.
        trigger:    ``"split"``, ``"merge"``, or ``None`` when no event.
    """
    lost_peers: Set[int] = field(default_factory=set)
    new_peers: Set[int] = field(default_factory=set)
    trigger: Optional[str] = None


class CollectionMode(Enum):
    """How the new leader collects replicated state after an election."""
    FROM_SUB_LEADERS = auto()
    """Collect only from leaders of former sub-swarms."""
    FROM_ALL_MEMBERS = auto()
    """Collect data from every member individually."""


# ── Read-only context snapshot passed to strategies ────────────────────────


@dataclass
class ElectionContext:
    """Read-only snapshot of election-relevant state.

    Built by :class:`ElectionMixin` before each strategy call so that
    strategies never need a reference to the full mixin/protocol.
    """
    my_id: int
    is_leader: bool
    current_leader_id: Optional[int]
    known_peers: Set[int]
    peer_last_seen: Dict[int, float]
    current_time: float
    election_in_progress: bool
    packets: Dict[str, int]
    election_buffer: Dict[str, int]

    # Callback the strategy can use to broadcast a message dict
    broadcast: Callable[[dict], None] = field(repr=False, default=None)


# ── Abstract strategy interfaces ──────────────────────────────────────────


class AnomalyDetectionStrategy(ABC):
    """Detects split / merge events by inspecting peer state."""

    @abstractmethod
    def check_for_anomaly(self, ctx: ElectionContext) -> AnomalyResult:
        """Inspect peer heartbeats (or other signals) and return any
        detected anomalies.

        The mixin will handle removing lost peers and updating internal
        bookkeeping; the strategy only needs to *identify* them.
        """
        ...


class InvitationStrategy(ABC):
    """Decides whether *this* node should initiate an election and how
    the election message is built."""

    @abstractmethod
    def should_start_election(
        self, ctx: ElectionContext, anomaly: AnomalyResult,
    ) -> bool:
        """Given a detected anomaly, return ``True`` if this node should
        start a new election round."""
        ...

    @abstractmethod
    def build_election_message(self, ctx: ElectionContext) -> dict:
        """Build the JSON-serialisable dict that will be broadcast to
        invite other nodes into the election."""
        ...


class ReconciliationStrategy(ABC):
    """Decides who becomes leader and how state is consolidated
    after the election completes."""

    @abstractmethod
    def choose_leader(self, ctx: ElectionContext, candidates: Set[int]) -> int:
        """Given a set of candidate node IDs (including self), return the
        ID that should become leader."""
        ...

    @abstractmethod
    def get_collection_mode(self) -> CollectionMode:
        """Return how the new leader should collect replicated state from
        the swarm members."""
        ...
