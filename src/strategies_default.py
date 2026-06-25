"""
Default election strategy implementations.

These reproduce the **Bully algorithm** behaviour that was originally
hard-coded in :class:`ElectionMixin`:

- :class:`HeartbeatTimeoutDetection` — split/merge via heartbeat timeout
- :class:`AnyMemberLeadInvitation` — any member that detects an anomaly
  starts the election
- :class:`BullyReconciliation` — highest-ID wins; all non-leaders send
  their data to the leader

To create a custom policy, subclass the corresponding ABC in
``strategies.py`` and set it as a class-level attribute on the UAV
protocol (see README for examples).
"""
from __future__ import annotations

from typing import Set

from src.strategies import (
    AnomalyDetectionStrategy,
    AnomalyResult,
    CollectionMode,
    ElectionContext,
    InvitationStrategy,
    ReconciliationStrategy,
)

# Re-use the same timeout constant the mixin uses by default.
# Strategies that need a different threshold can define their own.
_DEFAULT_PEER_TIMEOUT = 5.0


# ── Anomaly Detection ─────────────────────────────────────────────────────


class HeartbeatTimeoutDetection(AnomalyDetectionStrategy):
    """Detect splits via heartbeat timeout (member → lead interaction).

    A peer whose last heartbeat is older than ``peer_timeout`` seconds
    is considered out of range.  If the lost peer was the current leader,
    a ``"split"`` trigger is returned; otherwise the lost peers are
    reported but no election trigger is raised.

    Merge detection is handled separately by the mixin when a heartbeat
    from a previously-unknown peer arrives (see
    :pymethod:`ElectionMixin.handle_packet`).
    """

    def __init__(self, peer_timeout: float = _DEFAULT_PEER_TIMEOUT) -> None:
        self.peer_timeout = peer_timeout

    def check_for_anomaly(self, ctx: ElectionContext) -> AnomalyResult:
        lost: Set[int] = set()
        for peer_id, last_seen in ctx.peer_last_seen.items():
            if ctx.current_time - last_seen > self.peer_timeout:
                lost.add(peer_id)

        if not lost:
            return AnomalyResult()

        trigger = None
        if ctx.current_leader_id in lost:
            trigger = "split"

        return AnomalyResult(lost_peers=lost, trigger=trigger)


# ── Invitation ────────────────────────────────────────────────────────────


class AnyMemberLeadInvitation(InvitationStrategy):
    """Any member that detects an anomaly immediately starts an election.

    This is the simplest invitation model: whoever notices the problem
    broadcasts an election message.  There is no lead-to-lead negotiation.
    """

    def should_start_election(
        self, ctx: ElectionContext, anomaly: AnomalyResult,
    ) -> bool:
        # Start an election whenever there is a trigger, regardless of role
        return anomaly.trigger is not None

    def build_election_message(self, ctx: ElectionContext) -> dict:
        return {
            "msg_type": "election",
            "sender_type": "uav",
            "sender_id": ctx.my_id,
        }


# ── Reconciliation ────────────────────────────────────────────────────────


class BullyReconciliation(ReconciliationStrategy):
    """Highest-ID candidate wins; every non-leader sends all data to the
    leader (``FROM_ALL_MEMBERS`` collection)."""

    def choose_leader(self, ctx: ElectionContext, candidates: Set[int]) -> int:
        return max(candidates)

    def get_collection_mode(self) -> CollectionMode:
        return CollectionMode.FROM_ALL_MEMBERS
