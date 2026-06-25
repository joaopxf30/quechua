"""
Bully-style leader election mixin for UAV protocols.

This mixin is designed to sit between the visualization mixin and the base
UAV protocol in the MRO::

    class MyUAV(UAVVizMixin, ElectionMixin, make_uav_protocol(...)):
        pass

Policy customisation
--------------------
Three class-level attributes control the election behaviour.  Override
them with your own :class:`~src.strategies.AnomalyDetectionStrategy`,
:class:`~src.strategies.InvitationStrategy`, or
:class:`~src.strategies.ReconciliationStrategy` to swap algorithms
without touching the core lifecycle (see ``strategies_default.py`` for
the built-in implementations and the README for usage examples).

Behaviour (defaults)
--------------------
- **Pre-split**: the initial leader is predefined (highest node ID in the
  group).  No election is run until the first split or merge event.
- **Split detection**: when heartbeats from a known peer stop arriving for
  ``PEER_TIMEOUT`` seconds, that peer is considered out of range.  If the
  lost peer was the leader, a new election is triggered in the local
  partition.
- **Merge detection**: when a heartbeat arrives from a previously-unknown
  peer, a merge event is detected and a new election is triggered.
- **During election**: sensor data is still collected but accumulated in a
  separate ``_election_buffer``.  The normal gossip/transfer path continues
  for ``self.packets`` so nothing already collected is lost.
- **After election**: non-leaders send a ``data_transfer`` message with
  their full buffer (``packets`` + ``_election_buffer``) to the leader, then
  clear both.  The leader merges incoming transfers.

Metrics
-------
Every election event is recorded in
``utils.metrics.election_stats.ELECTION_LOG`` with start/end times,
duration, winner ID, participants, and per-UAV buffer sizes at transfer
time.
"""
import json
import logging
from typing import Dict, Optional, Set

from gradysim.protocol.messages.communication import BroadcastMessageCommand

from utils.metrics.election_stats import ELECTION_LOG, ElectionEvent

from src.strategies import (
    AnomalyDetectionStrategy,
    CollectionMode,
    ElectionContext,
    InvitationStrategy,
    ReconciliationStrategy,
)
from src.strategies_default import (
    AnyMemberLeadInvitation,
    BullyReconciliation,
    HeartbeatTimeoutDetection,
)

# ── Tuning constants (not in config.py — this is example code) ────────────

ELECTION_WAIT = 3.0       # seconds to wait for "alive" before declaring victory
PEER_TIMEOUT = 5.0        # seconds without heartbeat → peer is out of range
ELECTION_CHECK_INTERVAL = 1.0  # how often to scan for split / merge


class ElectionMixin:
    """
    Bully leader-election mixin.

    Expected attributes on ``self`` (provided by the base UAV protocol):
        provider            — GrADySim node provider
        packets             — Dict[str, int]  sensor → count
        _lamport            — LamportClock instance
        _cluster_index      — int
        _num_uavs           — int

    **Strategy attributes** (class-level, override to customise):

    .. attribute:: anomaly_strategy
       :type: AnomalyDetectionStrategy

       Controls *how* splits and merges are detected.
       Default: :class:`HeartbeatTimeoutDetection`.

    .. attribute:: invitation_strategy
       :type: InvitationStrategy

       Controls *who* starts an election and *how* peers are invited.
       Default: :class:`AnyMemberLeadInvitation`.

    .. attribute:: reconciliation_strategy
       :type: ReconciliationStrategy

       Controls *who* becomes leader and *how* state is consolidated.
       Default: :class:`BullyReconciliation`.

    The mixin adds its own timers (``election_check``, ``election_timeout``)
    and message types (``election``, ``alive``, ``leader``, ``data_transfer``)
    that coexist with the base protocol's heartbeat messages.
    """

    # ── Class-level strategy defaults ──────────────────────────────────
    # Override these in a subclass or in the factory to change behaviour.

    anomaly_strategy: AnomalyDetectionStrategy = HeartbeatTimeoutDetection()
    invitation_strategy: InvitationStrategy = AnyMemberLeadInvitation()
    reconciliation_strategy: ReconciliationStrategy = BullyReconciliation()

    # ── Initialization ─────────────────────────────────────────────────

    def initialize(self) -> None:
        super().initialize()

        # Election state
        self._is_leader: bool = False
        self._current_leader_id: Optional[int] = None
        self._election_in_progress: bool = False
        self._received_alive: bool = False
        self._election_start_time: Optional[float] = None
        self._election_id_counter: int = 0

        # Buffer for sensor data collected during an election
        self._election_buffer: Dict[str, int] = {}

        # Peer tracking for split / merge detection
        self._known_peers: Set[int] = set()
        self._peer_last_seen: Dict[int, float] = {}

        # Metrics: per-election buffer snapshot
        self._current_election_event: Optional[ElectionEvent] = None

        # Predefined leader: highest ID will be set on first heartbeat exchange
        # (we can't know the max ID at init time, so we defer)
        self._leader_predefined: bool = False

        # Schedule the periodic split / merge detector
        self._schedule_election_check()

    # ── Context builder ────────────────────────────────────────────────

    def _build_election_context(self) -> ElectionContext:
        """Create a read-only snapshot of election state for strategies."""
        return ElectionContext(
            my_id=self.provider.get_id(),
            is_leader=self._is_leader,
            current_leader_id=self._current_leader_id,
            known_peers=set(self._known_peers),       # defensive copy
            peer_last_seen=dict(self._peer_last_seen), # defensive copy
            current_time=self.provider.current_time(),
            election_in_progress=self._election_in_progress,
            packets=dict(self.packets),
            election_buffer=dict(self._election_buffer),
            broadcast=self._broadcast_dict,
        )

    def _broadcast_dict(self, msg: dict) -> None:
        """Convenience: JSON-encode *msg* and broadcast it."""
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(msg))
        )

    # ── Timer scheduling ───────────────────────────────────────────────

    def _schedule_election_check(self) -> None:
        self.provider.schedule_timer(
            "election_check",
            self.provider.current_time() + ELECTION_CHECK_INTERVAL,
        )

    def _schedule_election_timeout(self) -> None:
        self.provider.schedule_timer(
            "election_timeout",
            self.provider.current_time() + ELECTION_WAIT,
        )

    # ── Timer handler ──────────────────────────────────────────────────

    def handle_timer(self, timer: str) -> None:
        if timer == "election_check":
            self._check_for_split_or_merge()
            self._schedule_election_check()
        elif timer == "election_timeout":
            self._on_election_timeout()
        else:
            super().handle_timer(timer)

    # ── Packet handler (intercepts before base class) ──────────────────

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")

        # ── Election protocol messages ─────────────────────────────────
        if msg_type == "election":
            self._on_receive_election(raw)
            return
        elif msg_type == "alive":
            self._on_receive_alive(raw)
            return
        elif msg_type == "leader":
            self._on_receive_leader(raw)
            return
        elif msg_type == "data_transfer":
            self._on_receive_data_transfer(raw)
            return

        # ── Sensor data during election → buffer it ────────────────────
        sender_type = raw.get("sender_type")
        if sender_type == "sensor" and self._election_in_progress:
            incoming: Dict[str, int] = raw.get("packets", {})
            if incoming:
                for sensor, count in incoming.items():
                    self._election_buffer[sensor] = (
                        self._election_buffer.get(sensor, 0) + count
                    )
                logging.info(
                    f"UAV {self.provider.get_id()} buffered {incoming} during "
                    f"election | election_buffer: {self._election_buffer}"
                )
            return

        # ── UAV heartbeat → track peer for split/merge detection ───────
        if sender_type == "uav":
            peer_id = raw.get("sender_id")
            if peer_id is not None:
                now = self.provider.current_time()
                is_new_peer = peer_id not in self._known_peers
                self._known_peers.add(peer_id)
                self._peer_last_seen[peer_id] = now

                # First exchange: predefine leader as highest known ID
                if not self._leader_predefined and not self._election_in_progress:
                    self._try_predefine_leader()

                # Merge detection: new peer appeared
                if is_new_peer and self._leader_predefined:
                    logging.info(
                        f"UAV {self.provider.get_id()} detected MERGE — "
                        f"new peer {peer_id} appeared"
                    )
                    self._start_election("merge")

        # ── Default: delegate to base class ────────────────────────────
        super().handle_packet(message)

    # ── Pre-defined leader (highest ID seen so far) ────────────────────

    def _try_predefine_leader(self) -> None:
        """Set the initial leader to the highest known ID (including self)."""
        all_ids = self._known_peers | {self.provider.get_id()}
        leader = self.reconciliation_strategy.choose_leader(
            self._build_election_context(), all_ids,
        )
        self._current_leader_id = leader
        self._is_leader = (leader == self.provider.get_id())
        self._leader_predefined = True
        logging.info(
            f"UAV {self.provider.get_id()} predefined leader → {leader}"
            f"{' (self)' if self._is_leader else ''}"
        )

    # ── Split / merge detection (delegates to anomaly strategy) ────────

    def _check_for_split_or_merge(self) -> None:
        """
        Periodic check: ask the anomaly-detection strategy whether any
        peers have been lost or discovered, then act on the result.
        """
        ctx = self._build_election_context()
        result = self.anomaly_strategy.check_for_anomaly(ctx)

        if not result.lost_peers:
            return

        # Remove lost peers from internal bookkeeping
        for pid in result.lost_peers:
            self._known_peers.discard(pid)
            self._peer_last_seen.pop(pid, None)
            logging.info(
                f"UAV {self.provider.get_id()} lost contact with peer {pid}"
            )

        # Ask the invitation strategy whether to start an election
        # (rebuild context after peer removal)
        ctx = self._build_election_context()
        if self.invitation_strategy.should_start_election(ctx, result):
            logging.info(
                f"UAV {self.provider.get_id()} lost leader "
                f"{result.trigger} — triggering election"
            )
            self._start_election(result.trigger or "split")

    # ── Election logic (Bully algorithm) ───────────────────────────────

    def _start_election(self, trigger: str) -> None:
        """Begin a new election round."""
        if self._election_in_progress:
            return  # already electing

        self._election_in_progress = True
        self._received_alive = False
        self._election_start_time = self.provider.current_time()
        self._is_leader = False
        self._current_leader_id = None

        # Create metrics event
        self._election_id_counter += 1
        self._current_election_event = ElectionEvent(
            election_id=self._election_id_counter,
            trigger=trigger,
            start_time=self._election_start_time,
            participants=[self.provider.get_id()],
        )

        logging.info(
            f"UAV {self.provider.get_id()} starting election "
            f"(trigger={trigger}, election_id={self._election_id_counter})"
        )

        # Ask the invitation strategy to build the election message
        ctx = self._build_election_context()
        msg = self.invitation_strategy.build_election_message(ctx)
        self._broadcast_dict(msg)

        # Set timeout: if no "alive" received, we win
        self._schedule_election_timeout()

    def _on_receive_election(self, raw: dict) -> None:
        """Handle an incoming election message from a peer."""
        sender_id = raw["sender_id"]
        my_id = self.provider.get_id()

        logging.debug(
            f"UAV {my_id} received election from {sender_id}"
        )

        # Use the reconciliation strategy to decide who should prevail
        ctx = self._build_election_context()
        preferred = self.reconciliation_strategy.choose_leader(
            ctx, {my_id, sender_id},
        )

        if preferred == my_id:
            # We are preferred over the sender → send alive and start our
            # own election
            alive_msg = {
                "msg_type": "alive",
                "sender_type": "uav",
                "sender_id": my_id,
            }
            self._broadcast_dict(alive_msg)
            # Start our own election if not already running
            if not self._election_in_progress:
                self._start_election(
                    self._current_election_event.trigger
                    if self._current_election_event
                    else "merge"
                )
        # If sender is preferred, just wait — they'll become leader or
        # someone even more preferred will.

    def _on_receive_alive(self, raw: dict) -> None:
        """A higher-priority peer is alive — we won't be leader."""
        sender_id = raw["sender_id"]
        logging.debug(
            f"UAV {self.provider.get_id()} received alive from {sender_id} "
            f"— standing down"
        )
        self._received_alive = True

    def _on_election_timeout(self) -> None:
        """
        Election timer expired.  If no "alive" was received, declare
        ourselves leader and broadcast the result.
        """
        if not self._election_in_progress:
            return

        if self._received_alive:
            # A higher-priority node is out there; wait for their "leader"
            # message.  Reset and extend the timeout in case of delays.
            logging.debug(
                f"UAV {self.provider.get_id()} received alive — "
                f"extending election wait"
            )
            self._received_alive = False
            self._schedule_election_timeout()
            return

        # No alive received → we are the leader
        my_id = self.provider.get_id()
        self._declare_leader(my_id)

        # Broadcast leader announcement
        leader_msg = {
            "msg_type": "leader",
            "sender_type": "uav",
            "sender_id": my_id,
            "election_start_time": self._election_start_time,
        }
        self._broadcast_dict(leader_msg)

    def _on_receive_leader(self, raw: dict) -> None:
        """Accept the declared leader and transfer buffered data."""
        leader_id = raw["sender_id"]
        logging.info(
            f"UAV {self.provider.get_id()} accepts leader → {leader_id}"
        )
        self._declare_leader(leader_id)

    def _declare_leader(self, leader_id: int) -> None:
        """Finalise the election: record metrics and transfer data."""
        my_id = self.provider.get_id()
        now = self.provider.current_time()

        self._current_leader_id = leader_id
        self._is_leader = (leader_id == my_id)
        self._election_in_progress = False

        # Record election metrics
        if self._current_election_event is not None:
            evt = self._current_election_event
            evt.end_time = now
            evt.duration = now - evt.start_time
            evt.winner_id = leader_id

            # Record buffer size for this UAV
            buf_total = sum(self._election_buffer.values())
            evt.buffer_sizes[my_id] = buf_total

            if my_id not in evt.participants:
                evt.participants.append(my_id)

            ELECTION_LOG.append(evt)
            self._current_election_event = None

            logging.info(
                f"UAV {my_id} election resolved: leader={leader_id}, "
                f"duration={evt.duration:.2f}s, "
                f"election_buffer_size={buf_total}"
            )

        # Merge election buffer into main packets
        for sensor, count in self._election_buffer.items():
            self.packets[sensor] = self.packets.get(sensor, 0) + count
        self._election_buffer.clear()

        # Non-leaders: transfer data to the leader based on collection mode
        if not self._is_leader and leader_id is not None:
            collection = self.reconciliation_strategy.get_collection_mode()
            if collection == CollectionMode.FROM_ALL_MEMBERS:
                # Every non-leader sends its data to the leader
                self._transfer_data_to_leader(leader_id)
            elif collection == CollectionMode.FROM_SUB_LEADERS:
                # Only former sub-swarm leaders send data
                # (For now, in a simple topology every node was its own
                # "sub-leader" after a split, so this behaves identically.
                # Custom strategies can refine this with sub-swarm tracking.)
                self._transfer_data_to_leader(leader_id)

    def _transfer_data_to_leader(self, leader_id: int) -> None:
        """Send all collected packets to the leader, then clear local buffer."""
        if not self.packets:
            return

        transfer_msg = {
            "msg_type": "data_transfer",
            "sender_type": "uav",
            "sender_id": self.provider.get_id(),
            "packets": dict(self.packets),
        }
        self._broadcast_dict(transfer_msg)
        logging.info(
            f"UAV {self.provider.get_id()} transferred {sum(self.packets.values())} "
            f"packets to leader {leader_id} | breakdown: {self.packets}"
        )
        self.packets.clear()

    def _on_receive_data_transfer(self, raw: dict) -> None:
        """Leader receives data from a non-leader after election."""
        if not self._is_leader:
            return  # only the leader accepts transfers

        sender_id = raw.get("sender_id")
        incoming: Dict[str, int] = raw.get("packets", {})
        if incoming:
            for sensor, count in incoming.items():
                self.packets[sensor] = self.packets.get(sensor, 0) + count
            logging.info(
                f"UAV {self.provider.get_id()} (leader) received transfer "
                f"from {sender_id}: {incoming} | buffer: {self.packets}"
            )

    # ── Finish — log final election stats for this UAV ─────────────────

    def finish(self) -> None:
        my_id = self.provider.get_id()
        logging.info(
            f"UAV {my_id} (slot {self._cluster_index}) election finish | "
            f"is_leader={self._is_leader}, "
            f"leader_id={self._current_leader_id}, "
            f"elections_participated={self._election_id_counter}, "
            f"remaining_election_buffer={self._election_buffer}"
        )
        super().finish()
