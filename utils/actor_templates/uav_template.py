"""
UAV Protocol template — barebones sensor-collection + formation flight.

This module provides `make_uav_protocol`, a factory that returns a minimal
GrADySim-compatible IProtocol subclass.

What's included
---------------
- Formation flight: each UAV flies at a lateral offset from the shared
  cluster waypoints so that all UAVs stay within COMM_RANGE of each other
  (handled by PathDecisionMixin._goto_waypoint).
- Sensor packet collection: sensor broadcasts are absorbed into self.packets.
- Intra-cluster packet gossip: a periodic heartbeat broadcasts each UAV's
  collected sensor data to its cluster mates, so the cluster collectively
  accumulates all sensor readings regardless of which UAV passed closest.
- Lamport logical clock for causal ordering of heartbeats.
- Base delivery: when the UAV returns to waypoint[0] it broadcasts its
  collected packets to any base station in range, then turns outbound again.
- Configurable departure delay for staggered launch.

What's intentionally NOT here (add via subclass or mixin)
----------------------------------------------------------
- Battery drain / low-battery diversion.
- UAV-to-UAV packet *transfer* / swap (the cluster gossips, not transfers).
- Cluster split / merge for obstacle avoidance.

Extension pattern
-----------------
Students should NOT edit this file. Instead they can:

  1. Subclass the returned class:

        Base = make_uav_protocol(CLUSTER_WAYPOINTS, cluster_index=0, num_uavs=5)

        class MyUAV(Base):
            def handle_telemetry(self, telemetry):
                super().handle_telemetry(telemetry)
                # add battery drain, obstacle logic, etc.

  2. Prepend an additional mixin:

        class SwapMixin:
            def handle_packet(self, message):
                super().handle_packet(message)
                # add peer-to-peer swap logic here

        class MyUAV(SwapMixin, make_uav_protocol(CLUSTER_WAYPOINTS, ...)):
            pass

  3. Use make_uav_protocol_viz (in uav_viz_template.py) which wraps this
     factory and adds GrADySim visualisation transparently.
"""
import json
import logging
from typing import Dict, List, Tuple

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import BroadcastMessageCommand
from gradysim.protocol.messages.telemetry import Telemetry

from utils.actor_templates.path_decision_mixin import PathDecisionMixin
from utils.common.config import FLIGHT_ALT

# ── Arrival threshold (flat XY distance) ──────────────────────────────────

ARRIVAL_RADIUS = 2.0

# ── Heartbeat interval (seconds) ──────────────────────────────────────────

HEARTBEAT_INTERVAL = 1.0


# ── Lamport clock (minimal, self-contained) ───────────────────────────────

class _LamportClock:
    """Monotonically increasing logical clock."""

    def __init__(self) -> None:
        self.time: int = 0

    def tick(self) -> int:
        """Increment before sending."""
        self.time += 1
        return self.time

    def update(self, received: int) -> None:
        """Receive rule: time = max(local, received) + 1."""
        self.time = max(self.time, received) + 1


# ── Factory ────────────────────────────────────────────────────────────────

def make_uav_protocol(
    waypoints: List[Tuple[float, float, float]],
    cluster_index: int = 0,
    num_uavs: int = 1,
    departure_delay: float = 0.0,
) -> type:
    """
    Return a GrADySim-compatible IProtocol subclass for basic UAV patrol.

    Args:
        waypoints:      Ordered list of (x, y, z) positions, base → field.
                        Typically CLUSTER_WAYPOINTS from config.py.
                        Index 0 is the base; the last index is the far end.
        cluster_index:  This UAV's slot within the cluster (0-based).
                        Used to compute the lateral formation offset.
        num_uavs:       Total number of UAVs in the cluster.
                        Used together with cluster_index for formation spacing.
        departure_delay: Seconds to wait before starting patrol.
                        Stagger UAVs by passing i * delay for UAV i.

    Returns:
        A class (not an instance) ready for SimulationBuilder.add_node().
    """

    class _UAVProtocol(PathDecisionMixin, IProtocol):

        # ── Baked-in configuration (from factory closure) ──────────────
        _waypoints:       List[Tuple[float, float, float]] = waypoints
        _cluster_index:   int                              = cluster_index
        _num_uavs:        int                              = num_uavs
        _departure_delay: float                            = departure_delay

        # ── Initialization ─────────────────────────────────────────────

        def initialize(self) -> None:
            # Collected sensor data: sensor_name → packet count
            self.packets: Dict[str, int] = {}

            # Lamport clock for causal ordering of heartbeats
            self._lamport = _LamportClock()

            # Traversal state (read by PathDecisionMixin)
            self._wp_index:      int  = 1          # start heading to first field wp
            self._going_to_base: bool = False

            if self._departure_delay > 0:
                self.provider.schedule_timer(
                    "depart",
                    self.provider.current_time() + self._departure_delay,
                )
            else:
                self._start_patrol()

            self._schedule_heartbeat()

        # ── Heartbeat ──────────────────────────────────────────────────

        def _schedule_heartbeat(self) -> None:
            self.provider.schedule_timer(
                "heartbeat",
                self.provider.current_time() + HEARTBEAT_INTERVAL,
            )

        def _send_heartbeat(self) -> None:
            """
            Broadcast the current packet buffer to cluster mates.
            Cluster mates merge these counts so the whole cluster
            collectively accumulates all sensor readings.
            """
            self._lamport.tick()
            msg = {
                "sender_type": "uav",
                "sender_id":   self.provider.get_id(),
                "lamport":     self._lamport.time,
                "packets":     dict(self.packets),
            }
            self.provider.send_communication_command(
                BroadcastMessageCommand(json.dumps(msg))
            )

        def _merge_packets(self, incoming: Dict[str, int]) -> None:
            """
            Merge a {sensor: count} dict from a peer into self.packets.
            Uses max() per sensor to avoid double-counting the same reading.
            """
            for sensor, count in incoming.items():
                self.packets[sensor] = max(self.packets.get(sensor, 0), count)

        # ── Timers ─────────────────────────────────────────────────────

        def handle_timer(self, timer: str) -> None:
            if timer == "heartbeat":
                self._send_heartbeat()
                self._schedule_heartbeat()

            elif timer == "depart":
                logging.info(f"UAV {self.provider.get_id()} departing (slot {self._cluster_index})")
                self._start_patrol()

        # ── Incoming packets ───────────────────────────────────────────

        def handle_packet(self, message: str) -> None:
            """
            Dispatch incoming messages by sender_type:
            - "sensor": accumulate new readings (sum, since sensor counts are fresh).
            - "uav":    merge peer's buffer using max() to avoid double-counting.
            """
            try:
                raw = json.loads(message)
            except json.JSONDecodeError:
                return

            sender_type = raw.get("sender_type")

            if sender_type == "sensor":
                incoming: Dict[str, int] = raw.get("packets", {})
                if incoming:
                    for sensor, count in incoming.items():
                        self.packets[sensor] = self.packets.get(sensor, 0) + count
                    logging.info(
                        f"UAV {self.provider.get_id()} (slot {self._cluster_index}) "
                        f"collected {incoming} from sensor {raw.get('sender_id')} "
                        f"| buffer: {self.packets}"
                    )

            elif sender_type == "uav":
                # Update Lamport clock on receive if lower, then merge packets
                if "lamport" in raw:
                    if self._lamport.time < raw["lamport"]:
                        self._lamport.update(raw["lamport"])
                        # Merge peer's packet buffer into ours
                        peer_packets: Dict[str, int] = raw.get("packets", {})
                        if peer_packets:
                            self._merge_packets(peer_packets)
                            logging.debug(
                                f"UAV {self.provider.get_id()} merged from UAV {raw.get('sender_id')} "
                                f"[L={self._lamport.time}] | buffer: {self.packets}"
                            )
                    # Otherwise, it's an old/stale message — just ignore it.

        # ── Telemetry ──────────────────────────────────────────────────

        def handle_telemetry(self, telemetry: Telemetry) -> None:
            """
            Advance to the next waypoint when the UAV is within ARRIVAL_RADIUS
            of the current target (XY plane only).
            Override in a subclass to add battery drain, obstacle logic, etc.
            """
            pos    = telemetry.current_position
            target = self._waypoints[self._wp_index]
            ox, oy = self._formation_offset(self._wp_index)

            flat_dist = (
                (pos[0] - (target[0] + ox))**2 +
                (pos[1] - (target[1] + oy))**2
            ) ** 0.5

            if flat_dist < ARRIVAL_RADIUS:
                self._advance_waypoint()

        # ── Finish ─────────────────────────────────────────────────────

        def finish(self) -> None:
            logging.info(
                f"UAV {self.provider.get_id()} (slot {self._cluster_index}) finished "
                f"| buffer={self.packets}"
            )

    _UAVProtocol.__name__ = f"UAVProtocol_slot{cluster_index}"
    return _UAVProtocol
