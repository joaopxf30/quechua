"""
Path-decision helpers for UAV patrol.

Movement is driven entirely by GotoCoordsMobilityCommand.

Waypoint model
--------------
UAVs follow a linear list of (x, y, z) waypoints (passed via the factory
closure), ordered base → field.  Index 0 is the base; the last index is the
far end of the field.  The mixin tracks `_wp_index` and `_going_to_base` and
advances/reverses the index on each arrival.

Flight formation
----------------
UAVs fly in a lateral formation perpendicular to the direction of travel so
that every UAV in the cluster stays within COMM_RANGE of its neighbours.

Each UAV receives a `cluster_index` (0 … num_uavs-1) and `num_uavs` from the
factory closure.  When issuing a GotoCoordsMobilityCommand the mixin computes:

  spacing = COMM_RANGE / num_uavs          # guarantees all pairs < COMM_RANGE
  centre  = (cluster_index - (num_uavs-1) / 2) * spacing   # symmetric around 0
  offset  = centre * perpendicular_unit_vector

where the perpendicular is the 90-degree CCW rotation of the unit vector
pointing from the current waypoint toward the next one in the direction of
travel.

If the cluster has only one UAV (`num_uavs == 1`) no offset is applied.
"""
import logging
from math import sqrt
from typing import List, Tuple

from gradysim.protocol.messages.mobility import (
    GotoCoordsMobilityCommand,
    SetSpeedMobilityCommand,
)

from utils.common.config import (
    UAV_SPEED,
    FLIGHT_ALT,
    COMM_RANGE,
)


class PathDecisionMixin:
    """
    Mixin that provides waypoint traversal and lateral formation offset for a
    UAV protocol class.

    Expected attributes on `self` (set by the host protocol's initialize):
        _waypoints:      List[Tuple[float, float, float]]  — ordered base→field
        _wp_index:       int                               — current waypoint index
        _going_to_base:  bool
        _cluster_index:  int                               — this UAV's slot (0-based)
        _num_uavs:       int                               — total UAVs in cluster
        packets:         Dict[str, int]                    — sensor→count
        state:           str
    """

    # ── Geometry helpers ──────────────────────────────────────────────────

    @staticmethod
    def _get_distance(
        a: Tuple[float, float, float],
        b: Tuple[float, float, float],
    ) -> float:
        return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _formation_offset(self, wp_index: int) -> Tuple[float, float]:
        """
        Compute the (dx, dy) formation offset for this UAV.

        The offset is perpendicular to the travel direction at `wp_index`.
        Returns (0, 0) when there is only one UAV in the cluster.
        """
        if self._num_uavs <= 1:
            return 0.0, 0.0

        waypoints = self._waypoints

        # Direction vector: from current wp toward the next in travel direction
        if self._going_to_base:
            ref_idx = max(0, wp_index - 1)
        else:
            ref_idx = min(len(waypoints) - 1, wp_index + 1)

        ax, ay, _ = waypoints[wp_index]
        bx, by, _ = waypoints[ref_idx]
        dx, dy    = bx - ax, by - ay
        length    = sqrt(dx*dx + dy*dy) or 1.0

        # Normalised direction
        ux, uy = dx / length, dy / length
        # 90° CCW perpendicular: (-uy, ux)
        px, py = -uy, ux

        # Symmetric spacing so every pair of adjacent UAVs is COMM_RANGE / num_uavs apart
        spacing = (COMM_RANGE // 2) / self._num_uavs
        centre  = (self._cluster_index - (self._num_uavs - 1) / 2) * spacing

        return px * centre, py * centre

    # ── Goto helpers ──────────────────────────────────────────────────────

    def _goto_waypoint(self, index: int) -> None:
        """
        Issue a GotoCoordsMobilityCommand to waypoint[index],
        applying the lateral formation offset for this UAV.
        """
        x, y, _ = self._waypoints[index]
        ox, oy  = self._formation_offset(index)
        self.provider.send_mobility_command(
            GotoCoordsMobilityCommand(x + ox, y + oy, FLIGHT_ALT)
        )

    # ── Patrol start ──────────────────────────────────────────────────────

    def _start_patrol(self) -> None:
        self.provider.send_mobility_command(SetSpeedMobilityCommand(UAV_SPEED))
        self._goto_waypoint(self._wp_index)
        logging.debug(
            f"UAV {self.provider.get_id()} patrol → wp[{self._wp_index}] "
            f"slot={self._cluster_index}/{self._num_uavs} "
            f"({'inbound' if self._going_to_base else 'outbound'})"
        )

    # ── Waypoint advancement ──────────────────────────────────────────────

    def _advance_waypoint(self) -> None:
        """Step to the next waypoint in the current direction of travel."""
        waypoints = self._waypoints

        if self._going_to_base:
            if self._wp_index == 0:
                # Arrived at base — deliver packets, then turn outbound
                if self.packets:
                    self._deliver_to_base()
                self._going_to_base = False
                self._wp_index = min(1, len(waypoints) - 1)
            else:
                self._wp_index -= 1
        else:
            if self._wp_index < len(waypoints) - 1:
                self._wp_index += 1
            else:
                # Reached the far end — reverse toward base
                self._going_to_base = True
                self._wp_index = max(0, self._wp_index - 1)

        self._goto_waypoint(self._wp_index)
        logging.debug(
            f"UAV {self.provider.get_id()} → wp[{self._wp_index}] "
            f"({'inbound' if self._going_to_base else 'outbound'})"
        )

    # ── Delivery ──────────────────────────────────────────────────────────

    def _deliver_to_base(self) -> None:
        """
        Broadcast collected sensor packets to any base station in range.
        Override or extend in the host protocol to customise delivery behaviour.
        """
        import json
        from gradysim.protocol.messages.communication import BroadcastMessageCommand

        total = sum(self.packets.values())
        msg   = {
            "packets":     dict(self.packets),
            "sender_type": "uav",
            "sender_id":   self.provider.get_id(),
        }
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(msg))
        )
        logging.info(
            f"UAV {self.provider.get_id()} delivering {total} packets "
            f"to base | breakdown: {self.packets}"
        )
        self.packets = {}
