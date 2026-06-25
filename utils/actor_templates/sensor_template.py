import json
import logging


from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import SendMessageCommand

from gradysim.protocol.messages.telemetry import Telemetry

from utils.common.config import (
    SENSOR_PACKET_INTERVAL,
    PACKET_TTL_TICKS,
)

from utils.common.agent_types import AgentType
from utils.common.message_schemas import SimpleMessage
from utils.common.lamport import LamportClock
from utils.metrics.stats import SENSOR_STATS

class SensorProtocol(IProtocol):
    """
    Generates one packet every SENSOR_PACKET_INTERVAL seconds and transfers
    all buffered packets to the first UAV that broadcasts a heartbeat within
    radio range.

    Packet TTL
    ----------
    The sensor maintains a monotonic Lamport counter (local only — it does not
    exchange clocks with UAVs; its counter is purely an event count).
    Each batch of packets is stamped with the Lamport tick at which the oldest
    packet in the batch was generated (_batch_start_lamport).

    On every "gen" timer fire the sensor first checks whether the current batch
    has aged past PACKET_TTL_TICKS ticks without being collected.  If so it
    expires the whole buffer: the count moves to total_expired and the buffer
    resets.  Then it generates the new packet and starts a fresh batch window.

    When a UAV collects:
    • The full packet_count is sent.
    • The buffer resets; _batch_start_lamport is reset to the current tick.

    Delivery-ratio accounting
    -------------------------
    total_produced — incremented on every "gen" event (including those that
                     will later be expired, so this is the ground-truth total).
    total_expired  — incremented when a batch is flushed due to TTL.
    At finish() both values are written to SENSOR_STATS for the base station
    to consume.
    """

    packet_count:        int
    total_produced:      int
    total_expired:       int
    _sensor_name:        str
    _batch_start_lamport: int

    def initialize(self) -> None:
        self._lamport = LamportClock()
        self.packet_count        = 0
        self.total_produced      = 0
        self.total_expired       = 0
        self._batch_start_lamport = 0

        # Derive a stable name from the node ID at runtime.
        # The simulator assigns IDs in add_node order; sensors are added after
        # base (id=0) and e-station (id=1), so sensor_N gets id = N+1.
        self._sensor_name = f"sensor_{self.provider.get_id() - 1}"
        self._schedule_packet()

    def _schedule_packet(self) -> None:
        self.provider.schedule_timer(
            "gen", self.provider.current_time() + SENSOR_PACKET_INTERVAL
        )

    def _check_ttl_expiry(self) -> None:
        """
        If the current packet batch is older than PACKET_TTL_TICKS Lamport
        ticks and there are buffered packets, expire them now.
        """
        if self.packet_count > 0:
            age = self._lamport.time - self._batch_start_lamport
            if age >= PACKET_TTL_TICKS:
                logging.info(
                    f"Sensor {self._sensor_name}: TTL expired — "
                    f"discarding {self.packet_count} packets "
                    f"(age={age} ticks, TTL={PACKET_TTL_TICKS})"
                )
                self.total_expired       += self.packet_count
                self.packet_count         = 0
                self._batch_start_lamport = self._lamport.time

    def handle_timer(self, timer: str) -> None:
        if timer == "gen":
            # Advance the local Lamport clock (internal event).
            self._lamport.tick()

            # Check TTL before adding new packet.
            self._check_ttl_expiry()

            # If buffer was just reset, this new packet starts a fresh batch.
            if self.packet_count == 0:
                self._batch_start_lamport = self._lamport.time

            self.packet_count   += 1
            self.total_produced += 1

            self._schedule_packet()

    def handle_packet(self, message: str) -> None:
        try:
            msg: SimpleMessage = json.loads(message)
        except json.JSONDecodeError:
            return

        if msg.get("sender_type") == AgentType.UAV.value and self.packet_count > 0:
            response: SimpleMessage = {
                "packets":     {self._sensor_name: self.packet_count},
                "sender_type": AgentType.SENSOR.value,
                "sender_id":   self.provider.get_id(),
            }
            self.provider.send_communication_command(
                SendMessageCommand(json.dumps(response), msg["sender_id"])
            )
            logging.info(
                f"Sensor {self._sensor_name}: delivered {self.packet_count} packets "
                f"to UAV {msg['sender_id']} "
                f"(produced={self.total_produced}, expired={self.total_expired})"
            )
            self.packet_count         = 0
            self._batch_start_lamport = self._lamport.time

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        # Anything still in the buffer at simulation end is undelivered.
        undelivered = self.packet_count
        SENSOR_STATS[self._sensor_name] = {
            "produced":    self.total_produced,
            "expired":     self.total_expired,
            "undelivered": undelivered,
        }
        logging.info(
            f"Sensor {self.provider.get_id()} ({self._sensor_name}) shutdown | "
            f"produced={self.total_produced} | "
            f"expired={self.total_expired} | "
            f"undelivered={undelivered}"
        )