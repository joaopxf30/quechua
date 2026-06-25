# ---------------------------------------------------------------------------
# Shared sensor statistics registry
# ---------------------------------------------------------------------------
# Written by each SensorProtocol.finish(); read by BaseStationProtocol.finish().
# Both finish() calls occur at simulation end (order: sensors before base is
# not guaranteed, but gradysim calls finish in node-ID order, sensors < base
# in our setup, so this is reliable).

from typing import Dict

SENSOR_STATS: Dict[str, Dict[str, int]] = {}
# sensor_name → {"produced": N, "expired": M, "undelivered": K}


# TODO Not used for now but might be good to move to end point
# BASE_STATS: Dict[str, object] = {}
# Populated by BaseStationProtocol.finish(); read by dadca.py post-simulation.
# Keys: "received" (Dict[str, int]), "lamport" (int)