from utils.common.sensor_positions import (
    ForbiddenZone,
    generate_sensor_positions,
    compute_cluster_waypoints,
    compute_split_waypoints,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Overall simulation configs
SIMULATION_DURATION   = 120

# UAV configs
NUM_UAVS = 5
UAV_SPEED             = 3.0
FLIGHT_ALT      = 15.0
PASS_ALT_OFFSET = 5.0

# Communication configs
COMM_RANGE            = 20.0
ANNOUNCE_WINDOW       = 3.0

# Sensor configs
SENSOR_SEED = 2056
SENSOR_FIELD_X = (-100, 100)
SENSOR_FIELD_Y = (-100, 100)
SENSOR_PACKET_INTERVAL = 4.0
SENSOR_MIN_SPACING  = COMM_RANGE
# must be <= SENSOR_MIN_SPACING / 2 to stay valid
SENSOR_JITTER       = (SENSOR_MIN_SPACING  / 2) - 1


# ── Lamport clock / packet TTL ──────────────────────────────────────────────
# Each sensor "gen" tick advances the sensor's own Lamport counter by 1.
# Packets undelivered for more than PACKET_TTL_TICKS are expired and counted.
# TODO Check if still needed
PACKET_TTL_TICKS = 100

# ── Positions ──────────────────────────────────────────────────────────────
BASE_GROUND  = (sum(SENSOR_FIELD_X)/2, SENSOR_FIELD_Y[0], 0)
BASE_HOVER   = (BASE_GROUND[0], BASE_GROUND[1], FLIGHT_ALT)

ENDPOINT_GROUND = (sum(SENSOR_FIELD_X)/2, SENSOR_FIELD_Y[1], 0)
ENDPOINT_HOVER = (ENDPOINT_GROUND[0], ENDPOINT_GROUND[1], FLIGHT_ALT)

OBSTACLE_CENTER   = (sum(SENSOR_FIELD_X)/2, sum(SENSOR_FIELD_Y)/2)
OBSTACLE_HALF_W   = (COMM_RANGE + 50) / 2   
OBSTACLE_HALF_H   = (COMM_RANGE + 50) / 2

OBSTACLE_ZONE = ForbiddenZone(
    x_min = OBSTACLE_CENTER[0] - OBSTACLE_HALF_W,
    x_max = OBSTACLE_CENTER[0] + OBSTACLE_HALF_W,
    y_min = OBSTACLE_CENTER[1] - OBSTACLE_HALF_H,
    y_max = OBSTACLE_CENTER[1] + OBSTACLE_HALF_H,
)

SENSOR_POSITIONS = generate_sensor_positions(
    seed             = SENSOR_SEED,
    x_range          = SENSOR_FIELD_X,
    y_range          = SENSOR_FIELD_Y,
    min_spacing      = SENSOR_MIN_SPACING,
    forbidden_zones  = [OBSTACLE_ZONE],
    forbidden_margin = 5.0,
    jitter           = SENSOR_JITTER,
)

SENSORS_HOVER = {
    name: (pos[0], pos[1], FLIGHT_ALT)
    for name, pos in SENSOR_POSITIONS.items()
}

SENSOR_WAYPOINT_NAMES = list(SENSORS_HOVER.keys())

ALL_WAYPOINTS = {
    "base_ground":     BASE_GROUND,
    "base_hover":      BASE_HOVER,
    "endpoint_ground": ENDPOINT_GROUND,
    "endpoint_hover":  ENDPOINT_HOVER,
    **SENSORS_HOVER,
}

# Straight-line centroid path the UAV cluster follows, spaced by COMM_RANGE.
# Index 0 = base (start); last index = endpoint (far end of field).
CLUSTER_WAYPOINTS = compute_cluster_waypoints(
    start      = (BASE_GROUND[0], BASE_GROUND[1]),
    end        = (ENDPOINT_GROUND[0], ENDPOINT_GROUND[1]),
    comm_range = COMM_RANGE,
)

# ── Split paths around the obstacle ────────────────────────────────────────
# Two waypoint sequences that diverge around the forbidden zone and converge
# afterward.  Used to split the cluster into two sub-groups for testing
# leader-election algorithms during the merge phase.
OBSTACLE_MARGIN = 10.0

LEFT_WAYPOINTS, RIGHT_WAYPOINTS = compute_split_waypoints(
    start      = (BASE_GROUND[0], BASE_GROUND[1]),
    end        = (ENDPOINT_GROUND[0], ENDPOINT_GROUND[1]),
    obstacle   = OBSTACLE_ZONE,
    comm_range = COMM_RANGE,
    margin     = OBSTACLE_MARGIN,
)
