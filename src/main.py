"""
Simulation entry point.

Run with:
    python -m src.main
    python -m src.main --no-viz         # disable browser visualisation
    python -m src.main --force-regen  # regenerate sensor/waypoint layout
"""
import logging
import argparse
from collections import defaultdict

from gradysim.simulator.handler.communication import (
    CommunicationHandler,
    CommunicationMedium,
)
from gradysim.simulator.handler.mobility import MobilityHandler
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import (
    VisualizationHandler,
    VisualizationConfiguration,
)
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from utils.common.config import (
    COMM_RANGE,
    SIMULATION_DURATION,
    BASE_GROUND,
    ENDPOINT_GROUND,
    FLIGHT_ALT,
    SENSOR_POSITIONS,
    NUM_UAVS,
    CLUSTER_WAYPOINTS,
    LEFT_WAYPOINTS,
    RIGHT_WAYPOINTS,
)

from src.sensor import Sensor
from src.uav import make_uav_viz, set_total_uavs

from utils.metrics.election_stats import ELECTION_LOG


def main() -> None:
    parser = argparse.ArgumentParser(description="GrADySim UAV cluster simulation")
    parser.add_argument(
        "--no-viz",
        action="store_true",
        default=False,
        help="disable the browser visualisation",
    )
    parser.add_argument(
        "--force-regen",
        action="store_true",
        default=False,
        help="Force regeneration of sensor positions and cluster waypoints",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        default=False,
        help="UAVs loop (bounce) along their path instead of stopping at the endpoint",
    )
    args = parser.parse_args()

    # ── Simulation config ──────────────────────────────────────────────────
    config  = SimulationConfiguration(duration=SIMULATION_DURATION)
    builder = SimulationBuilder(config)

    logging.info(f"Base:              ({BASE_GROUND[0]:.1f}, {BASE_GROUND[1]:.1f})")
    logging.info(f"Endpoint:          ({ENDPOINT_GROUND[0]:.1f}, {ENDPOINT_GROUND[1]:.1f})")
    logging.info(f"Sensors placed:    {len(SENSOR_POSITIONS)}")
    logging.info(f"Cluster waypoints: {len(CLUSTER_WAYPOINTS)}")
    logging.info(f"UAVs:              {NUM_UAVS}")

    # ── Scene bounds for visualisation ────────────────────────────────────
    if not args.no_viz:
        padding = 50
        all_x   = [p[0] for p in SENSOR_POSITIONS.values()] + [BASE_GROUND[0], ENDPOINT_GROUND[0]]
        all_y   = [p[1] for p in SENSOR_POSITIONS.values()] + [BASE_GROUND[1], ENDPOINT_GROUND[1]]
        viz_config = VisualizationConfiguration(
            x_range      = (min(all_x) - padding, max(all_x) + padding),
            y_range      = (min(all_y) - padding, max(all_y) + padding),
            z_range      = (0, FLIGHT_ALT + 20),
            open_browser = True,
        )

    # ── Sensors (static nodes) ─────────────────────────────────────────────
    for sensor_pos in SENSOR_POSITIONS.values():
        builder.add_node(Sensor, sensor_pos)

    # ── UAV cluster (split around obstacle) ────────────────────────────────
    # Half the UAVs take the left path around the obstacle and the other
    # half take the right path.  Both groups merge after the obstacle,
    # creating a natural scenario for leader-election testing.
    n_left  = NUM_UAVS // 2
    n_right = NUM_UAVS - n_left

    logging.info(f"Left group:  {n_left} UAVs  ({len(LEFT_WAYPOINTS)} waypoints)")
    logging.info(f"Right group: {n_right} UAVs ({len(RIGHT_WAYPOINTS)} waypoints)")

    for i in range(n_left):
        UAVClass = make_uav_viz(
            cluster_index=i,
            waypoints=LEFT_WAYPOINTS,
            group_size=n_left,
            loop=args.loop,
        )
        builder.add_node(UAVClass, BASE_GROUND)

    for i in range(n_right):
        UAVClass = make_uav_viz(
            cluster_index=i,
            waypoints=RIGHT_WAYPOINTS,
            group_size=n_right,
            loop=args.loop,
        )
        builder.add_node(UAVClass, BASE_GROUND)

    # ── Handlers ───────────────────────────────────────────────────────────
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler())
    builder.add_handler(
        CommunicationHandler(CommunicationMedium(transmission_range=COMM_RANGE))
    )
    if not args.no_viz:
        builder.add_handler(VisualizationHandler(viz_config))

    # ── Run ────────────────────────────────────────────────────────────────
    if not args.loop:
        set_total_uavs(NUM_UAVS)

    simulation = builder.build()

    handlers = logging.root.handlers[:]
    for handler in handlers[-1:]:  # mantém só o primeiro
        logging.root.removeHandler(handler)

    simulation.start_simulation()

    # ── Election statistics summary ────────────────────────────────────────
    _print_election_summary()

    logging.info("Simulation complete.")


def _print_election_summary() -> None:
    """Print a formatted summary of all election events recorded during
    the simulation."""
    if not ELECTION_LOG:
        print("\n" + "=" * 60)
        print("  ELECTION SUMMARY: No elections were triggered.")
        print("=" * 60 + "\n")
        return

    grouped_rounds = _group_events_by_round(ELECTION_LOG)

    print("\n" + "=" * 60)
    print("  ELECTION SUMMARY")
    print("=" * 60)
    print(f"  Local election records: {len(ELECTION_LOG)}")
    print(f"  Grouped election rounds: {len(grouped_rounds)}")

    durations = [r["duration"] for r in grouped_rounds if r["duration"] > 0]
    if durations:
        print(f"  Average election duration: {sum(durations)/len(durations):.2f}s")
        print(f"  Max election duration:     {max(durations):.2f}s")
        print(f"  Min election duration:     {min(durations):.2f}s")

    merge_rounds = [r for r in grouped_rounds if r["trigger"] == "merge"]
    split_rounds = [r for r in grouped_rounds if r["trigger"] == "split"]
    print(f"  Merge-triggered rounds: {len(merge_rounds)}")
    print(f"  Split-triggered rounds: {len(split_rounds)}")
    print("-" * 60)

    for i, round_data in enumerate(grouped_rounds, start=1):
        buffer_sizes = round_data["buffer_sizes"]
        print(
            f"  Election Round #{i} | trigger={round_data['trigger']} | "
            f"winner=UAV {round_data['winner_id']} | "
            f"duration={round_data['duration']:.2f}s "
            f"({round_data['start_time']:.1f}s → {round_data['end_time']:.1f}s)"
        )
        if buffer_sizes:
            print(f"    Buffer sizes at transfer:")
            for uav_id, size in sorted(buffer_sizes.items()):
                print(f"      UAV {uav_id}: {size} packets buffered")
        print()

    # ── Per-merge time cost (the main metric) ──────────────────────────
    if merge_rounds:
        merge_durations = [r["duration"] for r in merge_rounds]
        total_merge_time = sum(merge_durations)
        print("-" * 60)
        print(f"  TIME LOST TO MERGES: {total_merge_time:.2f}s total")
        print(f"  Average merge election: {total_merge_time/len(merge_durations):.2f}s")

        total_buffered = sum(
            sum(r["buffer_sizes"].values()) for r in merge_rounds
        )
        print(f"  Total packets buffered during merges: {total_buffered}")

    if split_rounds:
        split_durations = [r["duration"] for r in split_rounds]
        total_split_time = sum(split_durations)
        print("-" * 60)
        print(f"  TIME LOST TO SPLITS: {total_split_time:.2f}s total")
        print(f"  Average split election: {total_split_time/len(split_durations):.2f}s")

        total_split_buffered = sum(
            sum(r["buffer_sizes"].values()) for r in split_rounds
        )
        print(f"  Total packets buffered during splits: {total_split_buffered}")

    print("=" * 60 + "\n")


def _group_events_by_round(events):
    """Group per-UAV local election records into consolidated rounds.

    A round is identified by trigger + winner + end-time bucket (0.1s),
    which aligns local records that refer to the same swarm election.
    """
    grouped = {}

    for evt in events:
        key = (evt.trigger, evt.winner_id, round(evt.end_time, 1))
        if key not in grouped:
            grouped[key] = {
                "trigger": evt.trigger,
                "winner_id": evt.winner_id,
                "start_time": evt.start_time,
                "end_time": evt.end_time,
                "duration": evt.duration,
                "buffer_sizes": defaultdict(int),
            }

        g = grouped[key]
        g["start_time"] = min(g["start_time"], evt.start_time)
        g["end_time"] = max(g["end_time"], evt.end_time)
        g["duration"] = max(g["duration"], evt.duration)

        for uav_id, size in evt.buffer_sizes.items():
            g["buffer_sizes"][uav_id] += size

    rounds = []
    for g in grouped.values():
        rounds.append(
            {
                "trigger": g["trigger"],
                "winner_id": g["winner_id"],
                "start_time": g["start_time"],
                "end_time": g["end_time"],
                "duration": g["duration"],
                "buffer_sizes": dict(g["buffer_sizes"]),
            }
        )

    rounds.sort(key=lambda r: (r["start_time"], r["end_time"], r["winner_id"]))
    return rounds


if __name__ == "__main__":
    main()