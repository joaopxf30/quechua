from math import hypot, sqrt
from random import Random
from dataclasses import dataclass

from utils.common.cache import load as cache_load, save as cache_save


@dataclass
class ForbiddenZone:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.x_min - margin <= x <= self.x_max + margin and
                self.y_min - margin <= y <= self.y_max + margin)


def generate_sensor_positions(
    seed: int,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    min_spacing: float,
    forbidden_zones: list[ForbiddenZone] | None = None,
    forbidden_margin: float = 0.0,
    jitter: float = 0.0,
    force_regen: bool = False,
) -> dict[str, tuple[float, float, float]]:
    """
    Generates sensor positions on a hexagonal grid, adding optional random jitter.
    This guarantees minimum spacing while filling the area efficiently.
    Uses caching to avoid regenerating identical layouts on subsequent runs.
    """

    forbidden_zones = forbidden_zones or []

    # All parameters that affect the output must be in here —
    # any change automatically produces a different hash and a cache miss
    params = {
        "fn":               "generate_sensor_positions",
        "seed":             seed,
        "x_range":          x_range,
        "y_range":          y_range,
        "min_spacing":      min_spacing,
        "forbidden_zones":  [vars(z) for z in forbidden_zones],
        "forbidden_margin": forbidden_margin,
        "jitter":           jitter,
    }

    if not force_regen:
        cached = cache_load("sensor_positions", params)
        if cached is not None:
            # JSON stores tuples as lists; convert back
            return {k: tuple(v) for k, v in cached.items()}

    # ── Generation ──────────────────────────────────────────────────────────
    rng = Random(seed)
    placed: dict[str, tuple[float, float, float]] = {}
    idx = 0

    # Hexagonal grid calculations: height of an equilateral triangle is side * sqrt(3)/2
    row_spacing = min_spacing * sqrt(3) / 2
    col_spacing = min_spacing

    y = y_range[0]
    row = 0
    # Iterate row by row
    while y <= y_range[1]:
        # Stagger every other row by half a column to create the interlocking hex pattern
        x_offset = (col_spacing / 2) if (row % 2 == 1) else 0.0
        x = x_range[0] + x_offset

        # Iterate column by column within the row
        while x <= x_range[1]:
            # Add random jitter to make it look less strictly artificial
            jx = rng.uniform(-jitter, jitter)
            jy = rng.uniform(-jitter, jitter)
            cx, cy = x + jx, y + jy

            # Check if this precise point lands inside a forbidden zone
            in_obstacle = any(
                zone.contains(cx, cy, margin=forbidden_margin)
                for zone in forbidden_zones
            )

            if not in_obstacle:
                placed[f"sensor_{idx}"] = (cx, cy, 0.0)
                idx += 1

            x += col_spacing
        y += row_spacing
        row += 1

    cache_save("sensor_positions", params, {k: list(v) for k, v in placed.items()})
    return placed


def compute_cluster_waypoints(
    start: tuple[float, float],
    end: tuple[float, float],
    comm_range: float,
    force_regen: bool = False,
) -> list[tuple[float, float, float]]:
    """
    Computes a straight line path of waypoints from `start` to `end`.
    Waypoints are spaced out evenly by `comm_range` distance.
    """
    params = {
        "fn":         "compute_cluster_waypoints",
        "start":      start,
        "end":        end,
        "comm_range": comm_range,
    }

    if not force_regen:
        cached = cache_load("cluster_waypoints", params)
        if cached is not None:
            return [tuple(v) for v in cached]

    # ── Generation ──────────────────────────────────────────────────────────
    # Calculate the vector from start to end and normalize it into a unit vector (ux, uy)
    dx, dy = end[0] - start[0], end[1] - start[1]
    total  = hypot(dx, dy)
    ux, uy = dx / total, dy / total

    waypoints: list[tuple[float, float, float]] = []
    dist = 0.0
    
    # Walk along the line by `comm_range` steps
    while dist <= total:
        waypoints.append((start[0] + ux * dist, start[1] + uy * dist, 0.0))
        dist += comm_range

    # Ensure the exact end point is always included as the final waypoint
    if waypoints[-1][:2] != (end[0], end[1]):
        waypoints.append((end[0], end[1], 0.0))

    cache_save("cluster_waypoints", params, [list(v) for v in waypoints])
    return waypoints


# ── Polyline interpolation helper ──────────────────────────────────────────

def _interpolate_polyline(
    corners: list[tuple[float, float]],
    spacing: float,
) -> list[tuple[float, float, float]]:
    """
    Walk along a 2-D polyline defined by *corners* and place waypoints
    every *spacing* units.  Returns 3-D tuples with ``z = 0``.

    The first corner is always included.  Fractional distances are carried
    across segment boundaries so waypoints remain evenly spaced even when
    a segment is shorter than *spacing*.
    """
    if len(corners) < 2:
        return [(corners[0][0], corners[0][1], 0.0)] if corners else []

    waypoints: list[tuple[float, float, float]] = [
        (corners[0][0], corners[0][1], 0.0)
    ]
    carry = 0.0  # distance traveled since the last placed waypoint

    for i in range(len(corners) - 1):
        sx, sy = corners[i]
        ex, ey = corners[i + 1]
        dx, dy = ex - sx, ey - sy
        seg_len = hypot(dx, dy)
        if seg_len < 1e-9:
            continue

        ux, uy = dx / seg_len, dy / seg_len

        d = spacing - carry  # distance to first candidate in this segment
        placed_any = False
        while d <= seg_len:
            waypoints.append((sx + ux * d, sy + uy * d, 0.0))
            d += spacing
            placed_any = True

        if placed_any:
            carry = seg_len - (d - spacing)
        else:
            carry += seg_len

    # Guarantee the final corner is included
    last = corners[-1]
    if waypoints[-1][:2] != (last[0], last[1]):
        waypoints.append((last[0], last[1], 0.0))

    return waypoints


# ── Split waypoints (left / right around obstacle) ────────────────────────

def compute_split_waypoints(
    start: tuple[float, float],
    end: tuple[float, float],
    obstacle: ForbiddenZone,
    comm_range: float,
    margin: float = 10.0,
    force_regen: bool = False,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """
    Compute two waypoint paths that diverge around an obstacle and converge
    afterward, creating a split-and-merge pattern suitable for testing
    leader-election algorithms.

    Both paths share the same *start* and *end* but take opposite sides of
    the obstacle.  Waypoints are spaced by *comm_range* along each path.

    Args:
        start:      (x, y) origin (e.g. base position).
        end:        (x, y) destination (e.g. endpoint position).
        obstacle:   The ForbiddenZone to route around.
        comm_range: Spacing between consecutive waypoints.
        margin:     Clearance between the obstacle boundary and the
                    bypass path centre-line.

    Returns:
        ``(left_waypoints, right_waypoints)`` — each a list of
        ``(x, y, 0.0)`` tuples.
    """
    params = {
        "fn":         "compute_split_waypoints",
        "start":      start,
        "end":        end,
        "obstacle":   vars(obstacle),
        "comm_range": comm_range,
        "margin":     margin,
    }

    if not force_regen:
        cached = cache_load("split_waypoints", params)
        if cached is not None:
            return (
                [tuple(v) for v in cached["left"]],
                [tuple(v) for v in cached["right"]],
            )

    # Key Y-coordinates (approach / departure)
    pre_y  = obstacle.y_min - margin
    post_y = obstacle.y_max + margin

    # Key X-coordinates (bypass offsets)
    left_x  = obstacle.x_min - margin
    right_x = obstacle.x_max + margin

    left_corners = [
        start,
        (start[0], pre_y),      # approach
        (left_x,   pre_y),      # turn left
        (left_x,   post_y),     # along left side
        (end[0],   post_y),     # rejoin centre
        end,
    ]

    right_corners = [
        start,
        (start[0], pre_y),      # approach
        (right_x,  pre_y),      # turn right
        (right_x,  post_y),     # along right side
        (end[0],   post_y),     # rejoin centre
        end,
    ]

    left_wps  = _interpolate_polyline(left_corners,  comm_range)
    right_wps = _interpolate_polyline(right_corners, comm_range)

    cache_save("split_waypoints", params, {
        "left":  [list(v) for v in left_wps],
        "right": [list(v) for v in right_wps],
    })

    return left_wps, right_wps