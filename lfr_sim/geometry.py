"""Geometric helpers: point-to-segment and point-to-polyline distance.

These are used both to simulate IR sensor readings (how far is a sensor's
virtual position from the black line?) and to compute the robot's true
cross-track error (perpendicular distance to the track centerline) for
grading a simulation run.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

Point = Tuple[float, float]


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Shortest Euclidean distance from point ``p`` to the segment ``a``-``b``.

    Projects ``p`` onto the infinite line through ``a`` and ``b``, clamps
    the projection parameter to [0, 1] so it stays on the segment, then
    returns the distance to that clamped point.
    """
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq == 0.0:
        # Degenerate segment (a == b): just the distance to the point.
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)


def point_polyline_distance(p: Point, polyline: Sequence[Point]) -> float:
    """Shortest distance from ``p`` to the nearest segment of ``polyline``."""
    if len(polyline) < 2:
        raise ValueError("polyline needs at least two points")

    best = math.inf
    for a, b in zip(polyline, polyline[1:]):
        d = point_segment_distance(p, a, b)
        if d < best:
            best = d
    return best
