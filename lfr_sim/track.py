"""Track generation: dense polylines representing the black line on the floor.

Tracks are authored programmatically as dense lists of (x, y) waypoints --
either a plain straight line, or a curvy track built from a sum of sine
waves (gentle curves) with an explicit tight circular arc appended at the
end (the "at least one sharper turn" requirement).
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Point = Tuple[float, float]


def make_straight_track(length: float = 60.0, step: float = 0.5) -> List[Point]:
    """The simplest possible track: a straight line along the x axis."""
    n = int(length / step) + 1
    return [(i * step, 0.0) for i in range(n)]


def make_curvy_track(length: float = 150.0, step: float = 0.5) -> List[Point]:
    """A curvy track: gentle sine-wave curves followed by one sharp turn.

    The gentle part is a sum of two sine waves of different frequency and
    phase (straights blend into curves naturally). A tight circular arc
    (~100 degrees at a small radius) is then appended, tangent to the end
    of the sine section, to give the track a genuinely sharp turn.
    """
    n = int(length / step) + 1
    points: List[Point] = []
    for i in range(n):
        x = i * step
        y = 8.0 * math.sin(x / 18.0) + 3.0 * math.sin(x / 6.0 + 1.0)
        points.append((x, y))

    x0, y0 = points[-1]
    x1, y1 = points[-2]
    heading = math.atan2(y0 - y1, x0 - x1)

    radius = 6.0
    turn_angle = math.radians(100.0)
    # Center of the turn circle is 90 degrees to the left of the current heading,
    # so the arc starts tangent to the track and curves sharply to one side.
    cx = x0 + radius * math.cos(heading + math.pi / 2)
    cy = y0 + radius * math.sin(heading + math.pi / 2)
    start_angle = math.atan2(y0 - cy, x0 - cx)

    steps = 60
    for i in range(1, steps + 1):
        angle = start_angle + (i / steps) * turn_angle
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)
        points.append((px, py))

    return points


def initial_heading(track: Sequence[Point]) -> float:
    """Heading (radians) of the tangent direction at the start of the track."""
    (x0, y0), (x1, y1) = track[0], track[1]
    return math.atan2(y1 - y0, x1 - x0)
