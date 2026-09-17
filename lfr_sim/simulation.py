"""The main simulation loop.

Advances a ``LineFollowingRobot`` along a track for up to N steps,
tracking the true cross-track error (perpendicular distance from the
robot's actual position to the track centerline -- not the sensor-based
error the controller uses) at every step. The run is graded as
"completed" if that error never exceeds ``max_error``, and "derailed"
(with an early stop) the first moment it does.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .geometry import point_polyline_distance
from .robot import LineFollowingRobot

Point = Tuple[float, float]


@dataclass
class StepLog:
    step: int
    x: float
    y: float
    heading: float
    error: float
    cross_track_error: float


@dataclass
class SimulationResult:
    completed: bool
    steps_run: int
    steps_planned: int
    log: List[StepLog] = field(default_factory=list)
    max_cross_track_error: float = 0.0
    avg_cross_track_error: float = 0.0
    derailed_at_step: Optional[int] = None

    @property
    def derailed(self) -> bool:
        return not self.completed


def run_simulation(
    robot: LineFollowingRobot,
    track: Sequence[Point],
    steps: int = 500,
    dt: float = 0.1,
    max_error: float = 3.0,
) -> SimulationResult:
    """Run ``robot`` along ``track`` for up to ``steps`` timesteps of length ``dt``.

    Stops early ("derails") the first time the true cross-track error
    exceeds ``max_error``, rather than continuing to run a robot that has
    already lost the track.
    """
    log: List[StepLog] = []
    total_error = 0.0
    max_seen = 0.0
    completed = True
    derailed_at: Optional[int] = None

    for i in range(steps):
        error = robot.step(track, dt)
        cte = point_polyline_distance((robot.pose.x, robot.pose.y), track)

        log.append(StepLog(i, robot.pose.x, robot.pose.y, robot.pose.heading, error, cte))
        total_error += cte
        max_seen = max(max_seen, cte)

        if cte > max_error:
            completed = False
            derailed_at = i
            break

    steps_run = len(log)
    avg_error = total_error / steps_run if steps_run else 0.0

    return SimulationResult(
        completed=completed,
        steps_run=steps_run,
        steps_planned=steps,
        log=log,
        max_cross_track_error=max_seen,
        avg_cross_track_error=avg_error,
        derailed_at_step=derailed_at,
    )
