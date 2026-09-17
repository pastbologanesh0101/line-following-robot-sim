"""LineFollowingRobot: ties sensors, PID control, and kinematics together.

Each call to ``step()`` does what a real line-follower's firmware loop
does every few milliseconds: read the sensor bar, turn that into an
error signal, run the error through the PID controller, convert the
correction into left/right wheel speeds, and move.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from .kinematics import Pose, step_differential_drive
from .pid import PIDController
from .sensors import SensorArray

Point = Tuple[float, float]


def weighted_line_error(
    readings: Sequence[float],
    offsets: Sequence[float],
    min_signal: float = 1e-3,
) -> Tuple[float, bool]:
    """Classic line-follower error metric.

    Error is the weighted average of each sensor's lateral offset,
    weighted by how strongly that sensor sees the line. A sensor far from
    center that reads strongly pulls the error further from zero than a
    center sensor reading the same strength -- exactly the "higher
    weight = further from center = larger error" rule.

    Returns ``(error, line_seen)``. If every sensor reads (almost)
    nothing, the line has been lost and ``line_seen`` is False.
    """
    total_weight = sum(readings)
    if total_weight < min_signal:
        return 0.0, False

    weighted_sum = sum(r * o for r, o in zip(readings, offsets))
    return weighted_sum / total_weight, True


@dataclass
class LineFollowingRobot:
    pose: Pose
    pid: PIDController
    sensors: SensorArray = field(default_factory=SensorArray)
    base_speed: float = 2.0
    wheel_base: float = 1.0
    max_wheel_speed: float = 6.0
    last_error: float = 0.0

    def sense_error(self, track: Sequence[Point]) -> Tuple[float, bool]:
        readings = self.sensors.read(self.pose.x, self.pose.y, self.pose.heading, track)
        offsets = self.sensors.offsets()
        return weighted_line_error(readings, offsets)

    def step(self, track: Sequence[Point], dt: float) -> float:
        """Advance the robot by one control step. Returns the error used."""
        error, seen = self.sense_error(track)
        if not seen:
            # Line lost entirely: keep steering hard toward whichever side
            # it was last seen on, same as a real robot's "line lost"
            # recovery behaviour.
            extreme = self.sensors.offsets()[-1]
            sign = 1.0 if self.last_error >= 0 else -1.0
            error = sign * extreme
        self.last_error = error

        correction = self.pid.update(error, dt)

        # Positive error means the line is to the left, so steer left:
        # speed up the right wheel relative to the left wheel.
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction
        left_speed = max(-self.max_wheel_speed, min(self.max_wheel_speed, left_speed))
        right_speed = max(-self.max_wheel_speed, min(self.max_wheel_speed, right_speed))

        self.pose = step_differential_drive(self.pose, left_speed, right_speed, self.wheel_base, dt)
        return error
