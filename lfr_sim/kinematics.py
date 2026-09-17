"""Differential-drive kinematics.

Models the robot as two independently-driven wheels (left/right) a fixed
``wheel_base`` apart, exactly like a typical two-motor line-follower
chassis (e.g. built on an Arduino with two DC gear motors). Given the two
wheel speeds, this computes the forward speed and turn rate and
integrates them forward by one timestep using simple Euler integration.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float
    y: float
    heading: float  # radians, 0 = facing +x, increases counterclockwise


def step_differential_drive(
    pose: Pose,
    left_speed: float,
    right_speed: float,
    wheel_base: float,
    dt: float,
) -> Pose:
    """Advance a differential-drive robot by ``dt`` given wheel speeds.

    v = (left + right) / 2 is the forward speed.
    omega = (right - left) / wheel_base is the turn rate (positive =
    turning left / counterclockwise, matching a real differential-drive
    robot where spinning the right wheel faster than the left turns it
    left).
    """
    v = (left_speed + right_speed) / 2.0
    omega = (right_speed - left_speed) / wheel_base

    new_x = pose.x + v * math.cos(pose.heading) * dt
    new_y = pose.y + v * math.sin(pose.heading) * dt
    new_heading = pose.heading + omega * dt

    return Pose(new_x, new_y, new_heading)
