"""Simulated IR sensor array mounted ahead of the robot.

A real line-follower typically has a small bar of N IR reflectance
sensors mounted across the front of the chassis. Each sensor reports a
strong signal when the black line is directly beneath it and a weak
signal otherwise. Here we simulate that geometrically: each sensor has a
virtual (x, y) position (computed from the robot's pose), and its
reading is a smooth falloff function of the true distance from that
position to the nearest point on the track polyline.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .geometry import point_polyline_distance

Point = Tuple[float, float]


@dataclass
class SensorArray:
    """N virtual IR sensors on a bar mounted ``lookahead`` ahead of the robot,
    spaced ``spacing`` apart, centered on the robot's heading.
    """

    num_sensors: int = 5
    spacing: float = 1.0
    lookahead: float = 2.0
    falloff_sigma: float = 1.0  # controls how quickly the reading drops with distance

    def offsets(self) -> List[float]:
        """Lateral offsets of each sensor from the centerline.

        E.g. for 5 sensors and spacing=1.0: [-2, -1, 0, 1, 2]. Positive
        offsets are to the robot's left, negative to its right.
        """
        half = (self.num_sensors - 1) / 2.0
        return [(i - half) * self.spacing for i in range(self.num_sensors)]

    def sensor_positions(self, robot_x: float, robot_y: float, heading: float) -> List[Point]:
        """World-space (x, y) position of every sensor for the given pose."""
        forward_x, forward_y = math.cos(heading), math.sin(heading)
        # Left-hand perpendicular to the heading vector.
        left_x, left_y = -forward_y, forward_x
        base_x = robot_x + forward_x * self.lookahead
        base_y = robot_y + forward_y * self.lookahead
        return [
            (base_x + left_x * off, base_y + left_y * off)
            for off in self.offsets()
        ]

    def read(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        track: Sequence[Point],
    ) -> List[float]:
        """Return one reading per sensor in [0, 1]; 1.0 = directly over the line."""
        readings = []
        for pos in self.sensor_positions(robot_x, robot_y, heading):
            distance = point_polyline_distance(pos, track)
            reading = math.exp(-(distance ** 2) / (2.0 * self.falloff_sigma ** 2))
            readings.append(reading)
        return readings
