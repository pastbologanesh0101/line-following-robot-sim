"""A textbook PID controller.

This is a genuine, general-purpose PID implementation (not a stub): it
accumulates an integral term over time and computes a derivative term
from consecutive errors. It is used here to turn "how far off the line
is the robot" into a steering correction, but it has no idea it is
controlling a simulated robot -- the same class would work unmodified
driving a real motor via PWM.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float

    _integral: float = 0.0
    _prev_error: float = 0.0
    _has_prev: bool = False

    def reset(self) -> None:
        """Clear accumulated state (integral and derivative history)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def update(self, error: float, dt: float) -> float:
        """Compute the PID output for the given error and timestep.

        output = kp*error + ki*integral(error) + kd*d(error)/dt
        """
        self._integral += error * dt

        derivative = 0.0
        if self._has_prev and dt > 0:
            derivative = (error - self._prev_error) / dt

        self._prev_error = error
        self._has_prev = True

        return self.kp * error + self.ki * self._integral + self.kd * derivative
