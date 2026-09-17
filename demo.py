#!/usr/bin/env python3
"""Command-line demo: run the 2D line-following simulator.

This is a SOFTWARE SIMULATION -- there is no physical robot, no
microcontroller, and no real motors or IR sensors. Everything the robot
"sees" and "does" is computed geometrically against a virtual track. See
README.md for details and for how the same control code would port to
real hardware.

Examples:
    python demo.py --demo
    python demo.py --track curvy --kp 1.2 --ki 0.05 --kd 0.6
    python demo.py --track straight --kp 0 --ki 0 --kd 0
"""
from __future__ import annotations

import argparse

from lfr_sim.kinematics import Pose
from lfr_sim.pid import PIDController
from lfr_sim.robot import LineFollowingRobot
from lfr_sim.sensors import SensorArray
from lfr_sim.simulation import SimulationResult, run_simulation
from lfr_sim.track import initial_heading, make_curvy_track, make_straight_track

TRACKS = {
    # A longer straight track than the library default so the default
    # --steps budget doesn't run the robot off the end of the line.
    "straight": lambda: make_straight_track(length=200.0),
    "curvy": make_curvy_track,
}


def build_robot(kp: float, ki: float, kd: float, track) -> LineFollowingRobot:
    x0, y0 = track[0]
    heading0 = initial_heading(track)
    pose = Pose(x0, y0, heading0)
    pid = PIDController(kp=kp, ki=ki, kd=kd)
    sensors = SensorArray(num_sensors=5, spacing=1.0, lookahead=2.0, falloff_sigma=1.0)
    return LineFollowingRobot(pose=pose, pid=pid, sensors=sensors, base_speed=2.0, wheel_base=1.0)


def report(name: str, result: SimulationResult) -> None:
    status = "COMPLETED" if result.completed else "DERAILED"
    print(f"--- {name} ---")
    print(f"  status:              {status}")
    print(f"  steps run:           {result.steps_run}/{result.steps_planned}")
    print(f"  avg cross-track err: {result.avg_cross_track_error:.3f}")
    print(f"  max cross-track err: {result.max_cross_track_error:.3f}")
    if result.derailed_at_step is not None:
        print(f"  derailed at step:    {result.derailed_at_step}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="2D PID line-following robot simulator")
    parser.add_argument("--track", choices=sorted(TRACKS), default="curvy")
    parser.add_argument("--kp", type=float, default=1.2)
    parser.add_argument("--ki", type=float, default=0.05)
    parser.add_argument("--kd", type=float, default=0.6)
    parser.add_argument("--steps", type=int, default=800)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--max-error", type=float, default=3.0)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Ignore other PID flags; run a fixed well-tuned vs. badly-tuned comparison on the curvy track",
    )
    args = parser.parse_args()

    if args.demo:
        track = make_curvy_track()

        well = build_robot(kp=1.2, ki=0.05, kd=0.6, track=track)
        report(
            "well-tuned  (kp=1.2 ki=0.05 kd=0.6) on curvy track",
            run_simulation(well, track, steps=args.steps, dt=args.dt, max_error=args.max_error),
        )

        bad = build_robot(kp=0.0, ki=0.0, kd=0.0, track=track)
        report(
            "badly-tuned (kp=0 ki=0 kd=0, no correction at all) on curvy track",
            run_simulation(bad, track, steps=args.steps, dt=args.dt, max_error=args.max_error),
        )
        return

    track = TRACKS[args.track]()
    robot = build_robot(kp=args.kp, ki=args.ki, kd=args.kd, track=track)
    result = run_simulation(robot, track, steps=args.steps, dt=args.dt, max_error=args.max_error)
    report(f"track={args.track} kp={args.kp} ki={args.ki} kd={args.kd}", result)


if __name__ == "__main__":
    main()
