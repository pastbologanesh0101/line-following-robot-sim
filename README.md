# Line Following Robot (2D Software Simulator)

A small, self-contained Python simulation of a **PID-controlled line-following
robot**.

> **This is a 2D software simulation. There is no real robot, no
> microcontroller, and no physical sensors or motors.** Everything the
> "robot" sees and does — its IR sensor bar, its wheels, the black line on
> the floor — is a geometric abstraction computed in Python against a
> virtual track. What *is* real is the control algorithm: a genuine PID
> controller reacting to a genuine (simulated) sensor-error signal, driving
> genuine differential-drive kinematics. That algorithm is directly testable
> here, and would port to real hardware (e.g. an Arduino or Raspberry Pi
> driving two DC motors and a 5-sensor IR reflectance array) by swapping out
> only the sensor-reading and motor-output functions — the `PIDController`
> and error-calculation logic would not need to change.

## How it works

1. **Track** (`lfr_sim/track.py`) — the "black line" is a dense polyline of
   `(x, y)` waypoints. `make_curvy_track()` builds one from a sum of two sine
   waves (gentle curves blending into straights) with a tight ~100° circular
   arc appended at the end (a genuinely sharp turn). `make_straight_track()`
   is a trivial straight line, useful as a sanity check.

2. **Sensors** (`lfr_sim/sensors.py`) — a `SensorArray` of N (default 5)
   virtual IR sensors sits on a bar mounted a fixed `lookahead` distance in
   front of the robot, spaced evenly across its width. Each sensor's world
   position is computed from the robot's current pose, its true distance to
   the *nearest point on the track polyline* is computed geometrically
   (point-to-segment distance, minimized over every segment), and that
   distance is converted to a reading in `[0, 1]` via a Gaussian falloff
   (`1.0` = directly over the line, `~0.0` = far from it).

3. **Control** (`lfr_sim/robot.py`, `lfr_sim/pid.py`) — the sensor readings
   are combined into the classic line-follower error metric: a weighted
   average of each sensor's lateral offset, weighted by how strongly it
   sees the line (`weighted_line_error`). That single error number is fed
   into a real `PIDController` (proportional + integral + derivative), and
   its output is the steering correction applied to the left/right wheel
   speeds.

4. **Kinematics** (`lfr_sim/kinematics.py`) — the two wheel speeds are
   turned into a forward speed and turn rate and integrated forward one
   timestep using standard differential-drive equations — the same
   equations that describe a real two-motor robot chassis.

5. **Simulation loop** (`lfr_sim/simulation.py`) — steps the robot forward
   for up to N timesteps, tracking the robot's *true* cross-track error
   (perpendicular distance from its actual position to the track
   centerline — independent of what its own sensors think). If that error
   ever exceeds a tolerance, the run stops immediately and is graded
   `DERAILED`; otherwise it runs to completion and is graded `COMPLETED`.

## What is real vs. simulated

| Component | Status |
|---|---|
| PID controller (`PIDController`) | Real, general-purpose PID math |
| Error calculation (`weighted_line_error`) | Real line-follower error formula |
| Differential-drive kinematics | Real equations of motion |
| IR sensor readings | **Simulated** — geometric distance-to-line, not a real photodiode |
| Track / line on the floor | **Simulated** — a polyline in Python, not a physical surface |
| Motors / wheels | **Simulated** — numbers, not physical actuators |

To port this to real hardware, you would replace `SensorArray.read()` with
code that reads actual analog/digital IR sensor pins, and replace the
wheel-speed assignment in `LineFollowingRobot.step()` with PWM writes to
real motor drivers. The `PIDController` and `weighted_line_error` logic
would be reused unchanged.

## Running the demo

No third-party dependencies are required (pure standard library). Requires
Python 3.11+.

```bash
# Well-tuned vs. badly-tuned comparison on the curvy track
python demo.py --demo

# Run one configuration yourself
python demo.py --track curvy --kp 1.2 --ki 0.05 --kd 0.6
python demo.py --track straight --kp 1.2 --ki 0.05 --kd 0.6

# Options
python demo.py --help
```

### Example output

A well-tuned PID configuration completes the curvy track (including its
sharp turn) with very little error, while a "badly-tuned" configuration with
all gains set to zero (i.e. it applies no corrective steering at all) drifts
off the line and derails partway through. This is real output from
`python demo.py --demo`:

```
--- well-tuned  (kp=1.2 ki=0.05 kd=0.6) on curvy track ---
  status:              COMPLETED
  steps run:           800/800
  avg cross-track err: 0.010
  max cross-track err: 0.032

--- badly-tuned (kp=0 ki=0 kd=0, no correction at all) on curvy track ---
  status:              DERAILED
  steps run:           54/800
  avg cross-track err: 0.997
  max cross-track err: 3.068
  derailed at step:    53
```

The well-tuned robot never lets the true cross-track error exceed 0.032
units; the untuned one exceeds the 3.0-unit derailment tolerance by step 53
and the simulation stops immediately rather than continuing to run a robot
that has already lost the line.

## Running the tests

```bash
pip install pytest   # or use the stdlib: python -m unittest discover -s tests
pytest -v
```

33 unit tests cover:

- Point-to-segment / point-to-polyline geometry (including known,
  hand-computed distances and endpoint clamping) — the same math used both
  for sensor readings and for cross-track error.
- Sensor readings: a sensor directly over the line reads near `1.0`; a
  sensor far from the line reads near `0.0`; readings fall off monotonically
  with distance.
- `PIDController`: zero error yields zero output, positive/negative error
  yields correctly-signed output, the integral term accumulates, the
  derivative term reacts to a changing error, and `reset()` clears history.
- `weighted_line_error`: symmetric sensor readings give zero error, a
  stronger reading on one side gives an error of the matching sign, and no
  signal at all is correctly reported as "line lost".
- Differential-drive kinematics: equal wheel speeds move straight, opposite
  speeds rotate in place with no translation, differing speeds turn and
  move together, and heading is respected correctly.
- End-to-end simulation: a well-tuned PID configuration completes a
  straight track and keeps low average error on the curvy track; a
  zero-gain configuration is correctly detected as derailing on the curvy
  track; the simulation loop stops early on derailment instead of running
  the full step budget.

CI (`.github/workflows/tests.yml`) runs the full suite on every push and
pull request against Python 3.11 and 3.12.

## Project layout

```
lfr_sim/
  geometry.py    # point-to-segment / point-to-polyline distance
  track.py       # track (polyline) generation
  sensors.py     # simulated IR sensor array
  pid.py         # PID controller
  kinematics.py  # differential-drive kinematics
  robot.py       # ties sensors + PID + kinematics together
  simulation.py  # the run loop, pass/fail grading
demo.py          # CLI entry point
tests/           # unit tests (unittest, runnable via pytest too)
```

## License

MIT — see [LICENSE](LICENSE).
