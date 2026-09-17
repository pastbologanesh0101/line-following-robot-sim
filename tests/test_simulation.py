import unittest

from lfr_sim.kinematics import Pose
from lfr_sim.pid import PIDController
from lfr_sim.robot import LineFollowingRobot
from lfr_sim.sensors import SensorArray
from lfr_sim.simulation import run_simulation
from lfr_sim.track import initial_heading, make_curvy_track, make_straight_track


def _build_robot(kp, ki, kd, track):
    x0, y0 = track[0]
    heading0 = initial_heading(track)
    pose = Pose(x0, y0, heading0)
    pid = PIDController(kp=kp, ki=ki, kd=kd)
    sensors = SensorArray(num_sensors=5, spacing=1.0, lookahead=2.0, falloff_sigma=1.0)
    return LineFollowingRobot(pose=pose, pid=pid, sensors=sensors, base_speed=2.0, wheel_base=1.0)


class TestSimulation(unittest.TestCase):
    def test_well_tuned_pid_completes_straight_track(self):
        track = make_straight_track(length=40.0)
        robot = _build_robot(kp=1.2, ki=0.05, kd=0.6, track=track)
        result = run_simulation(robot, track, steps=150, dt=0.1, max_error=3.0)

        self.assertTrue(result.completed)
        self.assertEqual(result.steps_run, 150)
        self.assertIsNone(result.derailed_at_step)
        self.assertLess(result.max_cross_track_error, 3.0)

    def test_badly_tuned_pid_derails_on_curvy_track(self):
        track = make_curvy_track(length=150.0)
        # Zero gains: the robot applies no corrective steering at all, so
        # as soon as the track curves away from a straight line it will
        # drive right off the line.
        robot = _build_robot(kp=0.0, ki=0.0, kd=0.0, track=track)
        result = run_simulation(robot, track, steps=800, dt=0.1, max_error=3.0)

        self.assertFalse(result.completed)
        self.assertTrue(result.derailed)
        self.assertIsNotNone(result.derailed_at_step)

    def test_simulation_stops_early_on_derailment(self):
        track = make_curvy_track(length=150.0)
        robot = _build_robot(kp=0.0, ki=0.0, kd=0.0, track=track)
        planned_steps = 800
        result = run_simulation(robot, track, steps=planned_steps, dt=0.1, max_error=3.0)

        # It must not have pointlessly run the full step budget.
        self.assertLess(result.steps_run, planned_steps)
        self.assertEqual(result.steps_run, result.derailed_at_step + 1)
        self.assertEqual(len(result.log), result.steps_run)

    def test_well_tuned_pid_keeps_low_average_error_on_curvy_track(self):
        track = make_curvy_track(length=150.0)
        robot = _build_robot(kp=1.4, ki=0.05, kd=0.7, track=track)
        result = run_simulation(robot, track, steps=1200, dt=0.1, max_error=4.0)

        self.assertTrue(result.completed)
        self.assertLess(result.avg_cross_track_error, 1.5)


if __name__ == "__main__":
    unittest.main()
