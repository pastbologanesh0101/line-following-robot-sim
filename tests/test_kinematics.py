import math
import unittest

from lfr_sim.kinematics import Pose, step_differential_drive


class TestDifferentialDriveKinematics(unittest.TestCase):
    def test_equal_wheel_speeds_move_straight(self):
        pose = Pose(x=0.0, y=0.0, heading=0.0)
        new_pose = step_differential_drive(pose, left_speed=2.0, right_speed=2.0, wheel_base=1.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 2.0)
        self.assertAlmostEqual(new_pose.y, 0.0)
        self.assertAlmostEqual(new_pose.heading, 0.0)

    def test_opposite_wheel_speeds_rotate_in_place(self):
        pose = Pose(x=0.0, y=0.0, heading=0.0)
        new_pose = step_differential_drive(pose, left_speed=-1.0, right_speed=1.0, wheel_base=2.0, dt=1.0)
        # v == 0 so position shouldn't change at all.
        self.assertAlmostEqual(new_pose.x, 0.0)
        self.assertAlmostEqual(new_pose.y, 0.0)
        # omega = (1 - (-1)) / 2 = 1.0 rad/s, for 1s => heading += 1.0
        self.assertAlmostEqual(new_pose.heading, 1.0)

    def test_differing_wheel_speeds_turn_and_move(self):
        pose = Pose(x=0.0, y=0.0, heading=0.0)
        new_pose = step_differential_drive(pose, left_speed=1.0, right_speed=3.0, wheel_base=2.0, dt=1.0)
        # v = (1+3)/2 = 2.0, omega = (3-1)/2 = 1.0
        self.assertAlmostEqual(new_pose.heading, 1.0)
        self.assertAlmostEqual(new_pose.x, 2.0 * math.cos(0.0))
        self.assertAlmostEqual(new_pose.y, 2.0 * math.sin(0.0))

    def test_motion_respects_nonzero_starting_heading(self):
        pose = Pose(x=0.0, y=0.0, heading=math.pi / 2)  # facing +y
        new_pose = step_differential_drive(pose, left_speed=2.0, right_speed=2.0, wheel_base=1.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 0.0, places=6)
        self.assertAlmostEqual(new_pose.y, 2.0, places=6)

    def test_zero_speed_leaves_pose_unchanged(self):
        pose = Pose(x=1.0, y=2.0, heading=0.5)
        new_pose = step_differential_drive(pose, left_speed=0.0, right_speed=0.0, wheel_base=1.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 1.0)
        self.assertAlmostEqual(new_pose.y, 2.0)
        self.assertAlmostEqual(new_pose.heading, 0.5)


if __name__ == "__main__":
    unittest.main()
