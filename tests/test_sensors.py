import unittest

from lfr_sim.sensors import SensorArray
from lfr_sim.track import make_straight_track


class TestSensorArray(unittest.TestCase):
    def setUp(self):
        self.track = make_straight_track(length=60.0)  # the line y == 0

    def test_offsets_are_symmetric_around_center(self):
        sensors = SensorArray(num_sensors=5, spacing=1.0)
        self.assertEqual(sensors.offsets(), [-2.0, -1.0, 0.0, 1.0, 2.0])

    def test_center_sensor_over_line_reads_high(self):
        sensors = SensorArray(num_sensors=5, spacing=1.0, lookahead=2.0, falloff_sigma=1.0)
        readings = sensors.read(robot_x=0.0, robot_y=0.0, heading=0.0, track=self.track)
        center = readings[2]
        self.assertGreater(center, 0.95)

    def test_far_sensor_reads_near_zero(self):
        # Wide spacing puts the outer sensors far from the line while the
        # center sensor stays on it.
        sensors = SensorArray(num_sensors=5, spacing=5.0, lookahead=2.0, falloff_sigma=1.0)
        readings = sensors.read(robot_x=0.0, robot_y=0.0, heading=0.0, track=self.track)
        self.assertGreater(readings[2], 0.95)  # center sensor: on the line
        self.assertLess(readings[0], 0.01)  # far-left sensor: nowhere near it
        self.assertLess(readings[-1], 0.01)  # far-right sensor: nowhere near it

    def test_reading_decreases_monotonically_with_distance(self):
        sensors = SensorArray(num_sensors=5, spacing=1.0, lookahead=2.0, falloff_sigma=1.0)
        readings = sensors.read(robot_x=0.0, robot_y=0.0, heading=0.0, track=self.track)
        # Sensors are symmetric and ordered outer-left..center..outer-right,
        # so the reading should rise then fall around the center index.
        self.assertLess(readings[0], readings[1])
        self.assertLess(readings[1], readings[2])
        self.assertGreater(readings[2], readings[3])
        self.assertGreater(readings[3], readings[4])

    def test_sensor_positions_are_ahead_of_robot(self):
        sensors = SensorArray(num_sensors=3, spacing=1.0, lookahead=4.0)
        positions = sensors.sensor_positions(robot_x=0.0, robot_y=0.0, heading=0.0)
        for x, _y in positions:
            self.assertAlmostEqual(x, 4.0)


if __name__ == "__main__":
    unittest.main()
