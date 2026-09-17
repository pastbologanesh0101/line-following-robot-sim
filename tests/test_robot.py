import unittest

from lfr_sim.robot import weighted_line_error


class TestWeightedLineError(unittest.TestCase):
    def test_symmetric_readings_give_zero_error(self):
        readings = [1.0, 1.0, 1.0, 1.0, 1.0]
        offsets = [-2.0, -1.0, 0.0, 1.0, 2.0]
        error, seen = weighted_line_error(readings, offsets)
        self.assertTrue(seen)
        self.assertAlmostEqual(error, 0.0)

    def test_error_sign_matches_stronger_side(self):
        # Stronger signal on the positive-offset (left) sensors.
        readings = [0.1, 0.1, 0.1, 0.5, 0.9]
        offsets = [-2.0, -1.0, 0.0, 1.0, 2.0]
        error, seen = weighted_line_error(readings, offsets)
        self.assertTrue(seen)
        self.assertGreater(error, 0.0)

    def test_error_sign_flips_for_other_side(self):
        readings = [0.9, 0.5, 0.1, 0.1, 0.1]
        offsets = [-2.0, -1.0, 0.0, 1.0, 2.0]
        error, seen = weighted_line_error(readings, offsets)
        self.assertTrue(seen)
        self.assertLess(error, 0.0)

    def test_no_signal_reports_line_lost(self):
        readings = [0.0, 0.0, 0.0, 0.0, 0.0]
        offsets = [-2.0, -1.0, 0.0, 1.0, 2.0]
        error, seen = weighted_line_error(readings, offsets)
        self.assertFalse(seen)
        self.assertAlmostEqual(error, 0.0)

    def test_center_only_signal_gives_zero_error(self):
        readings = [0.0, 0.0, 1.0, 0.0, 0.0]
        offsets = [-2.0, -1.0, 0.0, 1.0, 2.0]
        error, seen = weighted_line_error(readings, offsets)
        self.assertTrue(seen)
        self.assertAlmostEqual(error, 0.0)


if __name__ == "__main__":
    unittest.main()
