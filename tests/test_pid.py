import unittest

from lfr_sim.pid import PIDController


class TestPIDController(unittest.TestCase):
    def test_zero_error_gives_zero_output(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        output = pid.update(error=0.0, dt=0.1)
        self.assertAlmostEqual(output, 0.0)

    def test_positive_error_gives_positive_output(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update(error=1.0, dt=0.1)
        self.assertGreater(output, 0.0)
        self.assertAlmostEqual(output, 2.0)

    def test_negative_error_gives_negative_output(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update(error=-1.0, dt=0.1)
        self.assertLess(output, 0.0)
        self.assertAlmostEqual(output, -2.0)

    def test_integral_term_accumulates_over_time(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        first = pid.update(error=1.0, dt=1.0)
        second = pid.update(error=1.0, dt=1.0)
        self.assertAlmostEqual(first, 1.0)
        self.assertAlmostEqual(second, 2.0)
        self.assertGreater(second, first)

    def test_derivative_term_reacts_to_change(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
        first = pid.update(error=1.0, dt=1.0)  # no previous error yet
        second = pid.update(error=3.0, dt=1.0)  # error jumped by 2
        self.assertAlmostEqual(first, 0.0)
        self.assertAlmostEqual(second, 2.0)

    def test_reset_clears_integral_and_derivative_history(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=1.0)
        pid.update(error=5.0, dt=1.0)
        pid.reset()
        output = pid.update(error=1.0, dt=1.0)
        # After reset, derivative history is gone (treated as first sample)
        # and the integral restarted from zero, so output == ki * error.
        self.assertAlmostEqual(output, 1.0)


if __name__ == "__main__":
    unittest.main()
