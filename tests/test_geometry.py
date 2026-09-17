import math
import unittest

from lfr_sim.geometry import point_polyline_distance, point_segment_distance


class TestPointSegmentDistance(unittest.TestCase):
    def test_point_on_segment_is_zero(self):
        d = point_segment_distance((3.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        self.assertAlmostEqual(d, 0.0)

    def test_known_perpendicular_distance(self):
        # Segment along the x axis; point directly "above" the midpoint.
        d = point_segment_distance((5.0, 3.0), (0.0, 0.0), (10.0, 0.0))
        self.assertAlmostEqual(d, 3.0)

    def test_projection_clamps_to_nearest_endpoint(self):
        # Point is beyond the end of the segment -- distance should be to
        # endpoint b, not to the infinite extension of the line.
        d = point_segment_distance((15.0, 0.0), (0.0, 0.0), (10.0, 0.0))
        self.assertAlmostEqual(d, 5.0)

    def test_projection_clamps_to_start_endpoint(self):
        d = point_segment_distance((-4.0, 3.0), (0.0, 0.0), (10.0, 0.0))
        self.assertAlmostEqual(d, 5.0)  # hypot(4, 3) == 5

    def test_degenerate_segment(self):
        d = point_segment_distance((3.0, 4.0), (0.0, 0.0), (0.0, 0.0))
        self.assertAlmostEqual(d, 5.0)


class TestPointPolylineDistance(unittest.TestCase):
    def test_picks_nearest_of_multiple_segments(self):
        polyline = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
        # Closest to the vertical segment, not the horizontal one.
        d = point_polyline_distance((11.0, 5.0), polyline)
        self.assertAlmostEqual(d, 1.0)

    def test_requires_at_least_two_points(self):
        with self.assertRaises(ValueError):
            point_polyline_distance((0.0, 0.0), [(0.0, 0.0)])

    def test_matches_single_segment_distance(self):
        polyline = [(0.0, 0.0), (10.0, 0.0)]
        d = point_polyline_distance((5.0, 3.0), polyline)
        self.assertAlmostEqual(d, 3.0)


if __name__ == "__main__":
    unittest.main()
