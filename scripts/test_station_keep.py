#!/usr/bin/env python3
import math
import unittest
from d1h_nav2.station_keep import StationKeep


class StationKeepTest(unittest.TestCase):
    def test_correction_and_release(self):
        hold = StationKeep()
        self.assertEqual(hold.command(0, 0, 0, 0, 0, 0.05), (0, 0))
        v, w = hold.command(0.1, 0, 0.1, 0.05, 0.1, 0.05)
        self.assertLess(v, 0)
        self.assertLess(w, 0)
        self.assertLessEqual(abs(v), 0.15)
        self.assertLessEqual(abs(w), 0.25)
        hold.reset()
        self.assertEqual(hold.command(5, 3, 1, 0, 0, 0.05), (0, 0))

    def test_heading_wrap_and_saturation(self):
        hold = StationKeep()
        hold.command(0, 0, math.pi-0.01, 0, 0, 0.05)
        _, w = hold.command(0, 0, -math.pi+0.01, 0, 0, 0.05)
        self.assertLess(abs(w), 0.03)
        v, w = hold.command(-100, 0, 0, -100, 100, 0.05)
        self.assertLessEqual(abs(v), 0.15)
        self.assertLessEqual(abs(w), 0.25)


if __name__ == '__main__':
    unittest.main()
