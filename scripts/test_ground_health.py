"""Offline timing regression: fresh filtering must survive status intervals."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.ground_health import require_ground,STATUS_PERIOD_SEC


class GroundHealthTests(unittest.TestCase):
    def test_normal_intervals(self):
        self.assertEqual(STATUS_PERIOD_SEC,.1)
        for age in (.08,.17):
            for delay in (0.,.05,.099,.15):
                require_ground({'ready':True,'measurement_age_sec':age},delay)
    def test_measurement_staleness_still_stops(self):
        with self.assertRaisesRegex(ValueError,'Ground measurement stale'):
            require_ground({'ready':True,'measurement_age_sec':.17},.34)
    def test_status_staleness_distinct(self):
        with self.assertRaisesRegex(ValueError,'Ground status stale'):
            require_ground({'ready':True,'measurement_age_sec':.08},.5)
    def test_filter_rejection_preserved(self):
        with self.assertRaisesRegex(ValueError,'ground plane confidence insufficient'):
            require_ground({'ready':False,'reason':'ground plane confidence insufficient'},.01)
    def test_invalid_age(self):
        for age in (None,float('nan'),float('inf'),True,-1):
            with self.assertRaises(ValueError):require_ground({'ready':True,'measurement_age_sec':age},.01)


if __name__=='__main__':unittest.main()
