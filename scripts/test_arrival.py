import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.arrival import Arrival,approach_cap
class ArrivalTests(unittest.TestCase):
    def test_stop_and_confirm_not_single_sample(self):
        a=Arrival();self.assertEqual(a.update((0,0),0,1.,.5,1.),'TRACKING')
        self.assertEqual(a.update((0,0),0,1.1,.1,1.1),'BRAKING')
        for i in range(1,12):result=a.update((0,0),0,1.1+i*.1,.1,1.1+i*.1)
        self.assertEqual(result,'SETTLED')
    def test_moving_overshoot_timeout_and_duplicates(self):
        a=Arrival();a.update((0,0),0,1.,.1,1.)
        self.assertEqual(a.update((.2,0),0,1.1,.3,1.1),'OVERSHOOT')
        a=Arrival();a.update((0,0),0,1.,.1,1.)
        for i in range(10):self.assertEqual(a.update((0,0),0,1.,.1,1.2),'BRAKING')
        self.assertEqual(a.update((0,0),0,1.,.1,6.1),'TIMEOUT')
        self.assertLessEqual(approach_cap(.3),.15)
    def test_stopped_outside_arrival_radius_is_not_success(self):
        a=Arrival();a.update((0,0),0,1.,.1,1.)
        for i in range(1,15):result=a.update((0,0),0,1.+i*.1,.2,1.+i*.1)
        self.assertEqual(result,'BRAKING')
if __name__=='__main__':unittest.main()
