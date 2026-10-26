"""Offline local tracking geometry and unknown-space regression; no robot."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.local_navigation import track
from d1h_nav2.navigation_reference import body_heading


class LocalTests(unittest.TestCase):
    def grid(self):return (.25,40,40,-5.,-5.,[0]*1600)
    def test_forward_turn_and_arrival(self):
        g=self.grid();path=[(0,0),(.5,0),(1,0)]
        v,w,done=track((0,0),0,(1,0),path,g)
        self.assertTrue(0<v<=.15);self.assertEqual(w,0);self.assertFalse(done)
        v,w,_=track((0,0),math.pi,(1,0),path,g)
        self.assertEqual(v,0);self.assertLessEqual(abs(w),.35)
        self.assertEqual(track((.9,0),0,(1,0),path,g),(0.,0.,True))
    def test_unknown_obstacle_and_missing_goal_rejected(self):
        for value in (-1,100):
            g=self.grid();g[-1][20*40+22]=value
            with self.assertRaises(ValueError):track((0,0),0,(1,0),[(0,0),(1,0)],g)
        with self.assertRaises(ValueError):track((0,0),0,(2,0),[(0,0),(.5,0)],self.grid())
    def test_departure_from_path_rejected(self):
        with self.assertRaises(ValueError):track((0,2),0,(1,0),[(0,0),(1,0)],self.grid())
    def test_corrected_robot_front_is_native_minus_x(self):
        v,w,_=track((0,0),body_heading((0,0,0,1)),(-1,0),[(0,0),(-.5,0),(-1,0)],self.grid())
        self.assertGreater(v,0);self.assertAlmostEqual(w,0)


if __name__=='__main__':unittest.main()
