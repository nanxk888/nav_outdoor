"""Offline local tracking geometry and unknown-space regression; no robot."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.local_navigation import track
from d1h_nav2.navigation_reference import body_heading
from d1h_nav2.footprint import path_free


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
    def test_current_circle_does_not_use_extra_square_margin(self):
        g=self.grid()
        # Outside radius .25 at this exact pose; old padded stencil rejected it.
        g[-1][21*40+21]=100
        v,_,_=track((0,0),math.pi,(-1,0),[(0,0),(-1,0)],g)
        self.assertGreater(v,0)
    def test_retained_path_revalidated_against_new_obstacle(self):
        points=[(0,0),(1,0)]
        cells={(x,y):0 for x in range(-4,9) for y in range(-4,5)}
        self.assertTrue(path_free(points,.25,.25,lambda c:cells.get(c,-1)==0))
        cells[(2,0)]=100
        self.assertFalse(path_free(points,.25,.25,lambda c:cells.get(c,-1)==0))


if __name__=='__main__':unittest.main()
