"""Synthetic geometry only: no ROS, hardware or navigation commands."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
import unittest
import numpy as np
from d1h_nav2.real_ground import split_ground


class GroundTests(unittest.TestCase):
    def scene(self,slope=0):
        x,y=np.meshgrid(np.linspace(-2,2,35),np.linspace(-2,2,35));x=x.ravel();y=y.ravel()
        return np.c_[x,y,slope*x-.6]
    def test_ground_removed_low_obstacle_retained(self):
        p=np.vstack([self.scene(),[1,0,-.56],[1,.1,-.4],[1,.2,-.585],[1,.3,1.]])
        nav,clear,plane=split_ground(p,(0,0,0),.6)
        self.assertEqual(len(nav),2);self.assertGreater(len(clear),1000)
    def test_local_slope(self):
        ground=self.scene(.08);nav,clear,plane=split_ground(ground,(0,0,0),.6)
        self.assertEqual(len(nav),0);self.assertAlmostEqual(plane[0],.08)
    def test_robot_rises_without_moving_ground(self):
        ground=self.scene();_,_,plane=split_ground(ground,(0,0,0),.6)
        # Same physical ground; IMU rises 40cm. Initial height is NOT reused.
        nav,clear,updated=split_ground(ground,(0,0,.4),.6,reference_plane=plane)
        self.assertEqual(len(nav),0);self.assertAlmostEqual(updated[2],-.6)
    def test_no_valid_ground_fails_closed(self):
        for height in (0,float('nan')):
            with self.assertRaises(ValueError):split_ground(self.scene(),(0,0,0),height)
        with self.assertRaises(ValueError):split_ground(self.scene()+[0,0,2],(0,0,0),.6)
    def test_ground_not_fixed_at_zero(self):
        nav,clear,plane=split_ground(self.scene()+[10,2,4],(10,2,4),.6)
        self.assertEqual(len(nav),0);self.assertAlmostEqual(plane[2],3.4)


if __name__=='__main__':unittest.main()
