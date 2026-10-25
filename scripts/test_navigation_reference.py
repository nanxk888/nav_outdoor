import sys
import math
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.navigation_reference import heading,registration


class ReferenceTests(unittest.TestCase):
    def test_tilted_forward_projects_correctly(self):
        a=math.radians(-30)
        self.assertAlmostEqual(heading((0,math.sin(a/2),0,math.cos(a/2))),0)
        # yaw 90 followed by pitch -30
        z=math.pi/2
        self.assertAlmostEqual(heading((-math.sin(z/2)*math.sin(a/2),math.cos(z/2)*math.sin(a/2),math.sin(z/2)*math.cos(a/2),math.cos(z/2)*math.cos(a/2))),z)
    def test_north_anchor_from_nonzero_lio_position(self):
        x,y,a=registration((100,200),(3,4),0,0)
        self.assertAlmostEqual(x+math.cos(a)*3-math.sin(a)*4,100)
        self.assertAlmostEqual(y+math.sin(a)*3+math.cos(a)*4,200)
        self.assertAlmostEqual(a,math.pi/2)
    def test_invalid_or_vertical_axis_rejected(self):
        with self.assertRaises(ValueError):heading((0,0,0,0))
        with self.assertRaises(ValueError):heading((0,math.sin(math.pi/4),0,math.cos(math.pi/4)))


if __name__=='__main__':unittest.main()
