import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.observed_memory import ObservedMemory


class MemoryTests(unittest.TestCase):
    def plane(self):
        m=ObservedMemory(radius=.3)
        m.cells={(x,y):(0,10.) for x in range(-10,31) for y in range(-10,11)}
        return m
    def test_obstacle_route_and_no_corner_cutting(self):
        m=self.plane()
        for y in range(-5,6):m.cells[(10,y)]=(100,10.)
        route=m.route((0,0),(5,0),10,False)
        self.assertFalse(route['partial']);self.assertTrue(any(abs(y)>1.5 for x,y in route['points']))
    def test_unknown_goal_stops_at_frontier(self):
        m=self.plane();route=m.route((0,0),(20,0),10)
        self.assertTrue(route['partial']);self.assertLess(max(x for x,y in route['points']),7.5)
        with self.assertRaises(ValueError):m.route((0,0),(20,0),10,False)
    def test_unknown_start_refused(self):
        with self.assertRaises(ValueError):ObservedMemory().route((0,0),(2,0),10)
    def test_ray_no_return_not_free_and_obstacle_dominates(self):
        m=ObservedMemory();m.observe((0,0),[(1,0)],[(3,0)],10)
        self.assertEqual(m.cells[(4,0)][0],100)
        self.assertNotIn((8,0),m.cells)
        m.observe((0,0),[],[],11);self.assertNotIn((0,10),m.cells)
    def test_expiry_and_clock_reset(self):
        m=self.plane();m.expire(309);self.assertTrue(m.cells)
        m.expire(311);self.assertFalse(m.cells)
        m=self.plane();m.expire(9);self.assertFalse(m.cells)


if __name__=='__main__':unittest.main()
