import sys
import math
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.observed_memory import ObservedMemory
from d1h_nav2.footprint import circle_cells


class MemoryTests(unittest.TestCase):
    def test_noise_confirmation_and_multiframe_clear(self):
        m=ObservedMemory(resolution=.1,min_points=3,confirm_frames=2)
        m.observe((0,0),[(1.,0)],[],1.)
        self.assertNotIn((10,0),m.cells)
        m.observe((0,0),[(1.,0)],[],2.)
        self.assertEqual(m.cells[(10,0)][0],100)
        m.observe((0,0),[],[(2.,0),(2.1,0)],3.)
        self.assertEqual(m.cells[(10,0)][0],100,'multiple rays in one scan are not multiple frames')
        m.observe((0,0),[],[(2.,0)],4.)
        self.assertEqual(m.cells[(10,0)][0],0)
        m.observe((0,0),[(1.,0),(1.01,0),(1.02,0)],[],5.)
        self.assertEqual(m.cells[(10,0)][0],100)
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
    def test_measured_radius_and_exact_smooth_endpoints(self):
        m=ObservedMemory()
        self.assertEqual(m.radius,.25)
        m.cells={(x,y):(0,10.) for x in range(-20,30) for y in range(-20,21)}
        for y in range(-5,6):m.cells[(4,y)]=(100,10.)
        start=(.03,.04);goal=(2.1,.02)
        route=m.route(start,goal,10,False);points=route['points']
        self.assertFalse(route['partial'])
        self.assertEqual(points[0],start);self.assertEqual(points[-1],goal)
        self.assertTrue(any(abs(y)>1.5 for x,y in points))
        self.assertTrue(all(math.dist(a,b)<=.126 for a,b in zip(points,points[1:])))
        # Every dense point retains the same conservative circular grid stencil.
        for p in points:
            for c in circle_cells(p,m.resolution,m.radius):
                self.assertEqual(m.cells.get(c,(-1,0))[0],0)
    def test_exact_circle_has_no_extra_cell_margin(self):
        cells=set(circle_cells((.125,.125),.25,.25))
        self.assertNotIn((2,0),cells)
        self.assertIn((0,0),cells)
        self.assertNotIn((2,2),cells)
        self.assertEqual(set(circle_cells((0,0),.25,.25)),{(-1,-1),(-1,0),(0,-1),(0,0)})
    def test_measured_radius_passes_where_old_radius_does_not(self):
        cells={(x,y):(0,10.) for x in range(-3,12) for y in range(-1,2)}
        m=ObservedMemory(radius=.25);m.cells=dict(cells)
        self.assertFalse(m.route((.125,.125),(2.125,.125),10,False)['partial'])
        old=ObservedMemory(radius=.6);old.cells=dict(cells)
        with self.assertRaises(ValueError):old.route((.125,.125),(2.125,.125),10,False)


class SessionMemoryTests(unittest.TestCase):
    def test_front_cone_can_clear_close_obstacle(self):
        m=ObservedMemory(resolution=.1,retention=0)
        m.cells[(4,0)]=(100,1.)
        m.observe((0,0),[],[(2,0)],2,heading=0)
        self.assertEqual(m.cells[(4,0)][0],100)
        m.observe((0,0),[],[(2,0)],3,heading=0)
        self.assertEqual(m.cells[(4,0)][0],0)
    def test_front_inner_13cm_remains_protected(self):
        m=ObservedMemory(resolution=.1,retention=0)
        m.cells[(0,0)]=(100,1.)
        for t in range(2,6):m.observe((0,0),[],[(2,0)],t,heading=0)
        self.assertEqual(m.cells[(0,0)][0],100)

    def test_near_obstacle_not_cleared_by_overhead_ray(self):
        m=ObservedMemory(resolution=.1,retention=0)
        m.cells[(0,4)]=(100,1.)
        for t in range(2,6):m.observe((0,0),[],[(0,2)],t,heading=0)
        self.assertEqual(m.cells[(0,4)][0],100)
    def test_rear_half_plane_preserved(self):
        m=ObservedMemory(resolution=.1,retention=0)
        m.cells[(-5,8)]=(100,1.)
        for t in range(2,6):m.observe((0,0),[],[(-1,1.6)],t,heading=0)
        self.assertEqual(m.cells[(-5,8)][0],100)

    def test_session_retention_and_reset(self):
        m=ObservedMemory(retention=0,max_cells=0)
        m.cells[(0,0)]=(100,10.)
        m.expire(10000.)
        self.assertEqual(m.cells[(0,0)][0],100)
        m.expire(1.)
        self.assertFalse(m.cells)

if __name__=='__main__':unittest.main()
