#!/usr/bin/env python3
"""Deterministic five-second/360-second and bounded-memory regressions."""
import unittest
from d1h_nav2.obstacle_memory import ObstacleMemory, cluster_points


class MemoryTest(unittest.TestCase):
    def test_immediate_obstacles_include_motion_with_independent_two_minute_ttl(self):
        m = ObstacleMemory(stable_sec=0., retention_sec=120., remember_all=True)
        m.update([((1., 0.), {(4, 0)})], 0.)
        self.assertEqual(m.occupied(0.), {(4, 0)})
        m.update([((1.5, 0.), {(6, 0)})], 1.)
        self.assertEqual(m.occupied(1.), {(4, 0), (6, 0)})
        self.assertIn((4, 0), m.occupied(119.9))
        self.assertEqual(m.occupied(120.), {(6, 0)})
        self.assertFalse(m.occupied(121.))

    def test_immediate_refresh_capacity_and_clock_reset(self):
        m = ObstacleMemory(retention_sec=120., max_cells=2, remember_all=True)
        m.update([((0., 0.), {(0, 0)})], 0.)
        m.update([((0., 0.), {(0, 0)})], 100.)
        self.assertTrue(m.occupied(219.9))
        self.assertFalse(m.occupied(220.))
        m.update([((0., 0.), {(1, 0)})], 221.)
        m.update([((0., 0.), {(2, 0), (3, 0)})], 222.)
        self.assertEqual(m.occupied(222.), {(2, 0), (3, 0)})
        self.assertFalse(m.occupied(0.))

    def test_immediate_mode_has_no_256_cluster_filter(self):
        m = ObstacleMemory(retention_sec=120., remember_all=True)
        m.update([((float(i), 0.), {(i, 0)}) for i in range(300)], 0.)
        self.assertEqual(len(m.occupied(0.)), 300)

    def test_stationary_promotes_only_after_five_seconds_and_expires(self):
        m = ObstacleMemory()
        group = [((1., 1.), {(4, 4), (4, 5)})]
        for i in range(50):
            m.update(group, i*.1)
        self.assertFalse(m.occupied(4.9))
        m.update(group, 5.)
        self.assertEqual(m.occupied(5.), group[0][1])
        self.assertTrue(m.occupied(364.9))
        self.assertFalse(m.occupied(365.))

    def test_motion_cumulative_and_departure_revoke_memory(self):
        m = ObstacleMemory()
        for i in range(70):
            m.update([((i*.02, 0.), {(int(i*.02/.25), 0)})], i*.1)
            self.assertFalse(m.occupied(i*.1))
        m = ObstacleMemory()
        for i in range(51):
            m.update([((1., 1.), {(4, 4)})], i*.1)
        self.assertTrue(m.occupied(5.))
        m.update([((1.3, 1.), {(5, 4)})], 5.1)
        self.assertFalse(m.occupied(5.1))

    def test_discontinuous_sightings_do_not_accumulate(self):
        m = ObstacleMemory()
        group = [((1., 1.), {(4, 4)})]
        for t in (0., 1., 2., 3., 4., 5., 6., 7.):
            m.update(group, t)
        self.assertFalse(m.occupied(7.))

    def test_refresh_expiry_capacity_and_clock_reset(self):
        m = ObstacleMemory(max_tracks=2, max_cells=2)
        for i in range(61):
            m.update([((float(j), 0.), {(j*4, 0)}) for j in range(5)], i*.1)
        self.assertLessEqual(len(m.tracks), 2)
        self.assertLessEqual(len(m.occupied(6.)), 2)
        self.assertTrue(m.occupied(365.))  # Refreshed through t=6, not t=5.
        self.assertFalse(m.occupied(366.))
        m.update([((0., 0.), {(0, 0)})], 0.)
        self.assertFalse(m.occupied(0.))

    def test_clustering_and_input_bound(self):
        groups = cluster_points([(0., 0.), (.1, .1), (3., 3.)])
        self.assertEqual(len(groups), 2)
        self.assertLessEqual(sum(len(g[1]) for g in cluster_points(
            [(float(i), 0.) for i in range(100)], max_cells=8)), 8)

    def test_new_surface_needs_its_own_five_seconds(self):
        m = ObstacleMemory()
        for i in range(51):
            m.update([((0., 0.), {(0, 0), (1, 0)})], i*.1)
        for i in range(51, 100):
            m.update([((0., 0.), {(0, 0), (1, 0), (2, 0)})], i*.1)
        self.assertNotIn((2, 0), m.occupied(9.9))
        m.update([((0., 0.), {(0, 0), (1, 0), (2, 0)})], 10.2)
        self.assertIn((2, 0), m.occupied(10.2))


if __name__ == '__main__':
    unittest.main()
