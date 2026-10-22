#!/usr/bin/env python3
"""Real ROS action/service integration for cadence, reuse and emergency bypass."""
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from d1h_nav2.stable_planner import StablePlanner


class StablePlannerTest(unittest.TestCase):
    def test_cadence_cache_and_obstacle_bypass(self):
        rclpy.init()
        stable, probe = StablePlanner(), Node('stable_planner_probe')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(stable)
        executor.add_node(probe)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.transform.rotation.w = 1.
        stable.buffer.set_transform_static(tf, 'test')
        counts, invalid, fail = dict(global_=0, local=0), set(), set()
        servers, services, clients = [], [], {}
        displayed = []
        display_sub = probe.create_subscription(Path, '/navigation/display_plan',
                                                lambda msg: displayed.append(msg), 1)
        def planner(kind):
            async def callback(handle):
                counts[kind] += 1
                result = ComputePathToPose.Result()
                if handle.request.goal.pose.position.x == 99.:
                    deadline = time.monotonic()+.8
                    await stable.wait_ready(lambda: handle.is_cancel_requested or time.monotonic() >= deadline, 1.)
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return result
                if kind in fail:
                    handle.abort()
                    return result
                result.path.header.frame_id = 'map'
                result.path.header.stamp = probe.get_clock().now().to_msg()
                for i in range(21):
                    p = PoseStamped()
                    p.header = result.path.header
                    p.pose.position.x = handle.request.goal.pose.position.x*i/20.
                    if kind == 'local' and kind in invalid:
                        import math
                        p.pose.position.y = math.sin(math.pi*i/20.)
                    p.pose.orientation.w = 1.
                    result.path.poses.append(p)
                handle.succeed()
                return result
            return callback
        def validator(kind):
            def callback(request, response):
                response.is_valid = kind not in invalid or (
                    kind == 'local' and any(p.pose.position.y > .5 for p in request.path.poses))
                return response
            return callback
        for kind, public, backend in [('global_', 'global', ''), ('local', 'local', '/local_planner')]:
            servers.append(ActionServer(probe, ComputePathToPose, backend+'/compute_path_to_pose', planner(kind),
                                        cancel_callback=lambda _: CancelResponse.ACCEPT))
            services.append(probe.create_service(IsPathValid, backend+'/is_path_valid', validator(kind)))
            clients[kind] = ActionClient(probe, ComputePathToPose, '/navigation/compute_'+public+'_path')
        def wait(future, timeout=5.):
            end = time.monotonic()+timeout
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
            self.assertTrue(future.done(), 'action timed out')
            return future.result()
        def spin(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
        def request(kind, x):
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x, goal.goal.pose.orientation.w = x, 1.
            goal.planner_id = 'GridBased' if kind == 'global_' else 'LocalGrid'
            handle = wait(clients[kind].send_goal_async(goal))
            self.assertTrue(handle.accepted)
            return wait(handle.get_result_async())
        try:
            for client in clients.values():
                self.assertTrue(client.wait_for_server(timeout_sec=3.))
            first = request('global_', 12.)
            for _ in range(3):
                again = request('global_', 12.)
                self.assertEqual(again.result.path, first.result.path)
            self.assertEqual(counts['global_'], 1)
            spin(1.05)
            request('global_', 12.)
            self.assertEqual(counts['global_'], 1, 'Valid global route was needlessly replaced')
            invalid.add('global_')
            spin(1.05)
            start = time.monotonic()
            self.assertEqual(request('global_', 12.).status, 4)
            self.assertLess(time.monotonic()-start, 1.)
            self.assertEqual(counts['global_'], 2)
            invalid.clear()
            spin(2.05)
            request('global_', 12.)
            self.assertEqual(counts['global_'], 2, 'Global checks must reuse an unblocked route')
            first = request('local', 5.)
            spin(.55)
            self.assertEqual(request('local', 5.).result.path, first.result.path)
            self.assertEqual(counts['local'], 1, 'Safe local path was needlessly replaced')
            # A moving rolling target may refresh at the next 2Hz slot.
            request('local', 5.1)
            self.assertEqual(counts['local'], 1)
            spin(.25)
            refreshed = request('local', 5.1)
            self.assertEqual(counts['local'], 2)
            self.assertAlmostEqual(refreshed.result.path.poses[-1].pose.position.x, 5.1)
            invalid.add('local')
            start = time.monotonic()
            detour = request('local', 5.1)
            self.assertEqual(detour.status, 4)
            self.assertLess(time.monotonic()-start, .5)
            self.assertEqual(counts['local'], 3)
            self.assertNotEqual(detour.result.path, refreshed.result.path,
                                'Blocked local route was not changed')
            stable.publish_display()
            spin(.1)
            self.assertTrue(displayed)
            near = detour.result.path.poses
            self.assertEqual(displayed[-1].poses[:len(near)], near,
                             'Near-field display contains a global shortcut instead of the local detour')
            fail.add('local')
            # Explicitly exhaust the handover window: transient failures may
            # now succeed with a separately verified safe prefix.
            stable._handover_since = stable.get_clock().now().nanoseconds/1e9-7.
            stable._handover_anchor = (0., 0.)
            spin(.25)
            self.assertEqual(request('local', 5.3).status, 6,
                             'A blocked path was returned as successful after replanning failed')
            self.assertIsNone(stable.cache['local'])
            print('STABLE PLANNER PASS: valid global retained at 1Hz; local endpoint moves 0.1m at 5Hz; blocked paths replan; failed replans abort')
            fail.clear()
            invalid.clear()
            timer_count = len(list(stable.timers))
            for i in range(60):
                response = request('global_', 100.+i)
                self.assertEqual(response.status, 4)
                self.assertAlmostEqual(response.result.path.poses[-1].pose.position.x, 100.+i)
            self.assertEqual(len(list(stable.timers)), timer_count,
                             'Action requests created/destroyed timers while executor was running')
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x, goal.goal.pose.orientation.w = 99., 1.
            handle = wait(clients['global_'].send_goal_async(goal))
            spin(.1)
            cancel = wait(handle.cancel_goal_async())
            self.assertTrue(cancel.goals_canceling)
            self.assertEqual(wait(handle.get_result_async()).status, 5)
            self.assertEqual(request('global_', 22.).status, 4, 'New target failed after cancellation')
            stable.set_parameters([Parameter('use_sim_time', value=True)])
            self.assertEqual(request('global_', 23.).status, 4,
                             'Paused simulation clock stalled action scheduling')
            self.assertFalse(stable._waiters)
            self.assertFalse(stable._cancellations)
            print('SCHEDULER STRESS PASS: 60 target changes, cancellation/new goal, paused clock, constant timer count')
        finally:
            executor.shutdown()
            for server in servers:
                server.destroy()
            for client in clients.values():
                client.destroy()
            stable.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
