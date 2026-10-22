#!/usr/bin/env python3
"""ROS actions: failed replacement -> safe prefix -> hold -> atomic commit."""
import copy
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped, TransformStamped
from d1h_nav2.stable_planner import StablePlanner


class HandoverTest(unittest.TestCase):
    def test_safe_prefix_and_atomic_switch(self):
        rclpy.init()
        stable, probe = StablePlanner(), Node('handover_probe')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(stable)
        executor.add_node(probe)
        position, obstacle, failing, bypass = [0.], [None], set(), [False]
        servers, services = [], []
        def pose(x):
            position[0] = x
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
            tf.transform.translation.x, tf.transform.rotation.w = x, 1.
            stable.buffer.set_transform_static(tf, 'test')
        pose(0.)
        def planner(kind):
            async def callback(handle):
                result = ComputePathToPose.Result()
                if kind in failing:
                    handle.abort()
                    return result
                if kind == 'local' and bypass[0]:
                    deadline = time.monotonic()+.3
                    await stable.wait_ready(lambda: time.monotonic() >= deadline, 1.)
                    vertices = [(position[0], 0.), (position[0], .8),
                                (handle.request.goal.pose.position.x, .8),
                                (handle.request.goal.pose.position.x, 0.)]
                else:
                    vertices = [(position[0]+i*(handle.request.goal.pose.position.x-position[0])/40., 0.)
                                for i in range(41)]
                result.path.header.frame_id = 'map'
                result.path.header.stamp = probe.get_clock().now().to_msg()
                for x, y in vertices:
                    p = PoseStamped()
                    p.header = result.path.header
                    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.
                    result.path.poses.append(p)
                handle.succeed()
                return result
            return callback
        def validate(request, response):
            response.is_valid = bool(request.path.poses) and not any(
                obstacle[0] is not None and obstacle[0] <= p.pose.position.x <= obstacle[0]+.4
                and abs(p.pose.position.y) < .35 for p in request.path.poses)
            return response
        clients = {}
        for kind, prefix in [('global', ''), ('local', '/local_planner')]:
            servers.append(ActionServer(probe, ComputePathToPose, prefix+'/compute_path_to_pose', planner(kind)))
            services.append(probe.create_service(IsPathValid, prefix+'/is_path_valid', validate))
            clients[kind] = ActionClient(probe, ComputePathToPose, '/navigation/compute_'+kind+'_path')
        def wait(future):
            deadline = time.monotonic()+6.
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            self.assertTrue(future.done())
            return future.result()
        def send(kind, x):
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x, goal.goal.pose.orientation.w = x, 1.
            goal.planner_id = 'GridBased' if kind == 'global' else 'LocalGrid'
            return wait(clients[kind].send_goal_async(goal)).get_result_async()
        try:
            for client in clients.values():
                self.assertTrue(client.wait_for_server(timeout_sec=3.))
            self.assertEqual(wait(send('global', 12.)).status, 4)
            self.assertEqual(wait(send('local', 5.)).status, 4)
            source = copy.deepcopy(stable._local_source)
            obstacle[0] = 4.
            failing.add('local')
            pose(.5)
            prefix = wait(send('local', 6.))
            self.assertEqual(prefix.status, 4, 'A failed candidate canceled a usable safe prefix')
            self.assertLess(prefix.result.path.poses[-1].pose.position.x, 3.4)
            self.assertGreater(prefix.result.path.poses[-1].pose.position.x, 2.)
            self.assertEqual(stable._local_source, source)
            self.assertTrue(stable._prefix_active)
            failing.add('global')
            self.assertEqual(wait(send('global', 12.)).status, 4,
                             'Global update failure interrupted safe local continuation')
            pose(3.6)
            hold = wait(send('local', 6.))
            self.assertEqual(hold.status, 4)
            self.assertEqual(len(hold.result.path.poses), 1)
            self.assertAlmostEqual(hold.result.path.poses[0].pose.position.x, 3.6)
            old_front = copy.deepcopy(stable.cache['local']['result'].path)
            failing.clear()
            bypass[0] = True
            replacement = send('local', 6.)
            for _ in range(5):
                executor.spin_once(timeout_sec=.01)
            self.assertEqual(stable.cache['local']['result'].path, old_front,
                             'Front buffer was cleared while candidate was being computed')
            self.assertEqual(wait(replacement).status, 4)
            self.assertFalse(stable._prefix_active)
            self.assertTrue(any(p.pose.position.y > .7 for p in stable.cache['local']['result'].path.poses))
            failing.add('local')
            stable._handover_since = stable.get_clock().now().nanoseconds/1e9-7.
            stable._handover_anchor = (position[0], 0.)
            deadline = time.monotonic()+.25
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            self.assertEqual(wait(send('local', 7.)).status, 6,
                             'Persistent failures must not continue an old route forever')
            print('HANDOVER PASS: checked safe prefix, braking margin, verified hold, atomic detour switch, bounded failure')
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
