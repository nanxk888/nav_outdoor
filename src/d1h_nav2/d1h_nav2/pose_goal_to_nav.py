"""Forward RViz's /goal_pose topic to Nav2's NavigateToPose action."""

from __future__ import annotations

import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class PoseGoalToNav(Node):
    def __init__(self) -> None:
        super().__init__('pose_goal_to_nav')
        self.declare_parameter('input_topic', '/goal_pose')
        self._navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._subscription = self.create_subscription(
            PoseStamped, str(self.get_parameter('input_topic').value), self._on_goal, 10)

    def _on_goal(self, pose: PoseStamped) -> None:
        if not self._navigator.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 NavigateToPose action is unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = os.path.join(get_package_share_directory('d1h_nav2'),
                                          'config', 'navigate_rolling_5m.xml')
        self._navigator.send_goal_async(goal)
        self.get_logger().info(
            f'Forwarded RViz goal x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')


def main() -> None:
    rclpy.init()
    node = PoseGoalToNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
