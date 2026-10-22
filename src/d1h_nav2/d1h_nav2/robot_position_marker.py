"""Publish a small, unambiguous green marker at the navigation base frame."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


class RobotPositionMarker(Node):
    def __init__(self) -> None:
        super().__init__('robot_position_marker')
        self._publisher = self.create_publisher(Marker, '/navigation/robot_position', 1)
        self._subscription = self.create_subscription(Odometry, '/odometry/global', self._publish, 5)

    def _publish(self, odom) -> None:
        marker = Marker()
        # Show the navigation XY position on the ground, independently of
        # the physical chassis height during stand-up.
        marker.header.frame_id = 'map'
        marker.header.stamp = odom.header.stamp
        marker.ns = 'd1h_navigation'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = odom.pose.pose.position.x
        marker.pose.position.y = odom.pose.pose.position.y
        marker.pose.position.z = 0.19
        marker.scale.x = 0.38
        marker.scale.y = 0.38
        marker.scale.z = 0.38
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0
        self._publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = RobotPositionMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
