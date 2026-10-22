"""Convert WGS84 NavSatFix goals to the map frame's local ENU approximation."""

from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsGoalToEnu(Node):
    def __init__(self) -> None:
        super().__init__('gps_goal_to_enu')
        # Match navigation_empty.world spherical_coordinates.  This makes a
        # GPS goal resolve to the same local ENU coordinates as Gazebo.
        self.declare_parameter('datum_latitude_deg', 31.2304)
        self.declare_parameter('datum_longitude_deg', 121.4737)
        self.declare_parameter('input_topic', '/goal_gps/fix')
        self.declare_parameter('output_topic', '/goal_pose')
        self._lat0 = math.radians(float(self.get_parameter('datum_latitude_deg').value))
        self._lon0 = math.radians(float(self.get_parameter('datum_longitude_deg').value))
        self._publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10)
        self._navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._subscription = self.create_subscription(
            NavSatFix, str(self.get_parameter('input_topic').value), self._on_goal, 10)

    def _on_goal(self, fix: NavSatFix) -> None:
        if math.isnan(fix.latitude) or math.isnan(fix.longitude):
            self.get_logger().warning('Ignoring GPS goal with invalid latitude or longitude')
            return
        radius_m = 6378137.0
        lat = math.radians(fix.latitude)
        lon = math.radians(fix.longitude)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = radius_m * (lon - self._lon0) * math.cos(self._lat0)
        goal.pose.position.y = radius_m * (lat - self._lat0)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self._publisher.publish(goal)
        self.get_logger().info(f'GPS goal -> ENU x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f} m')
        if not self._navigator.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 NavigateToPose action is not available; goal_pose was still published')
            return
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal
        self._navigator.send_goal_async(action_goal)


def main() -> None:
    rclpy.init()
    node = GpsGoalToEnu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
