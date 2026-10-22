"""Bound Nav2 velocity commands before sending them to the D1H controller."""

from __future__ import annotations

import time
import math
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from d1h_nav2.station_keep import StationKeep


class NavCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__('nav_cmd_bridge')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/command/cmd_twist')
        self.declare_parameter('max_forward_mps', 0.50)
        self.declare_parameter('max_reverse_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.70)
        self.declare_parameter('timeout_sec', 0.40)
        self.declare_parameter('station_keep_enabled', True)
        self.declare_parameter('require_localization', False)
        self._localization_ok = False
        self._localization_rx = -1e9
        self._localization_sub = self.create_subscription(String, '/localization/status', self._on_localization, 5)

        self._max_forward = float(self.get_parameter('max_forward_mps').value)
        self._max_reverse = float(self.get_parameter('max_reverse_mps').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rps').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._last_rx = time.monotonic()
        self._latest = Twist()
        self._cloud_stamp = None
        self._odom = None
        self._hold_mode = False
        self._hold = StationKeep()
        self._odom_sub = self.create_subscription(Odometry, '/odometry/local', self._on_odom, 5)
        key_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._key_sub = self.create_subscription(String, '/command/cmd_key', self._on_key, key_qos)
        self._cloud_sub = self.create_subscription(
            PointCloud2, '/livox/lidar/pointcloud_nav', self._on_cloud, 5)
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10)
        self._subscription = self.create_subscription(
            Twist, str(self.get_parameter('input_topic').value), self._on_command, 10)
        self._timer = self.create_timer(0.05, self._publish_safe_command)
        self.get_logger().info(
            f'Bridging {self.get_parameter("input_topic").value} to '
            f'{self.get_parameter("output_topic").value}; timeout={self._timeout:.2f}s')

    def _on_command(self, message: Twist) -> None:
        self._latest.linear.x = min(self._max_forward, max(-self._max_reverse, message.linear.x))
        # First navigation profile is non-holonomic; lateral commands are deliberately rejected.
        self._latest.linear.y = 0.0
        self._latest.linear.z = 0.0
        self._latest.angular.x = 0.0
        self._latest.angular.y = 0.0
        self._latest.angular.z = min(self._max_yaw, max(-self._max_yaw, message.angular.z))
        self._last_rx = time.monotonic()

    def _on_localization(self, msg):
        try:
            status = json.loads(msg.data)
            self._localization_ok = status.get('state') == 'TRACKING_PROTOTYPE' and status.get('accepted') is True
        except (ValueError, TypeError):
            self._localization_ok = False
        self._localization_rx = time.monotonic()

    def _on_cloud(self, msg):
        self._cloud_stamp = Time.from_msg(msg.header.stamp)

    def _on_odom(self, msg):
        self._odom = msg

    def _on_key(self, msg):
        self._hold_mode = msg.data == 'rl_0'
        self._hold.reset()

    def _publish_safe_command(self) -> None:
        cloud_age = ((self.get_clock().now() - self._cloud_stamp).nanoseconds / 1e9
                     if self._cloud_stamp is not None else float('inf'))
        fresh_command = time.monotonic() - self._last_rx <= self._timeout
        moving = fresh_command and (abs(self._latest.linear.x) > 0.001 or abs(self._latest.angular.z) > 0.001)
        localization_safe = (not self.get_parameter('require_localization').value or (
            self._localization_ok and time.monotonic()-self._localization_rx <= 1.0 and
            self._odom is not None and -0.1 <= (self.get_clock().now()-Time.from_msg(
                self._odom.header.stamp)).nanoseconds/1e9 <= .35))
        if not localization_safe or not -0.1 <= cloud_age <= 0.75:
            self._hold.reset()
            self._publisher.publish(Twist())
        elif moving:
            self._hold.reset()
            self._publisher.publish(self._latest)
        else:
            output = Twist()
            odom = self._odom
            if self._hold_mode and self.get_parameter('station_keep_enabled').value and odom is not None:
                age = (self.get_clock().now()-Time.from_msg(odom.header.stamp)).nanoseconds/1e9
                q = odom.pose.pose.orientation
                roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
                pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if -0.1 <= age <= 0.3 and abs(roll) < 0.35 and abs(pitch) < 0.35:
                    p, v = odom.pose.pose.position, odom.twist.twist
                    output.linear.x, output.angular.z = self._hold.command(
                        p.x, p.y, yaw, v.linear.x, v.angular.z, 0.05)
                else:
                    self._hold.reset()
            output.linear.x = min(self._max_forward, max(-self._max_reverse, output.linear.x))
            output.angular.z = min(self._max_yaw, max(-self._max_yaw, output.angular.z))
            self._publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = NavCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
