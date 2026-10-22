"""Simulation reference localization, preserving the physical sensor attitude.

map and odom coincide with Gazebo world.  Do not flatten base_link: doing so
loses the sensor height/roll/pitch during stand-up and projects ground returns
into the obstacle layer. Nav2 plans in XY using this full physical pose.
"""
import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class GazeboLocalization(Node):
    def __init__(self):
        super().__init__('gazebo_reference_localization')
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.frame_id = 'map'
        fixed.child_frame_id = 'odom'
        fixed.transform.rotation.w = 1.0
        self.static_tf.sendTransform(fixed)
        self.local = self.create_publisher(Odometry, '/odometry/local', 5)
        self.global_pub = self.create_publisher(Odometry, '/odometry/global', 5)
        self.sub = self.create_subscription(
            Odometry, '/odometry/gazebo', self.on_odom, qos_profile_sensor_data)
        self._received = False
        self.get_logger().info('SIMULATION REFERENCE: map = odom = Gazebo world; GPS/IMU EKFs disabled')

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().error('Expected Gazebo odometry odom -> base_link')
            return
        if not self._received:
            self._received = True
            self.get_logger().info('Reference READY: receiving Gazebo odometry and publishing map/odom/base_link TF')
        tf = TransformStamped()
        tf.header = copy.deepcopy(msg.header)
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = copy.deepcopy(msg.pose.pose.orientation)
        self.tf.sendTransform(tf)
        self.local.publish(msg)
        global_msg = copy.deepcopy(msg)
        global_msg.header.frame_id = 'map'
        self.global_pub.publish(global_msg)


def main():
    rclpy.init()
    node = GazeboLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
