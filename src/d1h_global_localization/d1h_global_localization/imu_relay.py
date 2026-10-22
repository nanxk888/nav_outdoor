"""Republish the DDT controller IMU under the navigation-standard /imu/data topic."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        self.declare_parameter('input_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('output_topic', '/imu/data')
        # Only set when the sensor's actual mounting frame is known.  DDT's
        # Gazebo broadcaster labels its imu_link data as the sensor name
        # trunk_imu, which is not a URDF frame.
        self.declare_parameter('output_frame_id', '')
        self.output_frame_id = self.get_parameter('output_frame_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(Imu, output_topic, 20)
        self.subscription = self.create_subscription(Imu, input_topic, self.callback, 20)
        self.get_logger().info(f'Relaying {input_topic} to {output_topic}')

    def callback(self, message: Imu):
        if self.output_frame_id:
            message.header.frame_id = self.output_frame_id
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = ImuRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
