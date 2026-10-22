"""Filter navigation scans at acquisition time; never replay or retime scans."""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException
from d1h_nav2.cloud_geometry import transform_xyz


class PointCloudRetime(Node):
    def __init__(self):
        super().__init__('navigation_cloud_filter')
        self.declare_parameter('max_age_sec', 0.5)
        # This course has a level floor at odom z=0. Threshold is measured
        # AFTER the full mounting/body transform, never in lidar coordinates.
        self.declare_parameter('ground_z_m', 0.0)
        self.declare_parameter('ground_tolerance_m', 0.03)
        # Standing robot collision envelope; high roofs do not block the floor.
        self.declare_parameter('max_obstacle_height_m', 0.9)
        self.buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar/pointcloud_nav', 5)
        self.clear_publisher = self.create_publisher(PointCloud2, '/livox/lidar/pointcloud_clear', 5)
        self.sub = self.create_subscription(PointCloud2, '/livox/lidar/pointcloud',
                                            self.on_cloud, qos_profile_sensor_data)
        self.pending = None
        self.timer = self.create_timer(0.02, self.process)

    def on_cloud(self, msg):
        self.pending = msg

    def process(self):
        msg = self.pending
        if msg is None:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age > self.get_parameter('max_age_sec').value or age < -0.1:
            self.pending = None
            self.get_logger().warning(f'Dropped delayed scan: age={age:.2f}s', throttle_duration_sec=5.0)
            return
        try:
            tf = self.buffer.lookup_transform('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        self.pending = None
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields and fields[name].datatype == PointField.FLOAT32 for name in ('x', 'y', 'z')):
            self.get_logger().error('Expected float32 x/y/z fields')
            return
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                          'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                          'itemsize': msg.point_step})
        raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
        xyz = np.stack([raw[n].ravel() for n in ('x', 'y', 'z')], axis=1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[np.linalg.norm(xyz, axis=1) > 0.6]
        t, q = tf.transform.translation, tf.transform.rotation
        xyz = transform_xyz(xyz, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        self.publish_xyz(self.clear_publisher, xyz.astype('<f4'), msg.header.stamp)
        # Reject ground in the level world frame, even during stand-up.
        ground = float(self.get_parameter('ground_z_m').value)
        tolerance = float(self.get_parameter('ground_tolerance_m').value)
        ceiling = float(self.get_parameter('max_obstacle_height_m').value)
        xyz = xyz[(xyz[:, 2] > ground+tolerance) & (xyz[:, 2] < ground+ceiling)].astype('<f4')
        self.publish_xyz(self.publisher, xyz, msg.header.stamp)

    def publish_xyz(self, publisher, xyz, stamp):
        out = PointCloud2()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.height, out.width = 1, len(xyz)
        out.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        out.point_step, out.row_step = 12, len(xyz)*12
        out.is_dense = True
        out.data = xyz.tobytes()
        publisher.publish(out)


def main():
    rclpy.init()
    node = PointCloudRetime()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
