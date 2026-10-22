"""Promote stationary lidar clusters into a bounded, expiring global overlay."""
from array import array
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from d1h_nav2.obstacle_memory import ObstacleMemory, cluster_points
from d1h_nav2.cloud_geometry import transform_xyz


class ObstacleMemoryNode(Node):
    def __init__(self):
        super().__init__('obstacle_memory')
        defaults = dict(remember_all_obstacles=True, stable_sec=0., retention_sec=120., resolution_m=.25,
                        max_gap_sec=.6, motion_tolerance_m=.15, max_tracks=256,
                        max_cells=100000, max_range_m=5., max_scan_age_sec=.5)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.memory = ObstacleMemory(p('resolution_m'), p('stable_sec'), p('retention_sec'),
                                     p('max_gap_sec'), p('motion_tolerance_m'),
                                     p('max_tracks'), p('max_cells'), p('remember_all_obstacles'))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/navigation/global_memory_map', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/navigation/remembered_obstacles', 1)
        self.status_pub = self.create_publisher(String, '/navigation/obstacle_memory_status', 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.sub = self.create_subscription(PointCloud2, '/livox/lidar/pointcloud_nav',
                                            self.on_cloud, qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.prior, self.fused, self.pending = None, None, None
        self.previous, self.last_publish = set(), -float('inf')
        self.map_dirty = False
        self.last_scan = None
        self.timer = self.create_timer(.1, self.tick)
        mode = 'all observed obstacles immediately' if self.memory.remember_all else f'stationary >={self.memory.stable_sec}s'
        self.get_logger().info(f'Memory: {mode} -> global overlay; '
                               f'expiry {self.memory.retention_sec}s since confirmation')

    def on_map(self, msg):
        if abs(msg.info.resolution-self.memory.resolution) > 1e-6:
            self.get_logger().error('Prior and memory resolutions must match; refusing overlay')
            return
        self.prior = msg
        self.fused = OccupancyGrid()
        self.fused.header = copy.deepcopy(msg.header)
        self.fused.info = copy.deepcopy(msg.info)
        self.fused.data = array('b', msg.data)
        self.previous = set()
        self.map_dirty = True
        self.publish(self.get_clock().now().nanoseconds/1e9)

    def on_cloud(self, msg):
        self.pending = msg

    def process(self, now):
        msg, self.pending = self.pending, None
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = now-stamp.nanoseconds/1e9
        if not -.1 <= age <= self.get_parameter('max_scan_age_sec').value:
            return
        if self.last_scan is not None and stamp.nanoseconds <= self.last_scan:
            return  # Replayed/duplicate scans must never earn stationary time.
        try:
            tf = self.buffer.lookup_transform('map', msg.header.frame_id, stamp)
            sensor = self.buffer.lookup_transform('map', 'lidar_link', stamp)
        except TransformException:
            return  # Missing transforms cannot qualify an observation.
        fields = {f.name: f for f in msg.fields}
        if not all(n in fields and fields[n].datatype == PointField.FLOAT32 for n in ('x', 'y', 'z')):
            return
        dtype = np.dtype(dict(names=['x', 'y', 'z'],
                              formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                              offsets=[fields[n].offset for n in ('x', 'y', 'z')],
                              itemsize=msg.point_step))
        raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
        xyz = np.stack([raw[n].ravel() for n in ('x', 'y', 'z')], axis=1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        t, q = tf.transform.translation, tf.transform.rotation
        xyz = transform_xyz(xyz, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        s = sensor.transform.translation
        xyz = xyz[np.linalg.norm(xyz[:, :2]-[s.x, s.y], axis=1) <= self.get_parameter('max_range_m').value]
        self.last_scan = stamp.nanoseconds
        if self.memory.remember_all:
            cells = set(map(tuple, np.floor(xyz[:, :2]/self.memory.resolution).astype(np.int64)))
            self.memory.update([((0., 0.), cells)], now)
        else:
            self.memory.update(cluster_points(xyz[:, :2], self.memory.resolution,
                                              self.memory.max_cells), now)

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self.last_publish:
            self.last_scan = None
        self.process(now)
        self.memory.expire(now)
        # Decay continues even with no new scans. Latest-only subscriptions,
        # bounded tracks/cells, and depth-one maps bound retained memory.
        if now < self.last_publish or now-self.last_publish >= 1.:
            self.publish(now)

    def publish(self, now):
        cells = self.memory.occupied(now)
        if self.fused is not None:
            grid = self.fused.info
            def index(cell):
                x = round(cell[0]-grid.origin.position.x/grid.resolution)
                y = round(cell[1]-grid.origin.position.y/grid.resolution)
                return y*grid.width+x if 0 <= x < grid.width and 0 <= y < grid.height else None
            for cell in self.previous-cells:
                i = index(cell)
                if i is not None:
                    self.fused.data[i] = self.prior.data[i]
            for cell in cells-self.previous:
                i = index(cell)
                if i is not None:
                    self.fused.data[i] = 100
            if cells != self.previous or self.map_dirty:
                self.fused.header.stamp = self.get_clock().now().to_msg()
                self.map_pub.publish(self.fused)
                self.map_dirty = False
        self.previous = cells
        points = np.asarray([((x+.5)*self.memory.resolution,
                              (y+.5)*self.memory.resolution, .15) for x, y in cells], dtype='<f4')
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'map', self.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, len(cells)
        cloud.fields = [PointField(name=n, offset=i*4, datatype=7, count=1) for i, n in enumerate(('x', 'y', 'z'))]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, len(cells)*12, True
        cloud.data = points.tobytes()
        self.cloud_pub.publish(cloud)
        self.status_pub.publish(String(data=f'tracks={len(self.memory.tracks)} remembered_cells={len(cells)} '
                                      f'cap={self.memory.max_cells} stable_sec={self.memory.stable_sec} '
                                      f'remember_all={self.memory.remember_all} '
                                      f'retention_sec={self.memory.retention_sec}'))
        self.last_publish = now


def main():
    rclpy.init()
    node = ObstacleMemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
