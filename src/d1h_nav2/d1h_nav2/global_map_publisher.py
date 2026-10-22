"""Publish the initial 1 km x 1 km ENU global occupancy map.

The generated map is deliberately a georeferenced baseline: it establishes a
common 0.25 m global planning grid immediately. Replace it with a surveyed GIS
occupancy map before outdoor deployment without changing any Nav2 frames.
"""

from __future__ import annotations

import rclpy
import os
from array import array
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from d1h_nav2.scenario import known_boxes, rasterize_boxes
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GlobalMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__('global_map_publisher')
        self.declare_parameter('size_m', 1000)
        self.declare_parameter('resolution_m', 0.25)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_clearance_m', 0.9)
        self.declare_parameter('world_file', os.path.join(
            get_package_share_directory('d1h_navigation_description'),
            'worlds', 'navigation_empty.world'))
        size_m = int(self.get_parameter('size_m').value)
        resolution = float(self.get_parameter('resolution_m').value)
        width = int(size_m / resolution)
        if width <= 0 or width > 4096:
            raise ValueError('size_m / resolution_m must be in 1..4096')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self._publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self._markers = self.create_publisher(MarkerArray, '/navigation/known_obstacles', qos)
        self._map = OccupancyGrid()
        self._map.header.frame_id = str(self.get_parameter('frame_id').value)
        self._map.info.resolution = resolution
        self._map.info.width = width
        self._map.info.height = width
        self._map.info.origin.position.x = -size_m / 2.0
        self._map.info.origin.position.y = -size_m / 2.0
        self._map.info.origin.orientation.w = 1.0
        data = array('b', [0]) * (width * width)
        # A one-cell occupied perimeter guarantees planners remain inside the survey area.
        for x in range(width):
            data[x] = 100
            data[(width - 1) * width + x] = 100
        for y in range(width):
            data[y * width] = 100
            data[y * width + width - 1] = 100

        boxes = known_boxes(self.get_parameter('world_file').value)
        rasterize_boxes(data, width, resolution, self._map.info.origin.position.x,
                        self._map.info.origin.position.y, boxes,
                        float(self.get_parameter('robot_clearance_m').value))
        self._known = MarkerArray()
        import math
        for index, (name, x, y, z, sx, sy, sz, yaw) in enumerate(boxes):
            marker = Marker()
            marker.header.frame_id = self._map.header.frame_id
            marker.ns = 'known_obstacles'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
            marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
            marker.color.g, marker.color.r, marker.color.a = 0.8, 0.1, 0.8
            self._known.markers.append(marker)
        self._map.data = data
        # The prior is static. Transient-local QoS serves late subscribers;
        # do not serialize a 16-million-cell map every second.
        self._publish()
        self.get_logger().info(
            f'Published {size_m} m x {size_m} m ENU map at {resolution:.2f} m/cell')

    def _publish(self) -> None:
        self._map.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._map)
        self._markers.publish(self._known)


def main() -> None:
    rclpy.init()
    node = GlobalMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
