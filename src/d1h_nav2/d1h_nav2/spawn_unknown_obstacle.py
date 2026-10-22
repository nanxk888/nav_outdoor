"""Add a red Gazebo-only obstacle; it is never published in the prior map."""
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity


def main():
    rclpy.init()
    node = Node('spawn_unknown_obstacle')
    for key, value in (('x', 5.0), ('y', -1.8), ('size_x', 0.8),
                       ('size_y', 0.8), ('height', 1.0)):
        node.declare_parameter(key, value)
    x, y, sx, sy, sz = (float(node.get_parameter(k).value)
                        for k in ('x', 'y', 'size_x', 'size_y', 'height'))
    if min(sx, sy, sz) <= 0:
        raise ValueError('Obstacle dimensions must be positive')
    request = SpawnEntity.Request()
    request.name = f'unknown_red_{time.time_ns()}'
    request.reference_frame = 'world'
    request.initial_pose.position.x, request.initial_pose.position.y = x, y
    request.initial_pose.position.z = sz/2
    request.initial_pose.orientation.w = 1.0
    request.xml = f'''<sdf version="1.6"><model name="{request.name}"><static>true</static>
      <link name="link"><collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size>
      </box></geometry></collision><visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size>
      </box></geometry><material><ambient>0.9 0.08 0.05 1</ambient>
      <diffuse>0.9 0.08 0.05 1</diffuse></material></visual></link></model></sdf>'''
    client = node.create_client(SpawnEntity, '/spawn_entity')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Gazebo /spawn_entity is unavailable')
        node.destroy_node()
        rclpy.shutdown()
        return
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    response = future.result() if future.done() else None
    if response and response.success:
        node.get_logger().info(f'Red unknown obstacle at ({x}, {y}); prior /map unchanged')
    else:
        node.get_logger().error(response.status_message if response else 'Spawn timed out')
    node.destroy_node()
    rclpy.shutdown()
