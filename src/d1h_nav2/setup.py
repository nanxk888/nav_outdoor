from glob import glob
from setuptools import setup

package_name = 'd1h_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'osm_road_planner = d1h_nav2.osm_road_planner:main',
        'nav_cmd_bridge = d1h_nav2.nav_cmd_bridge:main',
        'obstacle_memory = d1h_nav2.obstacle_memory_node:main',
        'stable_planner = d1h_nav2.stable_planner:main',
        'gps_goal_to_enu = d1h_nav2.gps_goal_to_enu:main',
        'global_map_publisher = d1h_nav2.global_map_publisher:main',
        'pointcloud_retime = d1h_nav2.pointcloud_retime:main',
        'real_cloud_filter = d1h_nav2.real_cloud_filter:main',
        'lio_memory = d1h_nav2.lio_memory_node:main',
        'navigation_reference = d1h_nav2.navigation_reference_node:main',
        'pose_goal_to_nav = d1h_nav2.pose_goal_to_nav:main',
        'robot_position_marker = d1h_nav2.robot_position_marker:main',
        'spawn_unknown_obstacle = d1h_nav2.spawn_unknown_obstacle:main',
    ]},
)
