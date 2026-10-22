#!/usr/bin/env python3
"""Launch D1H global Nav2 planning, local Livox avoidance and safety bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('d1h_nav2')
    nav2_share = get_package_share_directory('nav2_bringup')
    params = os.path.join(share, 'config', 'nav2_d1h.yaml')
    params = RewrittenYaml(source_file=params, param_rewrites={
        'default_nav_to_pose_bt_xml': os.path.join(share, 'config', 'navigate_rolling_5m.xml'),
        'desired_linear_vel': LaunchConfiguration('max_forward_mps'),
        'rotate_to_heading_min_angle':LaunchConfiguration('rotate_to_heading_min_angle'),
        'movement_time_allowance':LaunchConfiguration('movement_time_allowance'),
        'rotate_to_heading_angular_vel': LaunchConfiguration('max_yaw_rps')},
        convert_types=True)
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_external_map', default_value='false'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('max_forward_mps', default_value='0.5'),
        DeclareLaunchArgument('max_reverse_mps', default_value='0.25'),
        DeclareLaunchArgument('max_yaw_rps', default_value='0.7'),
        DeclareLaunchArgument('require_localization', default_value='false'),
        DeclareLaunchArgument('station_keep_enabled', default_value='true'),
        DeclareLaunchArgument('ground_tolerance_m', default_value='0.03'),
        DeclareLaunchArgument('memory_resolution_m', default_value='0.25'),
        DeclareLaunchArgument('rotate_to_heading_min_angle', default_value='2.094395'),
        DeclareLaunchArgument('movement_time_allowance', default_value='8.0'),
        Node(package='d1h_nav2', executable='global_map_publisher', output='screen',
             condition=UnlessCondition(LaunchConfiguration('use_external_map')),
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             condition=IfCondition(LaunchConfiguration('use_external_map')),
             parameters=[{'use_sim_time':use_sim_time,'yaml_filename':LaunchConfiguration('map_yaml')}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='map_lifecycle_manager',
             condition=IfCondition(LaunchConfiguration('use_external_map')),
             parameters=[{'use_sim_time':use_sim_time,'autostart':True,'node_names':['map_server']}]),
        navigation,
        Node(package='d1h_nav2', executable='stable_planner', output='screen',
             parameters=[os.path.join(share, 'config', 'stable_planner.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='d1h_nav2', executable='obstacle_memory', output='screen',
             parameters=[os.path.join(share, 'config', 'obstacle_memory.yaml'),
                         {'use_sim_time': use_sim_time,
                          'resolution_m':LaunchConfiguration('memory_resolution_m')}]),
        Node(package='nav2_planner', executable='planner_server', namespace='local_planner',
             name='planner_server', output='screen',
             parameters=[os.path.join(share, 'config', 'local_planner_5m.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='local_planner', name='lifecycle_manager', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['planner_server']}]),
        Node(package='d1h_nav2', executable='pointcloud_retime', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'ground_tolerance_m':LaunchConfiguration('ground_tolerance_m')}]),
        Node(package='d1h_nav2', executable='nav_cmd_bridge', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'max_forward_mps':LaunchConfiguration('max_forward_mps'),
                          'max_reverse_mps':LaunchConfiguration('max_reverse_mps'),
                          'max_yaw_rps':LaunchConfiguration('max_yaw_rps'),
                          'require_localization':LaunchConfiguration('require_localization'),
                          'station_keep_enabled':LaunchConfiguration('station_keep_enabled')}]),
        Node(package='d1h_nav2', executable='gps_goal_to_enu', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='d1h_nav2', executable='pose_goal_to_nav', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='d1h_nav2', executable='robot_position_marker', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
