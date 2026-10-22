#!/usr/bin/env python3
"""D1H GPS + IMU + local-odometry localization graph."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('d1h_global_localization')
    config = lambda filename: os.path.join(share, 'config', filename)
    fusion = IfCondition(PythonExpression(["'", LaunchConfiguration('localization_mode'), "' == 'gps_imu'"]))
    reference = IfCondition(PythonExpression(["'", LaunchConfiguration('localization_mode'), "' == 'gazebo_truth'"]))

    return LaunchDescription([
        DeclareLaunchArgument('localization_mode', default_value='gazebo_truth',
                              choices=['gazebo_truth', 'gps_imu']),
        Node(package='d1h_global_localization', executable='gazebo_localization',
             output='screen', condition=reference, parameters=[{'use_sim_time': True}]),
        DeclareLaunchArgument(
            'local_odom_topic', default_value='/odometry/gazebo',
            description='Local continuous odometry; use /odometry/livo after Fast-LIVO2 starts.'),
        Node(package='d1h_global_localization', executable='imu_relay', name='imu_relay', output='screen',
             parameters=[{'use_sim_time': True, 'output_frame_id': 'imu_link'}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_local', output='screen', condition=fusion,
             parameters=[config('ekf_local.yaml'), {'use_sim_time': True,
                                                     'odom0': LaunchConfiguration('local_odom_topic')}],
             # navsat_transform and ekf_global consume the named local stream.
             # Without this remap ekf_local publishes its default
             # /odometry/filtered topic, leaving the downstream graph idle.
             remappings=[('odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform', output='screen', condition=fusion,
             parameters=[config('navsat.yaml'), {'use_sim_time': True}],
             remappings=[('imu/data', '/imu/data'), ('gps/fix', '/gps/fix'),
                         ('odometry/filtered', '/odometry/local'), ('odometry/gps', '/odometry/gps')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global', output='screen', condition=fusion,
             parameters=[config('ekf_global.yaml'), {'use_sim_time': True}],
             remappings=[('odometry/filtered', '/odometry/global')]),
    ])
