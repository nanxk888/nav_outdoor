"""Dedicated sensor-only odometry; no driver, camera, TF or actuator launch."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('d1h_fast_lio')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','navigation_lio.yaml')),
        Node(package='d1h_fast_lio',executable='lio_odometry',name='navigation_lio',output='screen',
             respawn=False,parameters=[LaunchConfiguration('params_file'),{'use_sim_time':False}]),
    ])
