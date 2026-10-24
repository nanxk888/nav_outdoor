"""Real GNSS-only stage. Deliberately no simulated localization or TF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('d1h_global_localization')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','real_lc29h_gnss.yaml')),
        DeclareLaunchArgument('log_level',default_value='error'),
        Node(package='d1h_global_localization',executable='real_gnss',name='real_gnss_gate',
             output='screen',arguments=['--ros-args','--log-level',LaunchConfiguration('log_level')],
             parameters=[LaunchConfiguration('params_file'),{'use_sim_time':False}]),
    ])
