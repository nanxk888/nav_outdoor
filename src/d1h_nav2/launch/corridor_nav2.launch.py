"""Low-speed simulation experiment: ICP-only positioning, existing Nav2 stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('d1h_nav2')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        Node(package='d1h_global_localization',executable='icp_navigation_bridge',output='screen',
             parameters=[{'use_sim_time':True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','d1h_nav2.launch.py')),
            launch_arguments={'use_external_map':'true','map_yaml':LaunchConfiguration('map_yaml'),
                              'max_forward_mps':'0.15','max_reverse_mps':'0.10','max_yaw_rps':'0.35',
                              'require_localization':'true','station_keep_enabled':'true',
                              'ground_tolerance_m':'0.08','memory_resolution_m':'0.1',
                              'rotate_to_heading_min_angle':'0.785398',
                              'movement_time_allowance':'15.0'}.items())])
