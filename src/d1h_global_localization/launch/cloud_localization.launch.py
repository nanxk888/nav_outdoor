from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments=[DeclareLaunchArgument('map_path'),
               DeclareLaunchArgument('use_sim_time',default_value='true')]
    values={'map_path':LaunchConfiguration('map_path'),
            'use_sim_time':LaunchConfiguration('use_sim_time')}
    for key,default in dict(initial_x='0.0',initial_y='0.0',initial_z='0.15',
                            initial_roll='0.0',initial_pitch='0.0',initial_yaw='0.0').items():
        arguments.append(DeclareLaunchArgument(key,default_value=default))
        values[key]=LaunchConfiguration(key)
    return LaunchDescription(arguments+[Node(package='d1h_global_localization',
        executable='cloud_localization',output='screen',parameters=[values])])
