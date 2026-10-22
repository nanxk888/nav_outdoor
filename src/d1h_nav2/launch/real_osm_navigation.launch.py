"""Real-robot planning ONLY. Hardware/localization/filtering are external prerequisites.

No simulator nodes, fake localization, or actuator bridge are launched here.
"""
import json
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def configure(context):
    profile=os.path.abspath(LaunchConfiguration('profile').perform(context))
    with open(profile,encoding='utf-8') as f:p=json.load(f)
    if not p.get('survey_confirmed') or not p.get('approved_way_ids'):
        raise RuntimeError('Confirm datum, surveyed minimum clear width and approved OSM way IDs in profile first')
    if not os.path.isabs(p['osm_file']) or not os.path.isfile(p['osm_file']):
        raise RuntimeError('profile.osm_file must be an existing absolute Linux path')
    share=get_package_share_directory('d1h_nav2')
    nav=get_package_share_directory('nav2_bringup')
    with open(os.path.join(share,'config','nav2_d1h.yaml')) as f:real_nav=yaml.safe_load(f)
    with open(os.path.join(share,'config','local_planner_5m.yaml')) as f:local=yaml.safe_load(f)
    local['/**']['ros__parameters']['use_sim_time']=False
    costmap=local['local_planner']['global_costmap']['global_costmap']['ros__parameters']
    costmap['plugins']=['road_boundary','obstacle_layer','inflation_layer']
    costmap['road_boundary']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/map',
                             'map_subscribe_transient_local':True,'subscribe_to_updates':False}
    controller_map=real_nav['local_costmap']['local_costmap']['ros__parameters']
    controller_map['plugins']=['road_boundary']+controller_map['plugins']
    controller_map['road_boundary']=dict(costmap['road_boundary'])
    # A multi-node parameter document must be passed as a FILE, not as one
    # node's parameter dictionary (the costmap is a separate child node).
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='real_osm_local_',delete=False) as f:
        yaml.safe_dump(local,f);local_file=f.name
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='real_osm_nav_',delete=False) as f:
        yaml.safe_dump(real_nav,f);nav_file=f.name
    params=RewrittenYaml(source_file=nav_file,
        param_rewrites={'use_sim_time':False,'default_nav_to_pose_bt_xml':os.path.join(share,'config','navigate_rolling_5m.xml'),
                        'desired_linear_vel':.2},convert_types=True)
    def cleanup(_context):
        for filename in (local_file,nav_file):
            if os.path.isfile(filename):os.unlink(filename)
        return []
    return [GroupAction([
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        # Never send unvalidated deployment-profile commands to the hardware.
        SetRemap(src='/cmd_vel',dst='/real_navigation/cmd_vel_candidate'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav,'launch','navigation_launch.py')),
            launch_arguments={'use_sim_time':'false','params_file':params,'autostart':'true','use_composition':'False'}.items()),
        Node(package='d1h_nav2',executable='osm_road_planner',output='screen',parameters=[{'profile':profile,'use_sim_time':False}]),
        Node(package='d1h_nav2',executable='stable_planner',output='screen',
             parameters=[os.path.join(share,'config','stable_planner.yaml'),{'use_sim_time':False}],
             remappings=[('/compute_path_to_pose','/roads/compute_path_to_pose'),('/is_path_valid','/roads/is_path_valid')]),
        Node(package='d1h_nav2',executable='obstacle_memory',output='screen',
             parameters=[os.path.join(share,'config','obstacle_memory.yaml'),{'use_sim_time':False,'resolution_m':.5}]),
        Node(package='nav2_planner',executable='planner_server',namespace='local_planner',name='planner_server',
             output='screen',parameters=[local_file]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='local_planner',
             parameters=[{'use_sim_time':False,'autostart':True,'node_names':['planner_server']}]),
        Node(package='d1h_nav2',executable='gps_goal_to_enu',parameters=[{'use_sim_time':False,
             'datum_latitude_deg':p['datum_latitude_deg'],'datum_longitude_deg':p['datum_longitude_deg'],
             'output_topic':'/real_navigation/gps_goal_preview'}]),
        Node(package='d1h_nav2',executable='pose_goal_to_nav',parameters=[{'use_sim_time':False}]),
        Node(package='d1h_nav2',executable='robot_position_marker',parameters=[{'use_sim_time':False}]),
    ])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('profile',description='Absolute path to reviewed real OSM JSON profile'),
                              OpaqueFunction(function=configure)])
