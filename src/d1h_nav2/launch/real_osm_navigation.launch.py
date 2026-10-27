"""Real-robot planning ONLY. Hardware/localization/filtering are external prerequisites.

No simulator nodes, fake localization, or actuator bridge are launched here.
"""
import json
import math
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def configure(context):
    profile=os.path.abspath(LaunchConfiguration('profile').perform(context))
    with open(profile,encoding='utf-8') as f:p=json.load(f)
    if not os.path.isabs(p['osm_file']) or not os.path.isfile(p['osm_file']):
        raise RuntimeError('profile.osm_file must be an existing absolute Linux path')
    share=get_package_share_directory('d1h_nav2')
    with open(os.path.join(share,'config','nav2_d1h.yaml')) as f:real_nav=yaml.safe_load(f)
    with open(os.path.join(share,'config','local_planner_5m.yaml')) as f:local=yaml.safe_load(f)
    def commissioning(value):
        if isinstance(value,dict):
            for key,item in list(value.items()):
                if key=='robot_base_frame':value[key]='navigation_base'
                elif key=='sensor_frame':value[key]='navigation_sensor'
                elif key=='robot_radius':value[key]=.25
                elif key=='use_sim_time':value[key]=False
                else:commissioning(item)
        elif isinstance(value,list):
            for item in value:commissioning(item)
    commissioning(real_nav);commissioning(local)
    real_nav['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml']=''
    follower=real_nav['controller_server']['ros__parameters']['FollowPath']
    follower['desired_linear_vel']=.2;follower['rotate_to_heading_angular_vel']=.4
    real_nav['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance']=math.pi
    real_nav['velocity_smoother']['ros__parameters'].update(max_velocity=[.3,0.,.5],min_velocity=[0.,0.,-.5])
    real_nav['behavior_server']['ros__parameters'].update(max_rotational_vel=.4,behavior_plugins=['spin','wait'])
    # Nav clouds are already height-filtered relative to fitted ground, not absolute map z.
    for params in (real_nav['local_costmap']['local_costmap']['ros__parameters'],
                   local['local_planner']['global_costmap']['global_costmap']['ros__parameters']):
        layer=params['obstacle_layer'];layer['max_obstacle_height']=100.
        for source in ('livox','livox_clear'):
            layer[source]['min_obstacle_height']=-100.;layer[source]['max_obstacle_height']=100.
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
    # Private runtime BT uses the explicitly named virtual navigation reference.
    bt=open(os.path.join(share,'config','navigate_rolling_5m.xml'),encoding='utf-8').read().replace('base_link','navigation_base')
    import xml.etree.ElementTree as ET
    tree=ET.fromstring(bt)
    for parent in tree.iter():
        for child in list(parent):
            if child.tag=='Sequence' and child.attrib.get('name')=='BackOffWhenBothTurnsBlocked':parent.remove(child)
    with tempfile.NamedTemporaryFile(mode='w',suffix='.xml',prefix='real_osm_bt_',delete=False) as f:
        f.write(ET.tostring(tree,encoding='unicode'));bt_file=f.name
    through=ET.parse(os.path.join(get_package_share_directory('nav2_bt_navigator'),'behavior_trees','navigate_through_poses_w_replanning_and_recovery.xml'))
    for parent in through.iter():
        for child in list(parent):
            if child.tag=='BackUp':parent.remove(child)
    with tempfile.NamedTemporaryFile(mode='w',suffix='.xml',prefix='real_osm_through_',delete=False) as f:
        f.write(ET.tostring(through.getroot(),encoding='unicode'));through_file=f.name
    params=RewrittenYaml(source_file=nav_file,
        param_rewrites={'use_sim_time':'false','default_nav_to_pose_bt_xml':bt_file,
                        'default_nav_through_poses_bt_xml':through_file,'desired_linear_vel':'0.2'},convert_types=True)
    def cleanup(_context):
        for filename in (local_file,nav_file,bt_file,through_file):
            if os.path.isfile(filename):os.unlink(filename)
        return []
    return [GroupAction([
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        # Never send unvalidated deployment-profile commands to the hardware.
        # Explicit velocity wiring avoids global remaps overriding smoother INPUT.
        Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[params],output='screen',
             arguments=['--ros-args','--log-level','error'],remappings=[('cmd_vel','/real_navigation/cmd_vel_raw')]),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params],output='screen',arguments=['--ros-args','--log-level','error']),
        Node(package='nav2_behaviors',executable='behavior_server',name='behavior_server',parameters=[params],output='screen',
             arguments=['--ros-args','--log-level','error'],remappings=[('cmd_vel','/real_navigation/cmd_vel_raw')]),
        Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[params],output='screen',arguments=['--ros-args','--log-level','error']),
        Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',parameters=[params],output='screen',
             arguments=['--ros-args','--log-level','error'],remappings=[('cmd_vel','/real_navigation/cmd_vel_raw'),('cmd_vel_smoothed','/real_navigation/cmd_vel_candidate')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time':False,'autostart':True,'node_names':['controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']}],
             arguments=['--ros-args','--log-level','error']),
        Node(package='d1h_nav2',executable='osm_road_planner',output='screen',parameters=[{'profile':profile,'use_sim_time':False,'robot_frame':'navigation_base','access_half_width_m':.9}]),
        Node(package='d1h_nav2',executable='stable_planner',output='screen',
             parameters=[os.path.join(share,'config','stable_planner.yaml'),{'use_sim_time':False,'robot_frame':'navigation_base'}],
             remappings=[('/compute_path_to_pose','/roads/compute_path_to_pose'),('/is_path_valid','/roads/is_path_valid')]),
        Node(package='d1h_nav2',executable='obstacle_memory',output='screen',
             parameters=[os.path.join(share,'config','obstacle_memory.yaml'),{'use_sim_time':False,'resolution_m':.5,'sensor_frame':'navigation_sensor'}]),
        Node(package='nav2_planner',executable='planner_server',namespace='local_planner',name='planner_server',
             output='screen',parameters=[local_file],arguments=['--ros-args','--log-level','error']),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='local_planner',
             parameters=[{'use_sim_time':False,'autostart':True,'node_names':['planner_server']}],arguments=['--ros-args','--log-level','error']),
        Node(package='d1h_nav2',executable='gps_goal_to_enu',parameters=[{'use_sim_time':False,
             'datum_latitude_deg':p['datum_latitude_deg'],'datum_longitude_deg':p['datum_longitude_deg'],
             'output_topic':'/real_navigation/gps_goal_preview'}]),
        Node(package='d1h_nav2',executable='pose_goal_to_nav',parameters=[{'use_sim_time':False}]),
        Node(package='d1h_nav2',executable='robot_position_marker',parameters=[{'use_sim_time':False}]),
    ])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('profile',description='Absolute path to OSM geometry/datum JSON profile (no approval fields required)'),
                              OpaqueFunction(function=configure)])
