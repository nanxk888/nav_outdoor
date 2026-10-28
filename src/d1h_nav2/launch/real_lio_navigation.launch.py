"""GPS-independent, namespaced observed-space Smac + RPP backend."""
import copy
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction,RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def configure(context):
    share=get_package_share_directory('d1h_nav2')
    with open(os.path.join(share,'config','nav2_d1h.yaml')) as f:base=yaml.safe_load(f)
    with open(os.path.join(share,'config','local_planner_5m.yaml')) as f:local=yaml.safe_load(f)
    controller=copy.deepcopy(base['controller_server']['ros__parameters'])
    controller.update(use_sim_time=False,odom_topic='/lio_navigation/odometry')
    controller['goal_checker'].update(xy_goal_tolerance=.15,yaw_goal_tolerance=3.141593)
    controller['FollowPath'].update(desired_linear_vel=.15,rotate_to_heading_angular_vel=.35,
                                    allow_reversing=False,rotate_to_heading_min_angle=.45)
    smoother=copy.deepcopy(base['velocity_smoother']['ros__parameters'])
    smoother.update(use_sim_time=False,odom_topic='/lio_navigation/odometry',max_velocity=[.15,0.,.35],min_velocity=[0.,0.,-.35])
    planner=copy.deepcopy(local['local_planner']['planner_server']['ros__parameters'])
    planner['use_sim_time']=False;planner['LocalGrid']['tolerance']=.0
    # Occupied, cleared and UNKNOWN evidence comes from the same live memory
    # grid used by the execution checks. Do not ray-clear unknown into free.
    costmap=dict(use_sim_time=False,global_frame='lio_odom',robot_base_frame='lio_navigation_base',
        rolling_window=True,width=12,height=12,resolution=.1,robot_radius=.25,footprint_padding=0.,
        track_unknown_space=True,update_frequency=5.,publish_frequency=5.,transform_tolerance=.25,
        always_send_full_costmap=True,plugins=['observed_space','inflation_layer'],
        observed_space=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/navigation/memory/grid',
                            map_subscribe_transient_local=False,subscribe_to_updates=False),
        inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.4,cost_scaling_factor=5.))
    params={'/lio_nav/controller_server':{'ros__parameters':controller},
            '/lio_nav/planner_server':{'ros__parameters':planner},
            '/lio_nav/velocity_smoother':{'ros__parameters':smoother},
            '/lio_nav/local_costmap/local_costmap':{'ros__parameters':costmap},
            '/lio_nav/global_costmap/global_costmap':{'ros__parameters':copy.deepcopy(costmap)}}
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='lio_nav2_',delete=False) as f:
        yaml.safe_dump(params,f);filename=f.name
    def cleanup(_):
        if os.path.isfile(filename):os.unlink(filename)
        return []
    def node(package,exe,name,remaps=()):
        return Node(package=package,executable=exe,name=name,namespace='lio_nav',parameters=[filename],
                    remappings=remaps,output='screen',arguments=['--ros-args','--log-level','error'])
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        node('nav2_controller','controller_server','controller_server',[('cmd_vel','/lio_navigation/cmd_vel_raw')]),
        node('nav2_planner','planner_server','planner_server'),
        node('nav2_velocity_smoother','velocity_smoother','velocity_smoother',
             [('cmd_vel','/lio_navigation/cmd_vel_raw'),('cmd_vel_smoothed','/lio_navigation/cmd_vel')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='lio_nav',
             name='lifecycle_manager',parameters=[dict(use_sim_time=False,autostart=True,
                 node_names=['controller_server','planner_server','velocity_smoother'])],output='screen'),
        Node(package='d1h_nav2',executable='nav2_speed_sync',namespace='lio_nav',name='nav2_speed_sync',
             parameters=[dict(use_sim_time=False,status_topic='/lio_navigation/speed_status')],output='screen')]


def generate_launch_description():return LaunchDescription([OpaqueFunction(function=configure)])
