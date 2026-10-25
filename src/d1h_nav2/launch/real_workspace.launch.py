"""Workspace bringup with optional guarded GPS driver; Livox driver is external."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import LogInfo
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def report_process_exit(event, context):
    if context.is_shutdown:
        return []
    get_logger('workspace').error('[PROCESS EXIT] '+event.process_name+' exited with code '+str(event.returncode)
                                +'; other workspace modules remain running; no automatic restart. Check this process logs.')
    return []


def configure(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    def enabled(name):
        value=arg(name).lower()
        if value not in ('true','false'): raise RuntimeError(name+' must be true or false')
        return value=='true'
    root=Path(arg('workspace')).resolve()
    if not (root/'scripts/navigation_web.py').is_file():
        raise RuntimeError('workspace must be the source checkout, e.g. /workspace/nxk')
    if arg('domain') != '0': raise RuntimeError('This bringup is explicitly Domain 0; bridge destination is 0')
    xml=Path(arg('gps_cyclonedds_xml')).resolve()
    if not xml.is_file(): raise RuntimeError('Shared CycloneDDS configuration not found: '+str(xml))
    actions=[SetEnvironmentVariable('ROS_DOMAIN_ID','0'),
             SetEnvironmentVariable('ROS_LOCALHOST_ONLY','0'),
             SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
             SetEnvironmentVariable('CYCLONEDDS_URI',xml.as_uri()),
             UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
             UnsetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE'),
             UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER')]
    critical=[]
    nav_enabled=enabled('with_nav')
    if nav_enabled:
        import json
        profile=Path(arg('nav_profile')).expanduser()
        if not profile.is_absolute() or not profile.is_file():
            raise RuntimeError('with_nav requires nav_profile:=/absolute/path/to/reviewed_profile.json')
        p=json.loads(profile.read_text(encoding='utf-8'))
        if p.get('survey_confirmed') is not True or not p.get('approved_way_ids'):
            raise RuntimeError('Nav profile must confirm survey and approved_way_ids; example profile is not deployable')
        if not Path(p.get('osm_file','')).is_absolute() or not Path(p['osm_file']).is_file():
            raise RuntimeError('Nav profile osm_file must exist at an absolute path')
    if enabled('with_gps_driver'):
        guard=ExecuteProcess(cmd=[sys.executable,str(root/'scripts/ensure_real_gnss_driver.py'),
            '--root',arg('gps_driver_root'),'--config',arg('gps_driver_config'),
            '--cyclonedds',arg('gps_cyclonedds_xml')],output='screen',name='gps_driver_guard')
        critical.append(guard)
    if enabled('with_gps_bridge'):
        bridge=ExecuteProcess(cmd=[str(Path(get_package_prefix('domain_bridge'))/'lib/domain_bridge/domain_bridge'),
            str(root/'map/gnss_bridge.yaml')],output='screen',
            additional_env={'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp','CYCLONEDDS_URI':xml.as_uri()},
            name='gps_domain_bridge')
        critical.append(bridge)
    if enabled('with_lio'):
        lio=Node(package='d1h_fast_lio',executable='lio_odometry',name='navigation_lio',
                 arguments=['--ros-args','--log-level',arg('log_level')],
                 parameters=[os.path.join(get_package_share_directory('d1h_fast_lio'),'config','navigation_lio.yaml'),
                             {'use_sim_time':False}],output='screen',respawn=False)
        critical.append(lio)
    if enabled('with_gnss_gate'):
        gate=Node(package='d1h_global_localization',executable='real_gnss',name='real_gnss_gate',
                  arguments=['--ros-args','--log-level',arg('log_level')],
                  parameters=[os.path.join(get_package_share_directory('d1h_global_localization'),'config','real_lc29h_gnss.yaml'),
                              {'use_sim_time':False}],output='screen',respawn=False)
        critical.append(gate)
    if enabled('with_nav_cloud'):
        height=float(arg('sensor_height_m'))
        if not .15<=height<=2.:
            raise RuntimeError('with_nav_cloud requires measured standing IMU sensor_height_m (0.15..2.0)')
        cloud=Node(package='d1h_nav2',executable='real_cloud_filter',name='real_navigation_cloud_filter',
                   parameters=[{'use_sim_time':False,'sensor_height_m':height}],
                   arguments=['--ros-args','--log-level',arg('log_level')],output='screen')
        critical.append(cloud)
    if enabled('with_memory'):
        critical.append(Node(package='d1h_nav2',executable='lio_memory',name='lio_observed_memory',
            parameters=[{'use_sim_time':False}],arguments=['--ros-args','--log-level',arg('log_level')],output='screen'))
    if enabled('with_web'):
        web=ExecuteProcess(cmd=[sys.executable,str(root/'scripts/navigation_web.py')],output='screen',
                           additional_env={'NXK_CONTROL_CONFIG':arg('control_config'),
                                           'NXK_LOG_LEVEL':arg('log_level')},name='navigation_web')
        critical.append(web)
    if nav_enabled:
        critical.append(ExecuteProcess(cmd=['ros2','launch','d1h_nav2','real_osm_navigation.launch.py',
            'profile:='+str(profile)],output='screen',name='nav_navigation'))
    # Isolate process exits; Ctrl+C still uses launch's normal global shutdown.
    def started(event, _context):
        return [LogInfo(msg='[STARTED] '+event.process_name+' process started; data readiness checked separately')]
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessStart(target_action=process,on_start=started)))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=report_process_exit)))
    actions.extend(critical)
    checks=[]
    for flag,label in [('with_lio','lio'),('with_gps_bridge','gps'),('with_gnss_gate','gate'),
                       ('with_nav_cloud','cloud'),('with_memory','memory'),('with_web','web'),('with_nav','nav')]:
        if enabled(flag):checks.append(label)
    actions.append(ExecuteProcess(cmd=[sys.executable,str(root/'scripts/workspace_startup_check.py'),
        '--checks',','.join(checks)],output='screen',name='startup_readiness'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace',description='Absolute source checkout path'),
        DeclareLaunchArgument('domain',default_value='0'),
        DeclareLaunchArgument('log_level',default_value='error'),
        DeclareLaunchArgument('control_config',default_value='',description='Optional private reviewed control config; empty disables robot control'),
        DeclareLaunchArgument('with_gps_driver',default_value='true'),
        DeclareLaunchArgument('gps_driver_root',default_value='/workspace/d1h-autonomy-final'),
        DeclareLaunchArgument('gps_driver_config',default_value='/workspace/d1h-autonomy-final/.local/robot/perception/gnss.yaml'),
        DeclareLaunchArgument('with_gps_bridge',default_value='true'),
        DeclareLaunchArgument('with_lio',default_value='true'),
        DeclareLaunchArgument('with_nav_cloud',default_value='true'),
        DeclareLaunchArgument('with_memory',default_value='true'),
        DeclareLaunchArgument('sensor_height_m',default_value='0.46',description='Measured INITIAL Livox IMU height; override if not in verified low stance'),
        DeclareLaunchArgument('with_nav',default_value='false'),
        DeclareLaunchArgument('nav_profile',default_value='',description='Reviewed OSM profile required when with_nav:=true'),
        DeclareLaunchArgument('with_gnss_gate',default_value='true'),
        DeclareLaunchArgument('with_web',default_value='true'),
        DeclareLaunchArgument('gps_cyclonedds_xml',default_value='/workspace/d1h-autonomy-final/.local/robot/perception/cyclonedds.xml'),
        OpaqueFunction(function=configure)])
