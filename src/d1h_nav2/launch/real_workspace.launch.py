"""Workspace bringup with isolated Livox readiness + optional GPS management."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
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
        import tempfile
        if not arg('nav_profile'):
            p=json.loads((root/'map/real_osm_profile.example.json').read_text(encoding='utf-8'))
            p['osm_file']=str(root/'map/xd_map.osm')
            with tempfile.NamedTemporaryFile(mode='w',suffix='.json',prefix='nxk_navigation_',delete=False) as f:
                json.dump(p,f);profile=Path(f.name)
            def cleanup(_context):
                profile.unlink(missing_ok=True);return []
            actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
        else:profile=Path(arg('nav_profile')).expanduser()
        if not profile.is_absolute() or not profile.is_file():
            raise RuntimeError('with_nav requires nav_profile:=/absolute/path/to/osm_profile.json')
        p=json.loads(profile.read_text(encoding='utf-8'))
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
    if enabled('with_lio') or enabled('with_livox_driver'):
        command=[sys.executable,str(root/'scripts/ensure_livox_and_lio.py'),
                 '--setup',arg('livox_setup'),'--config',arg('livox_config'),
                 '--xml',str(xml),'--timeout',arg('livox_wait_sec')]
        if not enabled('with_livox_driver'):command.append('--external-only')
        if enabled('with_lio'):
            command.extend(['--lio-command',str(Path(get_package_prefix('d1h_fast_lio'))/'lib/d1h_fast_lio/lio_odometry'),
                            '--ros-args','--log-level',arg('log_level'),'-r','__node:=navigation_lio',
                            '--params-file',os.path.join(get_package_share_directory('d1h_fast_lio'),'config','navigation_lio.yaml'),
                            '-p','use_sim_time:=false'])
        critical.append(ExecuteProcess(cmd=command,output='screen',name='livox_lio_guard'))
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
            parameters=[{'use_sim_time':False,'footprint_radius_m':.25}],arguments=['--ros-args','--log-level',arg('log_level')],output='screen'))
        critical.append(Node(package='d1h_nav2',executable='local_navigation',name='lio_local_navigation',
            parameters=[{'use_sim_time':False}],arguments=['--ros-args','--log-level',arg('log_level')],output='screen'))
    critical.append(Node(package='d1h_nav2',executable='navigation_reference',name='navigation_reference',
        parameters=[{'use_sim_time':False,'sensor_height_m':float(arg('sensor_height_m')),
                     'profile':str(profile) if nav_enabled else ''}],
        arguments=['--ros-args','--log-level',arg('log_level')],output='screen'))
    if enabled('with_web'):
        web=ExecuteProcess(cmd=[sys.executable,str(root/'scripts/navigation_web.py')],output='screen',
                           additional_env={'NXK_CONTROL_CONFIG':arg('control_config'),
                                           'NXK_LOG_LEVEL':arg('log_level')},name='navigation_web')
        critical.append(web)
    if nav_enabled:
        critical.append(ExecuteProcess(cmd=[sys.executable,str(root/'scripts/start_navigation_when_ready.py'),
            '--profile',str(profile)],output='screen',name='nav_navigation'))
    # Isolate process exits; Ctrl+C still uses launch's normal global shutdown.
    def started(event, _context):
        return [LogInfo(msg='[STARTED] '+event.process_name+' process started; data readiness checked separately')]
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessStart(target_action=process,on_start=started)))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=report_process_exit)))
    actions.extend(critical)
    checks=['reference']
    if enabled('with_memory'):checks.append('local')
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
        DeclareLaunchArgument('control_config',default_value='',description='Optional private UDP connection config; empty disables robot control'),
        DeclareLaunchArgument('with_gps_driver',default_value='true'),
        DeclareLaunchArgument('gps_driver_root',default_value='/workspace/d1h-autonomy-final'),
        DeclareLaunchArgument('gps_driver_config',default_value='/workspace/d1h-autonomy-final/.local/robot/perception/gnss.yaml'),
        DeclareLaunchArgument('with_gps_bridge',default_value='true'),
        DeclareLaunchArgument('with_lio',default_value='true'),
        DeclareLaunchArgument('with_livox_driver',default_value='true',description='Start Livox if absent; reuse only a matching external driver'),
        DeclareLaunchArgument('livox_setup',default_value='/workspace/D1h/d1h-autonomy/.local/sensor-driver/install/setup.bash'),
        DeclareLaunchArgument('livox_config',default_value='/workspace/D1h/d1h-autonomy/.local/sensor-driver/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json'),
        DeclareLaunchArgument('livox_wait_sec',default_value='30',description='Wait for live LiDAR/IMU before LIO; failure does not stop web'),
        DeclareLaunchArgument('with_nav_cloud',default_value='true'),
        DeclareLaunchArgument('with_memory',default_value='true'),
        DeclareLaunchArgument('sensor_height_m',default_value='0.46',description='Measured INITIAL Livox IMU height; override if not in verified low stance'),
        DeclareLaunchArgument('with_nav',default_value='true'),
        DeclareLaunchArgument('nav_profile',default_value='',description='Optional OSM geometry/datum profile; empty uses project map/example datum'),
        DeclareLaunchArgument('with_gnss_gate',default_value='true'),
        DeclareLaunchArgument('with_web',default_value='true'),
        DeclareLaunchArgument('gps_cyclonedds_xml',default_value='/workspace/d1h-autonomy-final/.local/robot/perception/cyclonedds.xml'),
        OpaqueFunction(function=configure)])
