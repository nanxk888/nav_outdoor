"""One-command workspace bringup. Existing hardware drivers remain external."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    actions=[SetEnvironmentVariable('ROS_DOMAIN_ID','0'),
             SetEnvironmentVariable('ROS_LOCALHOST_ONLY','0'),
             SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),
             SetEnvironmentVariable('CYCLONEDDS_URI',''),
             SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',''),
             SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE',''),
             SetEnvironmentVariable('ROS_DISCOVERY_SERVER','')]
    critical=[]
    if enabled('with_gps_bridge'):
        xml=Path(arg('gps_cyclonedds_xml'))
        if not xml.is_file(): raise RuntimeError('GPS CycloneDDS configuration not found: '+str(xml))
        bridge=ExecuteProcess(cmd=[str(Path(get_package_prefix('domain_bridge'))/'lib/domain_bridge/domain_bridge'),
            str(root/'map/gnss_bridge.yaml')],output='screen',
            additional_env={'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp','CYCLONEDDS_URI':xml.as_uri()},
            name='gps_domain_bridge')
        critical.append(bridge)
    if enabled('with_lio'):
        lio=Node(package='d1h_fast_lio',executable='lio_odometry',name='navigation_lio',
                 parameters=[os.path.join(get_package_share_directory('d1h_fast_lio'),'config','navigation_lio.yaml'),
                             {'use_sim_time':False}],output='screen',respawn=False)
        critical.append(lio)
    if enabled('with_gnss_gate'):
        gate=Node(package='d1h_global_localization',executable='real_gnss',name='real_gnss_gate',
                  parameters=[os.path.join(get_package_share_directory('d1h_global_localization'),'config','real_lc29h_gnss.yaml'),
                              {'use_sim_time':False}],output='screen',respawn=False)
        critical.append(gate)
    if enabled('with_web'):
        web=ExecuteProcess(cmd=[sys.executable,str(root/'scripts/navigation_web.py')],output='screen',
                           additional_env={'NXK_CONTROL_CONFIG':'','NXK_WEB_TOKEN':''},name='navigation_web')
        critical.append(web)
    actions.append(LogInfo(msg='Domain 0: workspace modules only. Hardware drivers must already run. Web control DISABLED.'))
    # Register before spawning; fail the bringup if any owned top-level process exits.
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[
            EmitEvent(event=Shutdown(reason='A workspace process exited; no automatic restart'))])))
    actions.extend(critical)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace',description='Absolute source checkout path'),
        DeclareLaunchArgument('domain',default_value='0'),
        DeclareLaunchArgument('with_gps_bridge',default_value='true'),
        DeclareLaunchArgument('with_lio',default_value='true'),
        DeclareLaunchArgument('with_gnss_gate',default_value='true'),
        DeclareLaunchArgument('with_web',default_value='true'),
        DeclareLaunchArgument('gps_cyclonedds_xml',default_value='/workspace/d1h-autonomy-final/.local/robot/perception/cyclonedds.xml'),
        OpaqueFunction(function=configure)])
