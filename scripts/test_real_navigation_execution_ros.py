"""Synthetic ROS/Nav2 -> Control -> loopback UDP test. NEVER a physical robot.

Uses an isolated domain, artificial sensors and an in-process fake D1 policy.
The robot pose stays still; this validates command plumbing, not tracking quality.
"""
import os
import sys
import json
import math
import signal
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path
assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99,'Use isolated ROS_DOMAIN_ID=94'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix,PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String,Header
from lifecycle_msgs.srv import GetState
from d1h_nav2.navigation_reference_node import NavigationReference,stamp
from d1h_nav2.lio_memory_node import LioMemoryNode
from navigation_control import Control
from robot_policy import ReceiverPolicy
from robot_wire import encode,decode
from test_osm_roads import RoadsTest


def main():
    road=RoadsTest();road.setUp()
    with tempfile.TemporaryDirectory() as directory:
        directory=Path(directory)
        profile=directory/'profile.json';profile.write_text(json.dumps(dict(osm_file=str(road.file),
            datum_latitude_deg=0.,datum_longitude_deg=0.,road_width_m=4.)))
        key=b'isolated-fixture-only-no-real-robot-key';keyfile=directory/'udp.key';keyfile.write_bytes(key)
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.settimeout(.02)
        policy=ReceiverPolicy(True);received=[];modes=[];end=threading.Event();server_errors=[]
        def receiver():
            while not end.is_set():
                now=time.monotonic();policy.feedback('loco',now);policy.interfaces(True,True,now);policy.expire(now)
                try:packet,addr=sock.recvfrom(4097)
                except socket.timeout:continue
                try:
                    data=decode(packet,key);error=''
                    try:policy.accept(data,time.monotonic())
                    except ValueError as exc:error=str(exc)
                    state=policy.status(time.monotonic());state.update(kind='status',time=time.time(),reply_to=data['seq'],
                        fsm_valid=True,command_accepted=not error,rejection=error)
                    sock.sendto(encode(state,key),addr)
                    if data['kind']=='drive' and not error:received.append((data['v'],data['w']))
                    if data.get('mode'):modes.append(data['mode'])
                except Exception as exc:server_errors.append(str(exc));end.set()
        thread=threading.Thread(target=receiver);thread.start()
        config=directory/'control.json';config.write_text(json.dumps(dict(orin_ip='127.0.0.1',robot_ip='127.0.0.1',
            robot_port=sock.getsockname()[1],key_file=str(keyfile))))
        os.environ['NXK_CONTROL_CONFIG']=str(config)
        rclpy.init(args=['--ros-args','-p','profile:='+str(profile)])
        reference=NavigationReference();memory=LioMemoryNode();fixture=rclpy.create_node('execution_test_fixture')
        control=Control(fixture);executor=SingleThreadedExecutor()
        for n in (reference,memory,fixture):executor.add_node(n)
        pubs={k:fixture.create_publisher(t,topic,10) for k,t,topic in [
            ('odom',Odometry,'/navigation/lio/odometry'),('gps',NavSatFix,'/hardware/gnss/fix'),
            ('nav',PointCloud2,'/livox/lidar/pointcloud_nav'),('clear',PointCloud2,'/livox/lidar/pointcloud_clear'),
            ('ground',String,'/navigation/cloud_filter/status'),('register',String,'/navigation/reference/register')]}
        latest={};diagnostics={};next_pub=0.;next_heartbeat=0.;child=None;log=(directory/'nav2.log').open('w+')
        diagnostic_subs=[]
        for topic,kind in [('/real_navigation/cmd_vel_raw',Twist),('/real_navigation/cmd_vel_candidate',Twist),('/navigation/display_plan',RosPath),('/navigation/replanning_status',String)]:
            def record(msg,t=topic):
                diagnostics[t]=([msg.linear.x,msg.angular.z] if isinstance(msg,Twist) else len(msg.poses) if isinstance(msg,RosPath) else msg.data)
            diagnostic_subs.append(fixture.create_subscription(kind,topic,record,10))
        circle=[[6*math.cos(a*math.pi/180),6*math.sin(a*math.pi/180),-.46] for a in range(360)]
        def step(seconds,inputs=True):
            nonlocal next_pub,next_heartbeat
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                now=time.monotonic()
                if inputs and now>=next_pub:
                    next_pub=now+.1
                    h=Header();h.frame_id='lio_odom';h.stamp=fixture.get_clock().now().to_msg()
                    o=Odometry();o.header=h;o.child_frame_id='lio_imu'
                    o.pose.pose.orientation.y=math.sin(-math.pi/12);o.pose.pose.orientation.w=math.cos(math.pi/12)
                    g=NavSatFix();g.header.stamp=h.stamp;g.status.status=0;g.latitude=0.;g.longitude=0.
                    latest.update(odom=o,gps=g);pubs['odom'].publish(o);pubs['gps'].publish(g)
                    pubs['nav'].publish(point_cloud2.create_cloud_xyz32(h,[]));pubs['clear'].publish(point_cloud2.create_cloud_xyz32(h,circle))
                    pubs['ground'].publish(String(data='{"ready":true,"measurement_age_sec":0.02}'))
                if now>=next_heartbeat:control.enqueue('heartbeat',{});next_heartbeat=now+.2
                executor.spin_once(timeout_sec=.005)
        clients={name:fixture.create_client(GetState,'/'+name+'/get_state') for name in
            ('controller_server','bt_navigator','planner_server','local_planner/planner_server','velocity_smoother')}
        try:
            step(2.)
            pubs['register'].publish(String(data=json.dumps(dict(heading_deg=90.,lio_stamp=stamp(latest['odom']),gps_stamp=stamp(latest['gps']),request_time=time.time()))))
            step(1.);assert reference.anchor is not None and control.safe(),control.status()
            child=subprocess.Popen(['ros2','launch','d1h_nav2','real_osm_navigation.launch.py','profile:='+str(profile)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+40;active={};futures={}
            while time.monotonic()<deadline:
                step(.2)
                if child.poll() is not None:raise RuntimeError('Nav2 launch exited')
                for name,client in clients.items():
                    f=futures.get(name)
                    if f is not None and f.done():
                        active[name]=f.result().current_state.id;futures.pop(name)
                    if name not in futures and client.service_is_ready():futures[name]=client.call_async(GetState.Request())
                if all(active.get(name)==3 for name in clients):break
            assert all(active.get(name)==3 for name in clients),active
            assert control.nav.server_is_ready()
            control.enqueue('navigate',dict(latitude=0.,longitude=3/6378137*180/math.pi))
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not any(v>0 for v,w in received):step(.1)
            assert any(v>0 for v,w in received),(control.status(),active,diagnostics,received[-10:])
            control.enqueue('stop',{});step(.8)
            turn_start=len(received)
            control.enqueue('navigate',dict(latitude=3/6378137*180/math.pi,longitude=0.))
            deadline=time.monotonic()+10
            while time.monotonic()<deadline and not any(w>.01 for v,w in received[turn_start:]):step(.1)
            assert any(w>.01 for v,w in received[turn_start:]),control.status()
            assert all(0<=v<=.3 and abs(w)<=.5 for v,w in received)
            assert not modes,'Navigation must not auto stand/loco/lower'
            step(.9,False);assert control.phase=='STOPPED',control.status()
            assert not policy.active and policy.command==(0.,0.),'Receiver must stop, not just change web phase'
            assert not server_errors,server_errors
            print('PASS: real Nav2 ACTIVE + route/controller -> bounded forward AND left-turn velocity -> authenticated LOOPBACK UDP; sensor loss stops session; no robot-mode commands')
        except Exception:
            log.flush();log.seek(0);lines=log.read().splitlines();print('\n'.join(lines[-100:]));raise
        finally:
            control.close()
            if child is not None and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            end.set();thread.join();sock.close();log.close();executor.shutdown()
            for n in (reference,memory,fixture):n.destroy_node()
            rclpy.shutdown();road.tearDown()


if __name__=='__main__':main()
