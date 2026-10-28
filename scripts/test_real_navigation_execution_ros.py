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
from d1h_nav2.lio_nav2_executor import LioNav2Executor as LocalNavigation
from navigation_control import Control
from robot_policy import ReceiverPolicy
from robot_wire import encode,decode
from test_osm_roads import RoadsTest


def main():
    local_test='--local' in sys.argv
    road=RoadsTest();road.setUp()
    with tempfile.TemporaryDirectory() as directory:
        directory=Path(directory)
        profile=directory/'profile.json';profile.write_text(json.dumps(dict(osm_file=str(road.file),
            datum_latitude_deg=0.,datum_longitude_deg=0.,road_width_m=4.)))
        key=b'isolated-fixture-only-no-real-robot-key';keyfile=directory/'udp.key';keyfile.write_bytes(key)
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.settimeout(.02)
        policy=ReceiverPolicy(True);received=[];modes=[];end=threading.Event();server_errors=[];sdk_on=[True];fsm_value=['loco']
        def receiver():
            while not end.is_set():
                now=time.monotonic();policy.feedback(fsm_value[0],now);policy.interfaces(sdk_on[0],True,now);policy.expire(now)
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
        nodes=[reference,memory,fixture]
        local_executor=None
        if local_test:local_executor=LocalNavigation();nodes.append(local_executor)
        for n in nodes:executor.add_node(n)
        pubs={k:fixture.create_publisher(t,topic,10) for k,t,topic in [
            ('odom',Odometry,'/navigation/lio/odometry'),('gps',NavSatFix,'/hardware/gnss/fix'),
            ('nav',PointCloud2,'/livox/lidar/pointcloud_nav'),('clear',PointCloud2,'/livox/lidar/pointcloud_clear'),
            ('ground',String,'/navigation/cloud_filter/status'),('register',String,'/navigation/reference/register'),
            ('goal',String,'/navigation/memory/goal_candidate'),('reset',String,'/navigation/memory/reset')]}
        position=[0.,0.];browser_on=[True];obstacles=[]
        latest={};diagnostics={};next_pub=0.;next_heartbeat=0.;child=None;log=(directory/'nav2.log').open('w+')
        diagnostic_subs=[]
        for topic,kind in [('/real_navigation/cmd_vel_raw',Twist),('/real_navigation/cmd_vel_candidate',Twist),('/navigation/display_plan',RosPath),('/navigation/replanning_status',String)]:
            def record(msg,t=topic):
                diagnostics[t]=([msg.linear.x,msg.angular.z] if isinstance(msg,Twist) else len(msg.poses) if isinstance(msg,RosPath) else msg.data)
            diagnostic_subs.append(fixture.create_subscription(kind,topic,record,10))
        circle=[[5.5*math.cos(a*math.pi/180),5.5*math.sin(a*math.pi/180),-.46] for a in range(360)]
        def step(seconds,inputs=True):
            nonlocal next_pub,next_heartbeat
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                now=time.monotonic()
                if inputs and now>=next_pub:
                    next_pub=now+.1
                    h=Header();h.frame_id='lio_odom';h.stamp=fixture.get_clock().now().to_msg()
                    o=Odometry();o.header=h;o.child_frame_id='lio_imu'
                    o.pose.pose.orientation.x=-math.sin(-math.pi/12);o.pose.pose.orientation.z=math.cos(math.pi/12);o.pose.pose.orientation.w=0.
                    o.pose.pose.position.x=position[0];o.pose.pose.position.y=position[1]
                    g=NavSatFix();g.header.stamp=h.stamp;g.status.status=0;g.latitude=0.;g.longitude=0.
                    latest.update(odom=o,gps=g);pubs['odom'].publish(o)
                    if not local_test:pubs['gps'].publish(g)
                    pubs['nav'].publish(point_cloud2.create_cloud_xyz32(h,obstacles))
                    pubs['clear'].publish(point_cloud2.create_cloud_xyz32(h,[[x+position[0],y+position[1],z] for x,y,z in circle]))
                    pubs['ground'].publish(String(data='{"ready":true,"measurement_age_sec":0.02}'))
                if browser_on[0] and now>=next_heartbeat:control.enqueue('heartbeat',{});next_heartbeat=now+.2
                executor.spin_once(timeout_sec=.005)
        clients={name:fixture.create_client(GetState,'/'+name+'/get_state') for name in
            ('controller_server','bt_navigator','planner_server','local_planner/planner_server','velocity_smoother')}
        def local_scenarios():
            def select(x,y):
                pubs['goal'].publish(String(data=json.dumps(dict(x=x,y=y,frame=memory.frame,epoch=memory.epoch,request_time=time.time()))))
                step(1.2)
                assert control.local_state.get('ready'),control.local_state
            def begin():
                control.enqueue('lio_navigate',dict(epoch=memory.epoch,goal=list(memory.goal[:2])))
                deadline=time.monotonic()+3
                while control.phase!='NAVIGATING' and time.monotonic()<deadline:step(.1)
                assert control.phase=='NAVIGATING',(control.status(),control.local_state)
                end_command=time.monotonic()+3
                while local_executor.cmd==(0.,0.) and time.monotonic()<end_command:step(.1)
                step(.2)
            assert reference.anchor is None,'No GPS anchor is allowed in this test'
            pubs['goal'].publish(String(data=json.dumps(dict(x=10.,y=10.,frame=memory.frame,epoch=memory.epoch,request_time=time.time()))))
            step(1.2);assert not control.local_state.get('ready'),control.local_state
            try:control.start_local(dict(epoch=memory.epoch,goal=[10.,10.]))
            except ValueError:pass
            else:raise AssertionError('Partial frontier must not start local navigation')
            select(1.,0.);assert not any(v or w for v,w in received),'preview must not drive'
            begin();assert any(v>0 for v,w in received),control.status()
            for i in range(1,10):position[0]=i*.1;step(.15)
            step(.8);assert control.phase=='STOPPED' and 'SUCCEEDED' in control.reason,control.status()
            assert policy.command==(0.,0.) and not policy.active
            select(.9,1.);index=len(received);begin()
            assert any(w>.01 for v,w in received[index:]),received[index:]
            step(.9,False);assert control.phase=='STOPPED' and not policy.active
            step(1.3);assert control.phase=='STOPPED','sensor recovery must not resume'
            select(1.7,0.);begin();sdk_on[0]=False;step(.6)
            assert control.phase=='STOPPED' and not policy.active
            sdk_on[0]=True;step(.6);assert control.phase=='STOPPED'
            select(2.7,0.);begin();obstacles.append([1.8,0.,.2]);step(2.)
            assert control.phase=='NAVIGATING' and any(abs(p[1])>.2 for p in local_executor.plan_points),('Smac must reroute around obstacle',control.status(),local_executor.reason)
            control.enqueue('stop',{});step(.5)
            obstacles.clear();pubs['reset'].publish(String(data=json.dumps(dict(epoch=memory.epoch,request_time=time.time()))));step(1.2)
            assert control.phase=='STOPPED','Memory reset/recovery cannot resume'
            select(1.7,0.);begin()
            pubs['reset'].publish(String(data=json.dumps(dict(epoch=memory.epoch,request_time=time.time()))));step(.8)
            assert control.phase=='STOPPED' and not policy.active,'Active memory reset must stop the task'
            select(1.7,0.);begin();browser_on[0]=False;step(2.5)
            assert control.phase=='STOPPED' and not policy.active
            browser_on[0]=True;step(.5);select(1.7,0.);begin();fsm_value[0]='idle';step(.5)
            assert control.phase=='STOPPED' and not policy.active,'Leaving loco must stop'
            fsm_value[0]='loco';step(.6);assert control.phase=='STOPPED'
            select(1.7,0.);begin();control.timer.cancel();step(.7)
            assert control.phase=='STOPPED' and 'lease' in control.reason and not policy.active,control.status()
            control.timer.reset()
            assert all(0<=v<=.15 and abs(w)<=.35 for v,w in received)
            control.enqueue('speed_limits',dict(linear_mps=.4,angular_rps=.5));step(2.)
            select(2.7,0.);index=len(received);begin();step(1.)
            assert any(v>.15 for v,w in received[index:]),('LIO RPP must apply frontend speed, not the old fixed ceiling',control.status(),local_executor.backend)
            assert all(0<=v<=.400001 and abs(w)<=.500001 for v,w in received[index:])
            control.enqueue('speed_limits',dict(linear_mps=.1,angular_rps=.1));step(1.)
            index=len(received);step(.5)
            assert control.phase=='NAVIGATING' and all(v<=.100001 and abs(w)<=.100001 for v,w in received[index:]),control.status()
            local_executor.cancel_backend();local_executor.next_plan=time.monotonic()+5.
            step(3.6)
            assert control.phase=='NAVIGATING' and policy.command==(0.,0.),('temporary missing path must hold task at zero, not cancel after 3s',control.status())
            local_executor.started-=65.;local_executor.next_plan=0.;step(1.)
            assert control.phase=='NAVIGATING','No fixed 60-second task deadline'
            control.enqueue('stop',{});step(.5)
            select(4.5,0.);begin();step(.3)
            assert control.phase=='NAVIGATING',('Observed goal beyond 2.5m must be accepted',control.status())
            control.enqueue('stop',{});step(.5)
            assert not modes and not server_errors
            print('PASS: GPS-free Smac/RPP -> local forward/turn -> authenticated LOOPBACK UDP; partial rejected, arrival zero, obstacle reroute, reset/sensor/SDK/FSM/browser/lease stop, no auto resume or mode commands')
        try:
            if local_test:
                child=subprocess.Popen(['ros2','launch','d1h_nav2','real_lio_navigation.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+40
                while time.monotonic()<deadline:
                    step(.2)
                    if local_executor.planner.server_is_ready() and local_executor.follower.server_is_ready() and local_executor.backend.get('ready'):break
                assert local_executor.planner.server_is_ready() and local_executor.backend.get('ready'),local_executor.backend
            step(2.)
            if local_test:
                local_scenarios();return
            pubs['register'].publish(String(data=json.dumps(dict(heading_deg=90.,lio_stamp=stamp(latest['odom']),gps_stamp=stamp(latest['gps']),request_time=time.time()))))
            step(1.);assert reference.anchor is not None and control.safe(),control.status()
            child=subprocess.Popen(['ros2','launch','d1h_nav2','real_osm_navigation.launch.py','profile:='+str(profile)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+40;active={};futures={};future_started={}
            while time.monotonic()<deadline:
                step(.2)
                if child.poll() is not None:raise RuntimeError('Nav2 launch exited')
                for name,client in clients.items():
                    f=futures.get(name)
                    if f is not None and f.done():
                        active[name]=f.result().current_state.id;futures.pop(name)
                    elif f is not None and time.monotonic()-future_started[name]>2:
                        f.cancel();futures.pop(name)
                    if name not in futures and client.service_is_ready():
                        futures[name]=client.call_async(GetState.Request());future_started[name]=time.monotonic()
                if all(active.get(name)==3 for name in clients) and control.outdoor_speed_ready():break
            assert all(active.get(name)==3 for name in clients),active
            assert control.nav.server_is_ready()
            assert control.outdoor_speed_ready(),control.status()
            control.enqueue('speed_limits',dict(linear_mps=.6,angular_rps=.5))
            step(.3);deadline=time.monotonic()+6
            while not control.outdoor_speed_ready() and time.monotonic()<deadline:step(.1)
            assert control.outdoor_speed_ready(),control.status()
            control.enqueue('navigate',dict(latitude=0.,longitude=3/6378137*180/math.pi))
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not any(v>.21 for v,w in received):step(.1)
            assert any(v>.21 for v,w in received),(control.status(),active,diagnostics,received[-10:])
            control.enqueue('stop',{});step(.8)
            turn_start=len(received)
            control.enqueue('navigate',dict(latitude=3/6378137*180/math.pi,longitude=0.))
            deadline=time.monotonic()+10
            while time.monotonic()<deadline and not any(w>.01 for v,w in received[turn_start:]):step(.1)
            assert any(w>.01 for v,w in received[turn_start:]),control.status()
            assert all(0<=v<=.600001 and abs(w)<=.500001 for v,w in received)
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
            for n in nodes:n.destroy_node()
            rclpy.shutdown();road.tearDown()


if __name__=='__main__':main()
