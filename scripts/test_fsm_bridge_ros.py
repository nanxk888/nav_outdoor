"""Real Humble API integration, isolated DDS domain, fake controllers only."""
import json
import os
import subprocess
import sys
import socket
import signal
import tempfile
from pathlib import Path
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState
from std_srvs.srv import Trigger
from robot_wire import encode,decode


def main():
    domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    if not 80<=domain<=99:raise SystemExit('Use an isolated ROS_DOMAIN_ID in 80..99')
    rclpy.init();executor=SingleThreadedExecutor();nodes=[];states={};latest={}
    process=None;receiver=None
    def node(name,namespace='/'):
        n=rclpy.create_node(name,namespace=namespace);nodes.append(n);executor.add_node(n);return n
    def wait(predicate,description,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.03)
            if predicate():return
        raise AssertionError(description+': '+repr(latest))
    try:
        active=node('d1h_rl_controller','/fixture');inactive=node('d1_rl_controller','/fixture')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub=active.create_publisher(String,'/fixture/rl_controller/fsm',qos)
        other=inactive.create_publisher(String,'/fixture/rl_controller/fsm',qos)
        manager=node('controller_manager','/fixture')
        teleop=node('teleop_command','/fixture');teleop.declare_parameter('use_sdk',False)
        def biped(request,response):response.success=True;response.message='biped';return response
        robot_service=manager.create_service(Trigger,'/fixture/command/get_controller_status',biped)
        states.update(d1h_rl_controller='active',d1_rl_controller='inactive',teleop_command='active')
        def listing(request,response):
            response.controller=[ControllerState(name=k,state=v) for k,v in states.items()];return response
        service=manager.create_service(ListControllers,'/fixture/controller_manager/list_controllers',listing)
        observer=node('fsm_test_observer')
        def receive(msg):latest.clear();latest.update(json.loads(msg.data))
        subscription=observer.create_subscription(String,'/nxk_receiver/fsm_status',receive,10)
        # Publish history before the bridge is launched. Inactive history must not win.
        for value in ('transform_up','loco','transform_down','idle'):pub.publish(String(data=value))
        other.publish(String(data='loco'))
        process=subprocess.Popen(['ros2','run','d1h_robot_bridge','fsm_filter','--ros-args',
                                  '-p','robot_namespace:=fixture','--log-level','error'])
        wait(lambda:latest.get('valid') and latest.get('state')=='idle','latched active writer history')
        old_stamp=latest['source_timestamp_ns'];writer=latest['writer']
        other.publish(String(data='debug'))
        deadline=time.monotonic()+.6
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.03)
        assert latest['state']=='idle' and latest['writer']==writer,'inactive event leaked'
        source_floor=time.time_ns();pub.publish(String(data='loco'))
        wait(lambda:latest.get('valid') and latest.get('state')=='loco','new active event')
        assert latest['source_timestamp_ns']>source_floor>old_stamp
        # Run the actual Python receiver against real Humble services/subscriptions.
        # Vendor SDK type is replaced ONLY IN THIS TEST, read-only, no UserCommand sent.
        script=Path(__file__).with_name('robot_udp_receiver.py')
        code=('import sys,types,runpy;sys.path.insert(0,'+repr(str(script.parent))+'); from std_msgs.msg import String; '
              'm=types.ModuleType("ddt_msgs.msg");m.UserCommand=String;'
              'sys.modules["ddt_msgs.msg"]=m;runpy.run_path('+repr(str(script))+',run_name="__main__")')
        with tempfile.TemporaryDirectory() as directory:
            key=b'isolated-test-not-a-real-deployment-key';keyfile=Path(directory)/'udp.key';keyfile.write_bytes(key)
            udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',0));port=udp.getsockname()[1];udp.close()
            receiver=subprocess.Popen([sys.executable,'-c',code,'--bind','127.0.0.1','--peer','127.0.0.1',
                '--port',str(port),'--key-file',str(keyfile)],env=dict(os.environ,ROBOT_NS='fixture'))
            client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);client.settimeout(.03)
            def receiver_ready():
                client.sendto(encode(dict(protocol=2,kind='hello',time=time.time(),seq=1),key),('127.0.0.1',port))
                try:state=decode(client.recvfrom(4096)[0],key)
                except socket.timeout:return False
                return state.get('fsm_valid') is True and state.get('biped') is True and state.get('fsm')=='loco' and state.get('authorized') is False and state.get('sdk_enabled') is False
            try:wait(receiver_ready,'real Humble Python receiver single-argument callback and services')
            finally:
                client.close();receiver.send_signal(signal.SIGINT)
                result=receiver.wait(timeout=5);receiver=None
                assert result==0,'read-only receiver did not exit cleanly'
        states['d1_rl_controller']='active'
        wait(lambda:latest.get('controllers_online') is False,'conflicting controller fails closed')
        states['d1_rl_controller']='inactive'
        wait(lambda:latest.get('valid') is True,'controller service recovery')
        manager.destroy_service(service)
        wait(lambda:latest.get('controllers_online') is False,'lost service expires',timeout=5)
        print('PASS: real Humble C++ MessageInfo/GID + Python receiver callback/services, latched history, inactive filtering, source time, controller conflict and service loss')
    finally:
        if receiver is not None:
            receiver.terminate();receiver.wait(timeout=5)
        if process is not None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        executor.shutdown()
        for n in nodes:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
