"""Foreground D1 process supervisor; no automatic SDK/mode/network changes."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def conflicting_processes(root=Path('/proc')):
    found=[]
    for entry in root.iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
        try:args=(entry/'cmdline').read_bytes().split(b'\0')
        except (OSError,PermissionError):continue
        names=[Path(os.fsdecode(a)).name for a in args if a]
        if 'robot_udp_receiver.py' in names or (names and names[0]=='fsm_filter'):
            found.append(entry.name)
    return found


def stop_children(children):
    # Independent groups include any child descendants; never kill unrelated nodes.
    for sig,seconds in ((signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,1)):
        alive=[p for p in children if p.poll() is None]
        if not alive:break
        for p in alive:
            try:os.killpg(p.pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+seconds
        while any(p.poll() is None for p in alive) and time.monotonic()<deadline:time.sleep(.05)
    for p in children:p.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind',default='192.168.19.90')
    parser.add_argument('--peer',default='192.168.19.91')
    parser.add_argument('--port',type=int,default=10099)
    parser.add_argument('--authorize-motion',action='store_true')
    parser.add_argument('--allow-pose',action='store_true')
    a=parser.parse_args()
    if a.bind=='0.0.0.0':raise RuntimeError('Use the actual D1 wired IPv4, not 0.0.0.0')
    directory=Path(__file__).resolve().parent
    for name in ('robot_udp_receiver.py','robot_policy.py','robot_wire.py','udp.key'):
        if not (directory/name).is_file():raise RuntimeError('Missing file: '+str(directory/name))
    if len((directory/'udp.key').read_bytes().strip())<32:raise RuntimeError('UDP key too short')
    ns=os.environ.get('ROBOT_NS','').strip('/')
    if not ns:raise RuntimeError('Use bash start_d1_receiver.sh to load namespace.sh')
    if os.environ.get('ROS_DOMAIN_ID')!='42' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Require D1 vendor Domain 42 / localhost-only environment')
    runtime=directory/'.runtime';runtime.mkdir(mode=0o700,exist_ok=True)
    lock=(runtime/'launcher.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('Another unified receiver launcher is already running')
    old=conflicting_processes()
    if old:raise RuntimeError('Existing filter/receiver PIDs '+','.join(old)+'; stop their owning terminals first. Nothing was killed.')
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as check:
        try:check.bind((a.bind,a.port))
        except OSError as exc:raise RuntimeError('Cannot bind D1 wired IP/UDP port; check cable/IP and port owner: '+str(exc))
    # Import checks do not create ROS nodes or publish commands.
    import rclpy
    from ddt_msgs.msg import UserCommand
    from sensor_msgs.msg import BatteryState
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListControllers
    prefix=subprocess.check_output(['ros2','pkg','prefix','d1h_robot_bridge'],text=True,timeout=10).strip()
    binary=Path(prefix)/'lib/d1h_robot_bridge/fsm_filter'
    if not binary.is_file():raise RuntimeError('Build and source bridge_ws first')
    if a.authorize_motion:
        sdk=subprocess.check_output(['ros2','param','get',f'/{ns}/teleop_command','use_sdk'],text=True,timeout=10)
        if 'Boolean value is: True' not in sdk:
            raise RuntimeError('SDK is not enabled. Operator must explicitly enable vendor SDK mode first; launcher never changes it.')
        print('AUTHORIZED: receiver will publish zero velocity/current posture. Verify hardware safety and sole command ownership.',flush=True)
    else:print('READ ONLY: no control command publication; SDK mode is unchanged.',flush=True)
    commands=[[str(binary),'--ros-args','-p',f'robot_namespace:={ns}','--log-level','error'],
              [sys.executable,str(directory/'robot_udp_receiver.py'),'--bind',a.bind,'--peer',a.peer,
               '--port',str(a.port),'--key-file',str(directory/'udp.key')]]
    if a.authorize_motion:commands[1].append('--authorize-motion')
    if a.allow_pose:commands[1].append('--allow-pose')
    children=[];stopping=False
    def stop_signal(signum,frame):
        nonlocal stopping
        stopping=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop_signal)
    try:
        for command in commands:
            if stopping:break
            children.append(subprocess.Popen(command,cwd=directory,start_new_session=True))
        if not stopping:print('Started filter + UDP receiver. Ctrl+C stops both; this is NOT a hardware emergency stop.',flush=True)
        while not stopping:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('A managed process exited; stopping its partner, no automatic restart')
            time.sleep(.1)
    finally:stop_children(children)
    lock.close()


if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('D1 bringup failed: '+str(exc))
