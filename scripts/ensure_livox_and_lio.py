"""Own an optional Livox driver; start LIO only after live sensor validation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def drivers(proc=Path('/proc'),executable='livox_ros_driver2_node'):
    found=[]
    for entry in proc.iterdir():
        if not entry.name.isdigit():continue
        try:
            argv=[os.fsdecode(x) for x in (entry/'cmdline').read_bytes().split(b'\0') if x]
            # Actual binary only: ros2 run's wrapper is not another driver.
            if not argv or Path(argv[0]).name!=executable:continue
            env=dict(os.fsdecode(x).split('=',1) for x in (entry/'environ').read_bytes().split(b'\0') if b'=' in x)
            found.append((int(entry.name),argv,env))
        except (FileNotFoundError,ProcessLookupError):continue
    return found


def compatible(existing,xml,config):
    pid,argv,env=existing
    expected={'ROS_DOMAIN_ID':'0','ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp','CYCLONEDDS_URI':Path(xml).resolve().as_uri()}
    if any(env.get(k)!=v for k,v in expected.items()):
        raise ValueError(f'Existing Livox PID={pid} DDS environment conflicts; stop/reconfigure it manually. No second driver started.')
    if not any(a=='user_config_path:='+str(Path(config).resolve()) for a in argv):
        raise ValueError(f'Existing Livox PID={pid} configuration path differs; inspect it manually')


def check_network(config):
    c=json.loads(Path(config).read_text());mid=c['MID360'];host=mid['host_net_info']
    ips={host[k] for k in ('cmd_data_ip','point_data_ip','imu_data_ip')}
    if len(ips)!=1:raise ValueError('Expected one Orin address for Livox command/point/IMU')
    address=ips.pop();lidars=c['lidar_configs']
    if len(lidars)!=1:raise ValueError('This guard supports exactly one configured Livox')
    target=lidars[0]['ip']
    links=json.loads(subprocess.check_output(['ip','-j','-4','addr','show'],text=True))
    owners=[d for d in links if any(a.get('local')==address for a in d.get('addr_info',[]))]
    if len(owners)!=1 or 'LOWER_UP' not in owners[0].get('flags',[]):
        raise ValueError(f'Livox host IP {address} missing or link down; inspect USB Ethernet/cable')
    route=json.loads(subprocess.check_output(['ip','-j','route','get',target],text=True))[0]
    if route.get('dev')!=owners[0]['ifname'] or route.get('prefsrc',route.get('src'))!=address:
        raise ValueError('Livox route/source mismatch; will not change networking automatically')
    subprocess.run(['ping','-n','-c','1','-W','2','-I',address,target],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
    print(f'[READY] Livox network: {owners[0]["ifname"]} {address} -> {target}',flush=True)


def lidar_header(raw):
    """Read bounded CDR1 CustomMsg metadata without 20k Python point objects.

    Header, uint64 timebase, uint32 point_num, uint8 lidar_id, uint8[3]
    reserved, sequence<CustomPoint>. CDR alignment is relative to encapsulation.
    Unsupported encodings fail explicitly, never count as live sensor data.
    """
    if len(raw)<16 or raw[:2] not in (b'\x00\x01',b'\x00\x00'):raise ValueError('Unsupported/truncated CustomMsg CDR1')
    endian='<' if raw[1]==1 else '>'
    sec,nsec,size=struct.unpack_from(endian+'iII',raw,4)
    if nsec>=1000000000 or not 1<=size<=256 or 16+size>len(raw):raise ValueError('Invalid CustomMsg header')
    if raw[15+size]!=0:raise ValueError('Invalid CustomMsg frame terminator')
    frame=raw[16:15+size].decode('utf-8')
    pos=4+((16+size-4+7)//8)*8
    if pos+20>len(raw):raise ValueError('Truncated CustomMsg metadata')
    points=struct.unpack_from(endian+'I',raw,pos+8)[0]
    count=struct.unpack_from(endian+'I',raw,pos+16)[0]
    if not 0<points<=100000 or count!=points or len(raw)-(pos+20)<count*20-1:
        raise ValueError(f'Invalid CustomMsg point counts/payload: point_num={points} sequence={count}')
    return sec+nsec/1e9,frame,points


class SensorGate:
    def __init__(self):self.state={};self.stats={}
    def reject(self,key,reason):
        self.state.pop(key,None)
        s=self.stats.setdefault(key,dict(received=0,rejected=0,last_reason=''))
        s['rejected']+=1;s['last_reason']=reason
    def observe(self,key,stamp,ros,now):
        s=self.stats.setdefault(key,dict(received=0,rejected=0,last_reason=''))
        s['received']+=1;s['source_age_sec']=round(ros-stamp,4)
        previous=self.state.get(key)
        if not -.05<=ros-stamp<.5:
            self.reject(key,f'source stamp stale/future: age={ros-stamp:.4f}s');return
        if previous and stamp<=previous[0]:
            self.reject(key,'source stamp not increasing');return
        first=previous[2] if previous and now-previous[1]<.4 else now
        count=previous[3]+1 if previous and now-previous[1]<.4 else 1
        self.state[key]=(stamp,now,first,count)
        s['last_reason']='accepted'
    def detail(self,now):
        return {key:dict(self.stats.get(key,{}),**(dict(count=self.state[key][3],receive_age=round(now-self.state[key][1],3)) if key in self.state else dict(count=0))) for key in ('lidar','imu')}
    def ready(self,ros,now):
        return all(k in self.state and now-self.state[k][1]<.3 and -.05<=ros-self.state[k][0]<.5
                   and now-self.state[k][2]>=1. and self.state[k][3]>=count
                   for k,count in (('lidar',5),('imu',40)))


def stop_child(child):
    if child is None or child.poll() is not None:return
    for sig,timeout in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        try:os.killpg(child.pid,sig)
        except ProcessLookupError:return
        try:child.wait(timeout=timeout);return
        except subprocess.TimeoutExpired:pass


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--setup',required=True);p.add_argument('--config',required=True);p.add_argument('--xml',required=True)
    p.add_argument('--timeout',type=float,default=30.)
    p.add_argument('--external-only',action='store_true')
    p.add_argument('--lio-command',nargs=argparse.REMAINDER,default=[])
    a=p.parse_args();owned=None;lio=None;node=None;rclpy=None
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        a.config=str(Path(a.config).resolve());a.xml=str(Path(a.xml).resolve())
        if not 5<=a.timeout<=120:raise ValueError('Sensor wait timeout must be 5..120 seconds')
        if not Path(a.xml).is_file():raise ValueError('Cyclone DDS XML missing')
        if a.lio_command and drivers(executable='lio_odometry'):
            raise ValueError('LIO process already running; stop old workspace before relaunch; no duplicate started')
        check_network(a.config)
        existing=drivers()
        if len(existing)>1:raise ValueError('Multiple Livox drivers detected; resolve duplicates manually')
        if existing:
            compatible(existing[0],a.xml,a.config)
            print(f'[REUSE] External Livox PID={existing[0][0]}; will NOT stop it on shutdown',flush=True)
        elif a.external_only:raise ValueError('External-only mode: no Livox driver running')
        else:
            setup=Path(a.setup).resolve();exe=setup.parent/'livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
            # Accommodate isolated and merged install prefixes.
            if not exe.is_file():exe=setup.parent/'lib/livox_ros_driver2/livox_ros_driver2_node'
            if not setup.is_file() or not exe.is_file():raise ValueError('Livox setup/executable missing; set livox_setup explicitly')
            command=['bash','-c','source "$1" && exec env ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="$2" "$3" "${@:4}"',
                     'livox_driver',str(setup),Path(a.xml).as_uri(),str(exe),'--ros-args']
            for k,v in dict(xfer_format=1,multi_topic=0,data_src=0,publish_freq=10.0,output_data_type=0,frame_id='livox_frame',user_config_path=a.config).items():
                command.extend(['-p',f'{k}:={v}'])
            owned=subprocess.Popen(command,start_new_session=True)
            print(f'[STARTED] Owned Livox PID={owned.pid}; Cyclone DDS matches navigation',flush=True)
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from sensor_msgs.msg import Imu
        from livox_ros_driver2.msg import CustomMsg
        rclpy.init();node=rclpy.create_node('nxk_livox_startup_guard');gate=SensorGate()
        def received(key,msg):
            try:
                if key=='lidar':stamp,frame,_=lidar_header(msg)
                else:
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;frame=msg.header.frame_id
                    if not all(math.isfinite(v) for xyz in (msg.linear_acceleration,msg.angular_velocity) for v in (xyz.x,xyz.y,xyz.z)):
                        raise ValueError('Nonfinite IMU')
                if frame!='livox_frame':raise ValueError(f'Unexpected frame: {frame!r}')
                gate.observe(key,stamp,node.get_clock().now().nanoseconds/1e9,time.monotonic())
            except (ValueError,TypeError,struct.error) as exc:gate.reject(key,str(exc))
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        subs=[node.create_subscription(CustomMsg,'/livox/lidar',lambda m:received('lidar',m),qos,raw=True),
              node.create_subscription(Imu,'/livox/imu',lambda m:received('imu',m),qos)]
        print('[WAIT] Livox: require fresh, increasing LiDAR + IMU for 1s before LIO',flush=True)
        deadline=time.monotonic()+a.timeout;graph_check=0.;report_at=time.monotonic()+5
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if owned is not None and owned.poll() is not None:raise ValueError(f'Owned Livox exited: code={owned.returncode}')
            if existing and not Path(f'/proc/{existing[0][0]}').exists():raise ValueError('External Livox exited; no automatic replacement')
            if time.monotonic()>=graph_check:
                if any(len(node.get_publishers_info_by_topic(t))>1 for t in ('/livox/lidar','/livox/imu')):raise ValueError('Multiple sensor publishers; LIO not started')
                graph_check=time.monotonic()+.5
            if gate.ready(node.get_clock().now().nanoseconds/1e9,time.monotonic()):break
            if time.monotonic()>=report_at:
                print('[WAIT] Livox sensor diagnostics: '+str(gate.detail(time.monotonic())),flush=True)
                report_at=time.monotonic()+5
            if time.monotonic()>deadline:
                detail=gate.detail(time.monotonic())
                raise ValueError(f'Livox sensor readiness timed out: {detail}; inspect DDS/stamps/driver. LIO not started')
        print('[READY] Livox: both streams fresh and advancing',flush=True)
        if a.lio_command:
            lio=subprocess.Popen(a.lio_command,start_new_session=True)
            print(f'[STARTED] LIO PID={lio.pid} after sensor readiness',flush=True)
        warned=False
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if owned is not None and owned.poll() is not None:raise ValueError(f'Owned Livox exited: code={owned.returncode}')
            if existing and not Path(f'/proc/{existing[0][0]}').exists():raise ValueError('External Livox exited')
            if lio is not None and lio.poll() is not None:
                print(f'[ERROR] LIO exited: code={lio.returncode}; driver and other modules remain running; explicit restart required',flush=True)
                lio=None
            healthy=gate.ready(node.get_clock().now().nanoseconds/1e9,time.monotonic())
            if not healthy and not warned:print('[ERROR] Livox stream stale/invalid; LIO retains its own fail-stop checks; no automatic restart',flush=True)
            warned=not healthy
    except KeyboardInterrupt:return 0
    except Exception as exc:
        print('[ERROR] Livox/LIO bringup: '+str(exc)+'; other workspace modules remain running',flush=True);return 1
    finally:
        stop_child(lio);stop_child(owned)
        if node is not None:node.destroy_node()
        if rclpy is not None and rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
