"""D1 paired UDP receiver v2, publishes the verified unified UserCommand SDK."""
import argparse
import os
import socket
import time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ddt_msgs.msg import UserCommand
from robot_wire import encode, decode
from robot_policy import ReceiverPolicy,valid_fsm_event


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bind',required=True)
    parser.add_argument('--peer',required=True)
    parser.add_argument('--port',type=int,default=10099)
    parser.add_argument('--key-file',required=True)
    parser.add_argument('--authorize-motion',action='store_true')
    parser.add_argument('--allow-pose',action='store_true')
    parser.add_argument('--allow-height',action='store_true',help='Only after delivered firmware height control is verified')
    args=parser.parse_args()
    ns=os.environ.get('ROBOT_NS','').strip('/')
    if not ns or args.bind=='0.0.0.0': raise SystemExit('Require namespace.sh and actual wired bind IPv4')
    key=Path(args.key_file).read_bytes().strip()
    if len(key)<32: raise SystemExit('Shared key must contain at least 32 bytes')
    rclpy.init();node=rclpy.create_node('nxk_robot_receiver')
    policy=ReceiverPolicy(args.authorize_motion,args.allow_pose,args.allow_height)
    pub=node.create_publisher(UserCommand,f'/{ns}/command/user_command',1)
    topic=f'/{ns}/rl_controller/fsm'
    events={};required_event_ns=0;event_deadline=-100.
    # Cache per DDS writer, never merge histories from active/inactive controllers.
    def fsm_event(msg,info):
        gid=bytes(info.publisher_gid)
        previous=events.get(gid)
        stamp=info.source_timestamp
        if stamp>0 and (previous is None or stamp>=previous[1]):
            events[gid]=(msg.data.strip(),stamp,time.monotonic())
        if len(events)>16:
            oldest=min(events,key=lambda k:events[k][2]);events.pop(oldest)
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub=node.create_subscription(String,topic,fsm_event,qos)
    params=node.create_client(GetParameters,f'/{ns}/teleop_command/get_parameters')
    controller=node.create_client(Trigger,f'/{ns}/command/get_controller_status')
    manager=node.create_client(ListControllers,f'/{ns}/controller_manager/list_controllers')
    checks={'sdk':(False,-100.),'biped':(False,-100.),'controllers':(False,-100.)};pending={}
    def query():
        if manager.service_is_ready() and ('controllers' not in pending or pending['controllers'].done()):
            f=manager.call_async(ListControllers.Request());pending['controllers']=f
            def controllers_done(future):
                try:
                    states={c.name:c.state for c in future.result().controller}
                    ok=(states.get('d1h_rl_controller')=='active' and states.get('d1_rl_controller')=='inactive'
                        and states.get('teleop_command')=='active')
                    checks['controllers']=(ok,time.monotonic())
                except Exception:checks['controllers']=(False,-100.)
            f.add_done_callback(controllers_done)
        if params.service_is_ready() and ('sdk' not in pending or pending['sdk'].done()):
            request=GetParameters.Request();request.names=['use_sdk']
            f=params.call_async(request);pending['sdk']=f
            def sdk_done(future):
                try:
                    value=future.result().values[0];checks['sdk']=(value.type==1 and value.bool_value,time.monotonic())
                except Exception: checks['sdk']=(False,-100.)
            f.add_done_callback(sdk_done)
        if controller.service_is_ready() and ('biped' not in pending or pending['biped'].done()):
            f=controller.call_async(Trigger.Request());pending['biped']=f
            def biped_done(future):
                try:
                    value=future.result();checks['biped']=(value.success and value.message.strip()=='biped',time.monotonic())
                except Exception: checks['biped']=(False,-100.)
            f.add_done_callback(biped_done)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind((args.bind,args.port));sock.setblocking(False)
    def publish(command=(0.,0.),mode=''):
        nonlocal required_event_ns,event_deadline
        if mode:
            # Only a subsequently CREATED FSM sample can confirm this operation.
            required_event_ns=time.time_ns()
            event_deadline=time.monotonic()+2.
            policy.fsm='';policy.fsm_time=-100.
        msg=UserCommand();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='cmd'
        msg.fsm_mode=mode;msg.twist.linear.x=command[0];msg.twist.angular.z=command[1]
        q=policy.pose['xyzw'];msg.pose.orientation.x=q[0];msg.pose.orientation.y=q[1];msg.pose.orientation.z=q[2];msg.pose.orientation.w=q[3]
        if policy.pose['height_m'] is not None:msg.pose.position.z=policy.pose['height_m']
        pub.publish(msg)
    def tick():
        now=time.monotonic()
        online=checks['controllers'][0] and now-checks['controllers'][1]<1.5
        writers=[bytes(p.endpoint_gid) for p in node.get_publishers_info_by_topic(topic)
                 if p.node_name=='d1h_rl_controller' and p.node_namespace.rstrip('/')=='/'+ns]
        event=events.get(writers[0]) if len(writers)==1 else None
        # Cached state is usable only with a live, uniquely identified active controller.
        # It is NOT represented as a newly published event (reported separately).
        valid=valid_fsm_event(event,online,required_event_ns,now)
        if valid:policy.feedback(event[0],now)
        elif online and required_event_ns and now<event_deadline:
            # Await an event without claiming a mode completion or allowing drive.
            policy.fsm='';policy.fsm_time=now
        else:policy.fsm_time=-100.
        policy.interfaces(checks['sdk'][0],checks['biped'][0] and online,min(x[1] for x in checks.values()))
        policy.expire(now)  # Expire old session before accepting a late datagram.
        for _ in range(8):
            try:packet,addr=sock.recvfrom(4097)
            except BlockingIOError:break
            if addr[0]!=args.peer:continue
            try:
                data=decode(packet,key)
            except (ValueError,TypeError,KeyError) as exc:
                # Do not reply to unauthenticated or malformed packets.
                continue
            error=''
            try:policy.accept(data,time.monotonic())
            except (ValueError,TypeError,KeyError) as exc:error=str(exc)
            state=policy.status(time.monotonic());state.update(kind='status',time=time.time(),reply_to=data.get('seq',-1),
                                                               command_accepted=not error,rejection=error)
            state.update(fsm_valid=valid,fsm_event_age_sec=now-event[2] if event else None,
                         fsm_semantics='latched_state_with_controller_service_check',controllers_online=online)
            sock.sendto(encode(state,key),addr)
        command,mode=policy.tick(time.monotonic())
        if args.authorize_motion:publish(command,mode)
    query_timer=node.create_timer(.5,query);timer=node.create_timer(.05,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if args.authorize_motion:
            for _ in range(5):publish();time.sleep(.02)
        sock.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
