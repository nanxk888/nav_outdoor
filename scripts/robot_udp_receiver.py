"""D1 paired UDP receiver v2, publishes the verified unified UserCommand SDK."""
import argparse
import os
import socket
import time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ddt_msgs.msg import UserCommand
from robot_wire import encode, decode
from robot_policy import ReceiverPolicy


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
    sub=node.create_subscription(String,f'/{ns}/rl_controller/fsm',lambda m:policy.feedback(m.data,time.monotonic()),10)
    params=node.create_client(GetParameters,f'/{ns}/teleop_command/get_parameters')
    controller=node.create_client(Trigger,f'/{ns}/command/get_controller_status')
    checks={'sdk':(False,-100.),'biped':(False,-100.)};pending={}
    def query():
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
        msg=UserCommand();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='cmd'
        msg.fsm_mode=mode;msg.twist.linear.x=command[0];msg.twist.angular.z=command[1]
        q=policy.pose['xyzw'];msg.pose.orientation.x=q[0];msg.pose.orientation.y=q[1];msg.pose.orientation.z=q[2];msg.pose.orientation.w=q[3]
        if policy.pose['height_m'] is not None:msg.pose.position.z=policy.pose['height_m']
        pub.publish(msg)
    def tick():
        now=time.monotonic()
        policy.interfaces(checks['sdk'][0],checks['biped'][0],min(checks['sdk'][1],checks['biped'][1]))
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
