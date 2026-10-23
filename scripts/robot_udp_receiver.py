"""Run on D1 in vendor ROS domain. Disabled unless explicitly authorized."""
import argparse
import os
import secrets
import socket
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from robot_wire import encode, decode, velocity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', required=True, help='D1 wired IPv4, not 0.0.0.0')
    parser.add_argument('--peer', required=True, help='Orin wired IPv4')
    parser.add_argument('--port', type=int, default=10099)
    parser.add_argument('--key-file', required=True)
    parser.add_argument('--authorize-motion', action='store_true')
    args = parser.parse_args()
    ns = os.environ.get('ROBOT_NS', '').strip('/')
    if not ns:
        raise SystemExit('Source vendor namespace.sh first')
    key = Path(args.key_file).read_bytes().strip()
    if len(key) < 32:
        raise SystemExit('Require at least 32 bytes of shared secret')
    rclpy.init()
    node = rclpy.create_node('nxk_robot_receiver')
    pub = node.create_publisher(TwistStamped, f'/{ns}/command/cmd_twist', 1)
    fsm_pub = node.create_publisher(String, f'/{ns}/command/cmd_key', 1)
    fsm = ['', 0.]
    def feedback(msg):
        fsm[:] = [msg.data.strip(), time.monotonic()]
    sub = node.create_subscription(String, f'/{ns}/rl_controller/fsm', feedback, 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port)); sock.setblocking(False)
    nonce = secrets.token_hex(16)
    session = None
    last_seq = -1
    last_rx = 0.
    active = False
    fault = True
    command = (0., 0.)
    operation = None
    output = (0., 0.)
    def publish(v=0., w=0.):
        msg = TwistStamped(); msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'; msg.twist.linear.x = v; msg.twist.angular.z = w
        pub.publish(msg)
    def tick():
        nonlocal session, last_seq, last_rx, active, command, fault, operation, nonce, output
        for _ in range(8):
            try:
                packet, addr = sock.recvfrom(4097)
            except BlockingIOError:
                break
            if addr[0] != args.peer:
                continue
            try:
                data = decode(packet, key)
                if data['kind'] != 'hello':
                    if data['nonce'] != nonce or type(data['seq']) is not int:
                        raise ValueError('boot/sequence mismatch')
                    proposed = velocity(data.get('v', 0.), data.get('w', 0.))
                    mode = data.get('mode', '')
                    if mode not in ('', 'transform_up', 'loco'):
                        raise ValueError('mode forbidden')
                    if mode and proposed != (0.,0.):
                        raise ValueError('mode transitions must be neutral')
                    if data['kind'] not in ('arm','drive','stop'):
                        raise ValueError('unknown command')
                    if data['kind'] == 'arm' and (proposed != (0.,0.) or mode):
                        raise ValueError('arm must be neutral')
                    if data['kind'] == 'arm' and args.authorize_motion and not active:
                        session = data['session']; last_seq = -1; fault = False; active = True
                    if not active or data['session'] != session or data['seq'] <= last_seq:
                        raise ValueError('inactive/replayed session')
                    last_seq = data['seq']; last_rx = time.monotonic()
                    if data['kind'] == 'stop':
                        command = (0., 0.); fault = True; active = False; nonce = secrets.token_hex(16)
                    elif data['kind'] in ('arm', 'drive'):
                        command = proposed
                        if mode and data.get('operation') != operation:
                            if time.monotonic()-fsm[1] > 1.:
                                raise ValueError('FSM feedback missing')
                            operation = data['operation']
                            fsm_pub.publish(String(data=mode))
                    else:
                        raise ValueError('unknown command')
            except (ValueError, TypeError, KeyError):
                continue
            status = {'time': time.time(), 'nonce': nonce, 'fsm': fsm[0],
                      'fsm_fresh': time.monotonic()-fsm[1] < 1., 'authorized': args.authorize_motion,
                      'active': active, 'fault': fault, 'session': session, 'ack': last_seq,
                      'reply_to': data.get('seq', -1), 'kind': 'status'}
            sock.sendto(encode(status, key), addr)
        if active and (time.monotonic()-last_rx > .2 or time.monotonic()-fsm[1] > 1.):
            active = False; fault = True; command = (0., 0.); nonce = secrets.token_hex(16)
        if args.authorize_motion:
            # Never replay nonzero motion unless actual fresh loco feedback is present.
            moving = active and not fault and fsm[0] == 'loco' and command != (0.,0.)
            if moving:
                # Conservative per-tick slew; no catch-up burst after scheduler stalls.
                output = tuple(old + max(-step, min(step, target-old))
                               for old,target,step in zip(output,command,(.015,.04)))
            else:
                output = (0.,0.)
            publish(*output)
    timer = node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if args.authorize_motion:
            for _ in range(5):
                publish(); time.sleep(.02)
        sock.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
