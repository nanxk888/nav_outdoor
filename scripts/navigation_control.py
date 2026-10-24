"""Optional Orin coordinator. No authorization/config => no UDP or ROS commands."""
import json
import math
import os
import queue
import secrets
import socket
import time
from pathlib import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from robot_wire import encode, decode, velocity
from robot_policy import posture


class Control:
    def __init__(self, node):
        self.node = node
        self.requests = queue.Queue(maxsize=16)
        self.phase = 'DISABLED'; self.reason = 'No reviewed control configuration'
        self.state_since = time.monotonic(); self.browser = 0.; self.cmd_time = 0.
        self.cmd = (0., 0.); self.robot = {}; self.robot_time = 0.
        self.safety = {}; self.safety_time = 0.; self.seq = 0
        self.goal_handle = None; self.pending = False; self.generation = 0
        self.config = None; self.sock = None
        self.pose = None
        filename = os.environ.get('NXK_CONTROL_CONFIG')
        if not filename: return
        self.config = json.loads(Path(filename).read_text())
        c = self.config
        if c.get('hardware_authorized') is not True or c.get('interfaces_verified') is not True:
            self.reason = 'Hardware/SDK interfaces not authorized'; return
        self.key = Path(c['key_file']).read_bytes().strip()
        if len(self.key) < 32: raise ValueError('Shared key too short')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((c['orin_ip'], 0)); self.sock.setblocking(False)
        self.target = (c['robot_ip'], int(c.get('robot_port', 10099)))
        self.session = secrets.token_hex(16); self.nonce = ''; self.operation = ''
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.subs = [node.create_subscription(Twist, '/real_navigation/cmd_vel_candidate', self.on_cmd, 1),
                     node.create_subscription(String, '/real_navigation/safety_status', self.on_safety, 1)]
        self.phase = 'IDLE'; self.reason = 'Waiting for explicit web start'
        self.timer = node.create_timer(.05, self.tick)

    def on_cmd(self, msg):
        if self.phase != 'NAVIGATING': return
        try:
            # Current commissioning policy: no automatic reverse.
            if any(abs(x)>1e-8 for x in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                raise ValueError('Non-planar command')
            self.cmd = velocity(msg.linear.x, msg.angular.z); self.cmd_time = time.monotonic()
        except ValueError as exc: self.stop(str(exc))

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data); self.safety_time = time.monotonic()
        except ValueError: self.safety = {}

    def safe(self):
        # Must be supplied by a REAL independent localization/perception supervisor.
        # Existing LIO_SENSOR_TRACKING / GNSS_READY alone are insufficient.
        s = self.safety
        return (time.monotonic()-self.safety_time < .5 and s.get('navigation_ready') is True
                and s.get('base_link_ready') is True and s.get('map_alignment_verified') is True
                and type(s.get('stamp')) in (int,float)
                and -.05 <= self.node.get_clock().now().nanoseconds/1e9-s['stamp'] < .5)

    def enqueue(self, kind, value):
        if not self.sock: raise ValueError(self.reason)
        self.requests.put_nowait((kind, value))
        return {'queued': True, 'executed': False, 'kind': kind}

    def transition(self, phase, reason):
        if phase=='STANDING':self.stand_sent_at=None
        self.phase = phase; self.reason = reason; self.state_since = time.monotonic()
        self.operation = secrets.token_hex(8)

    def stop(self, reason):
        self.generation += 1; self.pending = False
        self.cmd = (0., 0.); self.cmd_time = 0.
        if self.goal_handle:
            self.goal_handle.cancel_goal_async(); self.goal_handle = None
        if self.sock and self.nonce:
            self.send('stop')
        self.transition('STOPPED', reason)

    def send(self, kind, mode='', v=0., w=0., pose=None, confirm_stationary=False):
        self.seq += 1
        data={'protocol':2,'kind': kind, 'time': time.time(), 'seq': self.seq,
            'session': self.session, 'nonce': self.nonce, 'operation': self.operation,
            'mode': mode, 'v': v, 'w': w,'confirm_stationary':confirm_stationary}
        if pose is not None:data['pose']=pose
        self.sock.sendto(encode(data, self.key), self.target)

    def start_goal(self, value):
        if self.phase != 'READY' or not self.safe() or not self.nav.server_is_ready():
            raise ValueError('Need loco READY, fresh fused safety status and Nav2 server')
        c = self.config; lat, lon = value['latitude'], value['longitude']
        if not c.get('survey_confirmed'):raise ValueError('Surveyed map datum required for navigation')
        robot_pose=self.robot.get('pose',{})
        pending_pose=getattr(self,'pose',None) or {}
        if any(abs(p.get(k,0))>1e-5 for p in (robot_pose,pending_pose,self.robot.get('pose_target',{})) for k in ('roll_rad','pitch_rad')):
            raise ValueError('Return posture to neutral before navigation')
        lat0 = math.radians(c['datum_latitude_deg'])
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 6378137.*math.radians(lon-c['datum_longitude_deg'])*math.cos(lat0)
        goal.pose.pose.position.y = 6378137.*math.radians(lat-c['datum_latitude_deg'])
        goal.pose.pose.orientation.w = 1.
        self.cmd_time = 0.; self.cmd = (0.,0.); self.pending = True
        self.generation += 1; generation = self.generation
        self.transition('GOAL_PENDING', 'Waiting for Nav2 goal acknowledgement')
        future = self.nav.send_goal_async(goal)
        def accepted(f):
            try: handle = f.result()
            except Exception as exc:
                if generation == self.generation: self.stop('Goal request failed: '+str(exc))
                return
            if generation != self.generation:
                if handle.accepted: handle.cancel_goal_async()
                return
            self.pending = False
            if not handle.accepted: self.stop('Nav2 rejected goal'); return
            self.goal_handle = handle
            self.transition('NAVIGATING', 'Nav2 goal accepted')
            result = handle.get_result_async()
            def complete(done):
                if generation == self.generation:
                    self.stop('Nav2 finished; status='+str(done.result().status))
            result.add_done_callback(complete)
        future.add_done_callback(accepted)

    def tick(self):
        now = time.monotonic()
        try:
            for _ in range(8):
                try: packet, addr = self.sock.recvfrom(4097)
                except BlockingIOError: break
                if addr != self.target: continue
                try:
                    r = decode(packet, self.key)
                    if r['kind'] != 'status' or r.get('protocol')!=2: continue
                    # Only accept replies to packets sent recently, not old statuses.
                    if type(r.get('reply_to')) is not int or not 0 <= self.seq-r['reply_to'] <= 12: continue
                    self.robot = r; self.robot_time = now; self.nonce = r['nonce']
                    if r.get('command_accepted') is False and self.phase not in ('IDLE','STOPPED','DISABLED'):
                        self.stop('Receiver rejected command: '+r.get('rejection',''))
                except (ValueError, KeyError, TypeError): continue
            for _ in range(4):
                try: kind, value = self.requests.get_nowait()
                except queue.Empty: break
                try:
                    if kind == 'heartbeat': self.browser = now
                    elif kind == 'stop': self.stop('Operator stop')
                    elif kind == 'start':
                        if self.phase not in ('IDLE','STOPPED'): raise ValueError('Already active')
                        if now-self.robot_time>.3 or not all(self.robot.get(k) for k in ('authorized','fsm_fresh','sdk_enabled','biped','interfaces_fresh')):
                            raise ValueError('Robot receiver/FSM not ready')
                        if self.robot.get('fsm') not in ('idle','transform_up','loco'):
                            raise ValueError('Start requires idle, transform_up or loco; do not interrupt lowering')
                        delay=self.config.get('stand_loco_delay_sec',4.)
                        if type(delay) not in (int,float) or not math.isfinite(delay) or not 4<=delay<=8:
                            raise ValueError('stand_loco_delay_sec must be 4..8 seconds')
                        self.browser = now; self.cmd_time = 0.
                        self.pose = None
                        self.session = secrets.token_hex(16)
                        self.transition('ARMING','Request receiver session')
                    elif kind == 'navigate': self.start_goal(value)
                    elif kind == 'lower':
                        if self.phase!='READY' or value.get('confirm_stationary') is not True:
                            raise ValueError('Stop navigation, reattach loco standby and confirm actual standstill before lowering')
                        self.pose=None;self.cmd=(0.,0.);self.transition('LOWER_BRAKING','Holding zero before lowering')
                    elif kind == 'pose':
                        if self.phase!='READY' or now-self.state_since<.7 or not self.robot.get('pose_enabled'):
                            raise ValueError('Pose requires stationary loco READY and receiver pose permission')
                        posture(value,self.robot.get('height_enabled',False));self.pose=dict(value)
                        self.reason='Posture request pending receiver confirmation'
                except (ValueError, KeyError) as exc: self.reason = str(exc)
            active = self.phase not in ('IDLE','STOPPED','DISABLED')
            if active and (now-self.browser>2. or now-self.robot_time>.3):
                self.stop('Browser/robot heartbeat timeout'); active = False
            if active and self.phase != 'ARMING' and (self.robot.get('fault') or not self.robot.get('fsm_fresh')):
                self.stop('Receiver fault / stale FSM'); active = False
            if self.phase == 'ARMING':
                self.send('arm')
                if self.robot.get('active') and not self.robot.get('fault') and self.robot.get('session') == self.session:
                    if self.robot.get('fsm')=='loco':self.transition('READY','Attached to existing loco at zero speed')
                    elif self.robot.get('fsm')=='transform_up':
                        self.transition('STAND_WAIT','Already transform_up: hold zero before loco, do not repeat standing')
                    else:self.transition('STANDING','Request transform_up')
            elif self.phase == 'STANDING':
                if not getattr(self,'stand_sent_at',None):self.stand_sent_at=now
                acknowledged=(self.robot.get('operation')==self.operation and self.robot.get('last_mode')=='transform_up' and self.robot.get('fsm_valid') is True)
                self.send('drive','' if acknowledged else 'transform_up')
                completed=acknowledged and self.robot.get('fsm') in self.config.get('stand_ready_states',[])
                timed=acknowledged and self.robot.get('fsm')=='transform_up' and now-self.stand_sent_at>=self.config.get('stand_loco_delay_sec',4.)
                if completed or timed:
                    self.stand_sent_at=None
                    self.transition('ENTERING_LOCO','Request loco')
            elif self.phase=='STAND_WAIT':
                self.send('drive')
                if self.robot.get('fsm')!='transform_up' or self.robot.get('fsm_valid') is not True:
                    self.stop('Lost transform_up state while waiting; operator review required')
                elif now-self.state_since>=self.config.get('stand_loco_delay_sec',4.):
                    self.transition('ENTERING_LOCO','Existing transform_up held at zero for 4 seconds; request loco')
            elif self.phase == 'ENTERING_LOCO':
                self.send('drive','loco')
                if self.robot.get('fsm') == 'loco': self.transition('READY','loco, holding zero velocity')
            elif self.phase=='LOWER_BRAKING':
                self.send('drive')
                if now-self.state_since>.7:self.transition('LOWERING','Request transform_down')
            elif self.phase=='LOWERING':
                self.send('drive','transform_down',confirm_stationary=True)
                if self.robot.get('fsm')=='idle':self.stop('Lowering finished: idle feedback')
            elif self.phase in ('READY','GOAL_PENDING','NAVIGATING'):
                if self.robot.get('fsm') != 'loco': self.stop('Unexpected FSM'); return
                if self.phase in ('GOAL_PENDING','NAVIGATING') and not self.safe():
                    self.stop('Localization/perception not safe'); return
                if self.phase == 'NAVIGATING':
                    if self.cmd_time and now-self.cmd_time>.2:
                        self.stop('Navigation velocity timeout'); return
                    if not self.cmd_time and now-self.state_since>2.:
                        self.stop('No navigation velocity'); return
                fields={'v':self.cmd[0] if self.phase=='NAVIGATING' else 0.,'w':self.cmd[1] if self.phase=='NAVIGATING' else 0.}
                if self.phase=='READY' and getattr(self,'pose',None) is not None:fields['pose']=self.pose
                self.send('drive',**fields)
            else: self.send('hello')
            if self.phase in ('ARMING','STANDING','STAND_WAIT','ENTERING_LOCO','LOWERING') and now-self.state_since>10.:
                self.stop('Robot transition timeout')
            if self.phase == 'GOAL_PENDING' and now-self.state_since>2.:
                self.stop('Nav2 acknowledgement timeout')
        except OSError as exc:
            self.generation += 1; self.cmd = (0.,0.)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async(); self.goal_handle = None
            self.transition('STOPPED','Network error: '+str(exc))

    def status(self):
        return {'phase': self.phase, 'reason': self.reason, 'navigation_safe': self.safe(),
                'robot': self.robot, 'robot_receive_age_sec': time.monotonic()-self.robot_time,
                'enabled': self.sock is not None}

    def close(self):
        if self.sock:
            self.stop('Web gateway shutdown'); self.sock.close()
