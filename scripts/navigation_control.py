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
        self.phase = 'DISABLED'; self.reason = 'No robot UDP connection configuration'
        self.state_since = time.monotonic(); self.browser = 0.; self.cmd_time = 0.
        self.cmd = (0., 0.); self.robot = {}; self.robot_time = 0.
        self.safety = {}; self.safety_time = 0.; self.seq = 0
        self.goal_handle = None; self.pending = False; self.generation = 0
        self.config = None; self.sock = None
        self.pose = None
        self.batteries={};self.battery_time=0.;self.battery_query_time=-100.
        filename = os.environ.get('NXK_CONTROL_CONFIG')
        if not filename: return
        self.config = json.loads(Path(filename).read_text())
        c = self.config
        self.key = Path(c['key_file']).read_bytes().strip()
        if len(self.key) < 32: raise ValueError('Shared key too short')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:self.sock.bind((c['orin_ip'], 0)); self.sock.setblocking(False)
        except OSError:
            self.sock.close();self.sock=None;raise
        self.target = (c['robot_ip'], int(c.get('robot_port', 10099)))
        self.session = secrets.token_hex(16); self.nonce = ''; self.operation = ''
        self.local_mode=False;self.local_state={};self.local_time=-math.inf;self.local_run=None
        self.local_pub=node.create_publisher(String,'/navigation/local_execution/request',5)
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.subs = [node.create_subscription(Twist, '/real_navigation/cmd_vel_candidate', self.on_cmd, 1),
                     node.create_subscription(String, '/real_navigation/safety_status', self.on_safety, 1),
                     node.create_subscription(String,'/navigation/local_execution/status',self.on_local_status,5)]
        self.phase = 'IDLE'; self.reason = 'Waiting for explicit web start'
        self.timer = node.create_timer(.05, self.tick)

    def on_cmd(self, msg):
        if getattr(self,'local_mode',False):return
        if self.phase != 'NAVIGATING': return
        try:
            # Current commissioning policy: no automatic reverse.
            if any(abs(x)>1e-8 for x in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                raise ValueError('Non-planar command')
            self.cmd = velocity(msg.linear.x, msg.angular.z); self.cmd_time = time.monotonic()
        except ValueError as exc: self.stop(str(exc))

    def on_safety(self, msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Expected status object')
            previous=self.safety.get('reference_epoch')
            self.safety=value;self.safety_time=time.monotonic()
            if not getattr(self,'local_mode',False) and previous is not None and previous!=value.get('reference_epoch') and self.phase in ('GOAL_PENDING','NAVIGATING'):
                self.stop('Navigation reference changed; start a new goal explicitly')
        except (ValueError,TypeError): self.safety = {}

    def safe(self):
        if getattr(self,'local_mode',False):
            s=self.local_state
            if self.phase=='NAVIGATING' and (s.get('run_id')!=self.local_run or s.get('state')!='ACTIVE'):return False
            return (time.monotonic()-self.local_time<.3 and s.get('ready') is True and type(s.get('stamp')) in (int,float)
                    and -.05<=self.node.get_clock().now().nanoseconds/1e9-s['stamp']<.3)
        # Must be supplied by a REAL independent localization/perception supervisor.
        # Existing LIO_SENSOR_TRACKING / GNSS_READY alone are insufficient.
        s = self.safety
        return (time.monotonic()-self.safety_time < .5 and s.get('navigation_ready') is True
                and (s.get('navigation_base_ready') is True or s.get('base_link_ready') is True) and s.get('map_alignment_verified') is True
                and type(s.get('stamp')) in (int,float)
                and -.05 <= self.node.get_clock().now().nanoseconds/1e9-s['stamp'] < .5)

    def enqueue(self, kind, value):
        if not self.sock: raise ValueError(self.reason)
        self.requests.put_nowait((kind, value))
        return {'queued': True, 'executed': False, 'kind': kind}

    def transition(self, phase, reason):
        self.mode_sent=False
        self.phase = phase; self.reason = reason; self.state_since = time.monotonic()
        self.operation = secrets.token_hex(8)

    def stop(self, reason):
        if getattr(self,'local_mode',False) and getattr(self,'local_run',None):
            self.local_pub.publish(String(data=json.dumps(dict(kind='stop',run_id=self.local_run))))
        self.deferred_goal=None
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
        if self.phase not in ('IDLE','STOPPED','READY'):raise ValueError('Stop current task before submitting another goal')
        self.local_mode=False
        if self.phase in ('IDLE','STOPPED'):
            if time.monotonic()-self.robot_time>.3 or self.robot.get('fsm')!='loco' or not all(self.robot.get(k) for k in ('authorized','sdk_enabled','fsm_fresh','interfaces_fresh','biped')):
                raise ValueError('Navigation requires current SDK/loco feedback; robot modes remain manual')
            if not self.safe() or not self.nav.server_is_ready():raise ValueError('Navigation reference or Nav2 is not ready')
            self.deferred_goal=dict(value);self.browser=time.monotonic();self.session=secrets.token_hex(16)
            self.transition('ARMING','Attach to operator-selected loco for navigation');return
        if self.phase != 'READY':raise ValueError('Navigation chain incomplete: robot must be attached in loco READY')
        if not self.nav.server_is_ready():raise ValueError('Navigation chain incomplete: Nav2 action server is not running')
        if not self.safe():raise ValueError('Navigation chain incomplete: live body/map localization and perception status is missing/stale; this is not road approval')
        c = dict(self.config); lat, lon = value['latitude'], value['longitude']
        if self.safety.get('reference_frame')=='navigation_base':
            for k in ('datum_latitude_deg','datum_longitude_deg'):c[k]=self.safety.get(k)
        if any(type(c.get(k)) not in (int,float) or not math.isfinite(c[k]) for k in ('datum_latitude_deg','datum_longitude_deg')):
            raise ValueError('Map datum coordinates missing/invalid; this is coordinate configuration, not approval')
        if not -90<=c['datum_latitude_deg']<=90 or not -180<=c['datum_longitude_deg']<=180:
            raise ValueError('Map datum coordinates out of range')
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

    def on_local_status(self,msg):
        try:
            s=json.loads(msg.data)
            if not isinstance(s,dict):raise ValueError('Invalid local status')
            self.local_state=s;self.local_time=time.monotonic()
            if not getattr(self,'local_mode',False) or s.get('run_id')!=self.local_run:return
            if self.phase not in ('GOAL_PENDING','NAVIGATING'):return
            if s.get('state') in ('STOPPED','REJECTED','SUCCEEDED'):
                self.stop('LIO local '+s['state']+': '+s.get('reason',''));return
            if s.get('state')=='ACTIVE' and self.safe():
                v,w=velocity(s['v'],s['w'])
                if v>.150001 or abs(w)>.350001:raise ValueError('Local velocity exceeds first-stage limits')
                if self.phase=='GOAL_PENDING':self.transition('NAVIGATING','LIO full-path execution accepted')
                self.cmd=(v,w);self.cmd_time=time.monotonic()
        except (ValueError,TypeError,KeyError):
            self.local_state={}
            if getattr(self,'local_mode',False) and self.phase in ('GOAL_PENDING','NAVIGATING'):self.stop('Invalid local execution status')

    def start_local(self,value):
        if self.phase not in ('IDLE','STOPPED','READY'):raise ValueError('Stop current task first')
        s=getattr(self,'local_state',{})
        if time.monotonic()-getattr(self,'local_time',-math.inf)>.3 or s.get('ready') is not True:
            raise ValueError('Fresh complete LIO path required: '+s.get('reason','local executor unavailable'))
        if value.get('epoch')!=s.get('epoch') or value.get('goal')!=s.get('goal'):raise ValueError('Selected local goal changed; refresh preview')
        if time.monotonic()-self.robot_time>.3 or self.robot.get('fsm')!='loco' or not all(self.robot.get(k) for k in ('authorized','sdk_enabled','fsm_fresh','interfaces_fresh','biped')):
            raise ValueError('Manual SDK/loco with live receiver required')
        for p in (self.robot.get('pose',{}),self.robot.get('pose_target',{}),getattr(self,'pose',None) or {}):
            if any(abs(p.get(k,0))>1e-5 for k in ('roll_rad','pitch_rad')):raise ValueError('Neutral posture required')
        self.local_mode=True
        if not self.safe():raise ValueError('Local source status expired')
        if self.phase in ('IDLE','STOPPED'):
            self.deferred_goal=dict(value,_local=True);self.browser=time.monotonic();self.session=secrets.token_hex(16)
            self.transition('ARMING','Attach to manual loco for GPS-free LIO navigation');return
        self.local_run=secrets.token_hex(16);self.cmd=(0.,0.);self.cmd_time=0.;self.pending=False
        self.local_pub.publish(String(data=json.dumps(dict(kind='start',run_id=self.local_run,epoch=value['epoch'],goal=value['goal'],request_time=time.time()))))
        self.transition('GOAL_PENDING','Waiting for LIO executor acknowledgement')

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
                    if 'batteries' in r:
                        self.batteries=r['batteries'];self.battery_time=now
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
                        self.browser = now; self.cmd_time = 0.
                        self.pose = None
                        self.session = secrets.token_hex(16)
                        self.transition('ARMING','Request receiver session')
                    elif kind == 'navigate': self.start_goal(value)
                    elif kind == 'lio_navigate':self.start_local(value)
                    elif kind == 'loco':
                        if self.phase!='STAND_READY' or value.get('confirm_stationary') is not True:
                            raise ValueError('Confirm physically stable in STAND_READY before requesting loco')
                        self.transition('ENTERING_LOCO','Operator requested loco once')
                    elif kind == 'lower':
                        if self.phase not in ('READY','STAND_READY') or value.get('confirm_stationary') is not True:
                            raise ValueError('Stop navigation, reattach loco standby and confirm actual standstill before lowering')
                        self.pose=None;self.cmd=(0.,0.);self.transition('LOWER_BRAKING','Holding zero before lowering')
                    elif kind == 'pose':
                        if self.phase!='READY' or now-self.state_since<.7 or not self.robot.get('pose_enabled'):
                            raise ValueError('Pose requires stationary loco READY and receiver pose permission')
                        posture(value,self.robot.get('height_enabled',False));self.pose=dict(value)
                        self.reason='Posture request pending receiver confirmation'
                except (ValueError, KeyError) as exc: self.reason = str(exc)
            if now-getattr(self,'battery_query_time',-100.)>=60:
                self.send('battery');self.battery_query_time=now
            active = self.phase not in ('IDLE','STOPPED','DISABLED')
            if active and (now-self.browser>2. or now-self.robot_time>.3):
                self.stop('Browser/robot heartbeat timeout'); active = False
            if active and self.phase != 'ARMING' and (self.robot.get('fault') or not self.robot.get('fsm_fresh')):
                self.stop('Receiver fault / stale FSM'); active = False
            if self.phase == 'ARMING':
                if getattr(self,'deferred_goal',None) is not None and self.robot.get('fsm')!='loco':
                    self.stop('Manual loco state lost while attaching navigation');return
                self.send('arm')
                if self.robot.get('active') and not self.robot.get('fault') and self.robot.get('session') == self.session:
                    if self.robot.get('fsm')=='loco':
                        self.transition('READY','Attached to existing loco at zero speed')
                        goal=getattr(self,'deferred_goal',None);self.deferred_goal=None
                        if goal is not None:
                            try:
                                if goal.pop('_local',False):self.start_local(goal)
                                else:self.start_goal(goal)
                            except ValueError as exc:self.stop(str(exc))
                    elif self.robot.get('fsm')=='transform_up':
                        self.transition('STAND_READY','Already transform_up; waiting for separate loco button')
                    else:self.transition('STANDING','Request transform_up')
            elif self.phase == 'STANDING':
                if not getattr(self,'mode_sent',False):
                    self.send('drive','transform_up');self.mode_sent=True
                else:self.send('drive')
                acknowledged=(self.robot.get('operation')==self.operation and self.robot.get('last_mode')=='transform_up' and self.robot.get('fsm_valid') is True)
                if acknowledged and self.robot.get('fsm')=='transform_up':
                    self.transition('STAND_READY','transform_up received; operator must separately request loco')
            elif self.phase=='STAND_READY':
                self.send('drive')
                if self.robot.get('fsm')!='transform_up' or self.robot.get('fsm_valid') is not True:
                    self.stop('Lost transform_up state while waiting; operator review required')
            elif self.phase == 'ENTERING_LOCO':
                if not getattr(self,'mode_sent',False):
                    self.send('drive','loco');self.mode_sent=True
                else:self.send('drive')
                if self.robot.get('fsm') == 'loco' and self.robot.get('fsm_valid') is True and self.robot.get('operation')==self.operation:
                    self.transition('READY','loco, holding zero velocity')
            elif self.phase=='LOWER_BRAKING':
                self.send('drive')
                if now-self.state_since>.7:self.transition('LOWERING','Request transform_down')
            elif self.phase=='LOWERING':
                if not getattr(self,'mode_sent',False):
                    self.send('drive','transform_down',confirm_stationary=True);self.mode_sent=True
                else:self.send('drive')
                if self.robot.get('fsm')=='idle':self.stop('Lowering finished: idle feedback')
            elif self.phase in ('READY','GOAL_PENDING','NAVIGATING'):
                if self.robot.get('fsm') != 'loco': self.stop('Unexpected FSM'); return
                if getattr(self,'local_mode',False) and self.phase in ('GOAL_PENDING','NAVIGATING'):
                    self.local_pub.publish(String(data=json.dumps(dict(kind='heartbeat',run_id=self.local_run,request_time=time.time()))))
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
            if self.phase in ('ARMING','STANDING','ENTERING_LOCO','LOWERING') and now-self.state_since>10.:
                self.stop('Robot transition timeout')
            if self.phase == 'GOAL_PENDING' and now-self.state_since>2.:
                self.stop('Nav2 acknowledgement timeout')
        except OSError as exc:
            if getattr(self,'local_mode',False) and getattr(self,'local_run',None):
                self.local_pub.publish(String(data=json.dumps(dict(kind='stop',run_id=self.local_run))))
            self.generation += 1; self.cmd = (0.,0.)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async(); self.goal_handle = None
            self.transition('STOPPED','Network error: '+str(exc))

    def status(self):
        return {'phase': self.phase, 'reason': self.reason, 'navigation_safe': self.safe(),
                'robot': self.robot, 'robot_receive_age_sec': time.monotonic()-self.robot_time,
                'enabled': self.sock is not None,
                'navigation_mode':'lio_local' if getattr(self,'local_mode',False) else 'geographic',
                'batteries':getattr(self,'batteries',{}),
                'battery_receive_age_sec':time.monotonic()-self.battery_time if getattr(self,'battery_time',0) else None}

    def close(self):
        if self.sock:
            self.stop('Web gateway shutdown'); self.sock.close()
