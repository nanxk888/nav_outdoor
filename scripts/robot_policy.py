"""Pure D1 receive-side state and validation. No ROS, sockets or hardware."""
import math
import secrets
from robot_wire import velocity


def valid_fsm_event(event, online, required_source_ns, now):
    """Require a settled writer history and a post-command DDS source timestamp."""
    return bool(online and event is not None and now-event[2]>.3 and event[1]>required_source_ns)


def posture(value, allow_height=False):
    if not isinstance(value, dict) or set(value)-{'roll_rad','pitch_rad','height_m'}:
        raise ValueError('invalid posture fields')
    roll, pitch = value.get('roll_rad',0.), value.get('pitch_rad',0.)
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in (roll,pitch)):
        raise ValueError('nonfinite posture')
    if abs(roll)>.2 or abs(pitch)>.4: raise ValueError('roll/pitch limit exceeded')
    height=value.get('height_m')
    if height is not None:
        if not allow_height or type(height) not in (int,float) or not math.isfinite(height) or not .1<=height<=.3:
            raise ValueError('height not verified or out of range')
    # ROS quaternion (roll,pitch,yaw=0), radians.
    sr,cr,sp,cp=math.sin(roll/2),math.cos(roll/2),math.sin(pitch/2),math.cos(pitch/2)
    return {'roll_rad':roll,'pitch_rad':pitch,'height_m':height,
            'xyzw':[sr*cp,cr*sp,-sr*sp,cr*cp]}


class ReceiverPolicy:
    def __init__(self, authorized=False, allow_pose=False, allow_height=False):
        self.authorized=authorized;self.allow_pose=allow_pose;self.allow_height=allow_height
        self.nonce=secrets.token_hex(16);self.session=None;self.seq=-1
        self.active=False;self.fault=True;self.last_rx=-100.;self.operation=None
        self.command=(0.,0.);self.output=(0.,0.);self.pending_mode=''
        self.pose=posture({});self.pose_target=self.pose.copy();self.fsm='';self.fsm_time=-100.
        self.sdk=False;self.biped=False;self.interface_time=-100.
        self.zero_since=None;self.last_mode='';self.reason='not armed'

    def feedback(self, fsm, now): self.fsm=fsm.strip();self.fsm_time=now
    def interfaces(self, sdk, biped, now):
        self.sdk=sdk;self.biped=biped;self.interface_time=now
    def ready(self, now):
        return self.authorized and self.sdk and self.biped and now-self.interface_time<2. and now-self.fsm_time<1.
    def halt(self, reason):
        self.active=False;self.fault=True;self.command=(0.,0.);self.output=(0.,0.)
        self.pending_mode='';self.nonce=secrets.token_hex(16);self.reason=reason
        self.pose_target=self.pose.copy()

    def accept(self, data, now):
        if data.get('protocol')!=2: raise ValueError('requires paired protocol v2')
        kind=data.get('kind')
        if kind=='hello': return
        if data.get('nonce')!=self.nonce or type(data.get('seq')) is not int:
            raise ValueError('nonce/sequence mismatch')
        mode=data.get('mode','');op=data.get('operation')
        if kind not in ('arm','drive','stop') or mode not in ('','transform_up','transform_down','loco'):
            raise ValueError('forbidden command')
        proposed=velocity(data.get('v',0.),data.get('w',0.))
        pose=None
        if 'pose' in data:
            if not self.allow_pose: raise ValueError('pose control disabled')
            pose=posture(data['pose'],self.allow_height)
            if mode or proposed!=(0.,0.) or self.fsm!='loco': raise ValueError('pose requires stationary command in loco')
        if mode and (not isinstance(op,str) or not 1<=len(op)<=64 or proposed!=(0.,0.)):
            raise ValueError('mode requires operation ID and zero command')
        if kind=='arm':
            if proposed!=(0.,0.) or mode or pose is not None or not self.ready(now):
                raise ValueError('cannot arm: SDK/biped/FSM/neutral conditions')
            if self.active and data.get('session')!=self.session: raise ValueError('another owner is active')
        elif not self.active or data.get('session')!=self.session:
            raise ValueError('no active session')
        if not isinstance(data.get('session'),str) or not 1<=len(data['session'])<=64:
            raise ValueError('invalid session')
        new_session=kind=='arm' and not self.active
        if not new_session and data['seq']<=self.seq: raise ValueError('replayed sequence')
        if kind!='stop' and not self.ready(now): raise ValueError('SDK/biped/FSM stale or not ready')
        if proposed!=(0.,0.) and (self.fsm!='loco' or self.last_mode=='transform_down'):
            raise ValueError('drive requires actual loco and no lowering request')
        if proposed!=(0.,0.) and any(abs(p[k])>1e-5 for p in (self.pose,self.pose_target) for k in ('roll_rad','pitch_rad')):
            raise ValueError('restore neutral roll/pitch before driving')
        new_mode=bool(mode) and op!=self.operation
        if new_mode and mode=='transform_up' and self.fsm!='idle':
            raise ValueError('standing request requires actual idle feedback')
        if new_mode and mode=='transform_down':
            if data.get('confirm_stationary') is not True or self.zero_since is None or now-self.zero_since<.5:
                raise ValueError('lowering requires operator stationary confirmation and 0.5s zero command')
            if any(abs(self.pose[k])>1e-5 or abs(self.pose_target[k])>1e-5 for k in ('roll_rad','pitch_rad')):
                raise ValueError('restore neutral roll/pitch before lowering')
        if pose is not None and (self.zero_since is None or now-self.zero_since<.5 or self.last_mode=='transform_down'):
            raise ValueError('pose requires 0.5s zero command and no lowering')
        # All checks complete before any state mutation.
        if new_session:
            self.session=data['session'];self.active=True;self.fault=False
            self.zero_since=now;self.last_mode='';self.operation=None
        self.seq=data['seq'];self.last_rx=now
        if kind=='stop': self.halt('operator stop');return
        self.command=proposed;self.reason='command accepted'
        if proposed!=(0.,0.): self.zero_since=None
        elif self.zero_since is None: self.zero_since=now
        if new_mode:
            self.operation=op;self.pending_mode=mode;self.last_mode=mode
        if pose is not None:self.pose_target=pose

    def expire(self, now):
        if self.active and (now-self.last_rx>.2 or not self.ready(now)):
            self.halt('command timeout or SDK/FSM unavailable')

    def tick(self, now):
        self.expire(now)
        moving=self.active and not self.fault and self.fsm=='loco' and self.last_mode!='transform_down' and self.command!=(0.,0.)
        self.output=tuple(old+max(-step,min(step,target-old)) for old,target,step in zip(self.output,self.command,(.015,.04))) if moving else (0.,0.)
        if self.active and self.fsm=='loco' and self.command==(0.,0.) and self.last_mode!='transform_down':
            value={k:self.pose[k]+max(-.025,min(.025,self.pose_target[k]-self.pose[k])) for k in ('roll_rad','pitch_rad')}
            if self.pose_target['height_m'] is not None:
                old=self.pose['height_m']
                if old is None:old=self.pose_target['height_m']
                value['height_m']=old+max(-.0025,min(.0025,self.pose_target['height_m']-old))
            self.pose=posture(value,self.allow_height)
        mode=self.pending_mode if self.active else '';self.pending_mode=''
        return self.output, mode

    def status(self, now):
        return {'protocol':2,'nonce':self.nonce,'fsm':self.fsm,'fsm_fresh':now-self.fsm_time<1.,
                'authorized':self.authorized,'sdk_enabled':self.sdk,'biped':self.biped,
                'interfaces_fresh':now-self.interface_time<2.,'active':self.active,'fault':self.fault,
                'session':self.session,'ack':self.seq,'operation':self.operation,'last_mode':self.last_mode,
                'pose_enabled':self.allow_pose,'height_enabled':self.allow_height,
                'pose':self.pose,'pose_target':self.pose_target,'reason':self.reason}
