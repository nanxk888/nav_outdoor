"""GPS-free 1-2m full-path executor; only the coordinator can forward to UDP."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String
from d1h_nav2.navigation_reference import body_heading
from d1h_nav2.local_navigation import track


class LocalNavigation(Node):
    def __init__(self):
        super().__init__('lio_local_navigation')
        self.odom=None;self.memory={};self.ground={};self.grid=None;self.rx={}
        self.run_id=None;self.target=None;self.path=None;self.epoch=None;self.state='IDLE';self.reason='Waiting for full local path'
        self.started=0.;self.last_request=0.;self.lease_rx=-math.inf
        self.pub=self.create_publisher(String,'/navigation/local_execution/status',5)
        self.subs=[self.create_subscription(Odometry,'/navigation/lio/odometry',lambda m:self.cache('odom',m),5),
                   self.create_subscription(OccupancyGrid,'/navigation/memory/grid',lambda m:self.cache('grid',m),1),
                   self.create_subscription(String,'/navigation/local_execution/request',self.request,5)]
        for key,topic in [('memory','/navigation/memory/status'),('ground','/navigation/cloud_filter/status')]:
            self.subs.append(self.create_subscription(String,topic,lambda m,k=key:self.status_input(k,m),5))
        self.timer=self.create_timer(.05,self.tick)

    def cache(self,key,value):
        if key=='odom' and self.odom is not None and self.state=='ACTIVE':
            old=self.odom;p=value.pose.pose.position;q=old.pose.pose.position
            dt=(value.header.stamp.sec-old.header.stamp.sec)+(value.header.stamp.nanosec-old.header.stamp.nanosec)/1e9
            try:
                a=value.pose.pose.orientation;b=old.pose.pose.orientation
                da=body_heading((a.x,a.y,a.z,a.w))-body_heading((b.x,b.y,b.z,b.w))
                yaw_jump=abs(math.atan2(math.sin(da),math.cos(da)))>.3+2*max(0,dt)
            except ValueError:yaw_jump=True
            if dt<=0 or dt>1 or yaw_jump or math.dist((p.x,p.y,p.z),(q.x,q.y,q.z))>.5+2*max(0,dt):
                self.state='STOPPED';self.reason='LIO discontinuity; explicit new task required'
        setattr(self,key,value);self.rx[key]=time.monotonic()

    def status_input(self,key,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Invalid status')
            self.cache(key,value)
        except (ValueError,TypeError):setattr(self,key,{})

    def inputs(self):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
        if any(len(self.get_publishers_info_by_topic(topic))>1 for topic in ('/navigation/lio/odometry','/navigation/memory/grid','/navigation/memory/status')):
            raise ValueError('Conflicting LIO/memory publishers; stop duplicate instances')
        def fresh(key,limit):return now-self.rx.get(key,-math.inf)<limit
        def age(msg):return ros-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if not fresh('odom',.4) or self.odom is None or not -.05<=age(self.odom)<.4:raise ValueError('LIO stale')
        if self.odom.header.frame_id!='lio_odom' or self.odom.child_frame_id!='lio_imu':raise ValueError('Unexpected LIO frames')
        if not fresh('ground',.5) or self.ground.get('ready') is not True or not -.05<=float(self.ground.get('measurement_age_sec',99))+now-self.rx.get('ground',-math.inf)<.5:raise ValueError('Ground filtering unavailable')
        m=self.memory
        if not fresh('memory',1.) or m.get('ready') is not True or m.get('frame')!='lio_odom' or not -.05<=ros-float(m.get('source_stamp',0))<1.:raise ValueError('Observed memory stale')
        if not fresh('grid',1.) or self.grid is None or not -.05<=age(self.grid)<1. or self.grid.header.frame_id!='lio_odom':raise ValueError('Observed grid stale/mismatched')
        if abs(m['source_stamp']-(self.grid.header.stamp.sec+self.grid.header.stamp.nanosec/1e9))>.3:raise ValueError('Grid/memory source mismatch')
        g=m.get('goal');r=m.get('route')
        if not isinstance(g,list) or len(g)!=2 or not isinstance(r,dict) or r.get('partial') is not False or not isinstance(r.get('points'),list) or not r['points']:
            raise ValueError('Complete path required; partial frontier is preview only')
        if self.state=='ACTIVE' and (m.get('epoch')!=self.epoch or list(g)!=list(self.target)):raise ValueError('Memory reset or selected goal changed')
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        xyz=(p.x,p.y);yaw=body_heading((q.x,q.y,q.z,q.w))
        i=self.grid.info;o=i.origin
        if abs(o.orientation.w-1)>1e-6 or abs(o.orientation.x)+abs(o.orientation.y)+abs(o.orientation.z)>1e-6:raise ValueError('Nonplanar grid origin')
        points=self.path if self.state=='ACTIVE' else r['points']
        v,w,done=track(xyz,yaw,g,points,(i.resolution,i.width,i.height,o.position.x,o.position.y,self.grid.data))
        return v,w,done,xyz

    def request(self,msg):
        try:
            r=json.loads(msg.data)
            if not isinstance(r,dict):raise ValueError('Invalid local request object')
            if r.get('kind')=='heartbeat':
                t=r.get('request_time')
                if r.get('run_id')==self.run_id and self.state=='ACTIVE' and type(t) in (int,float) and -.1<=time.time()-t<.3:
                    self.lease_rx=time.monotonic()
                return
            if r.get('kind')=='stop':
                if r.get('run_id')==self.run_id and self.state=='ACTIVE':self.state='STOPPED';self.reason='Operator stop'
                return
            if set(r)!={'kind','run_id','epoch','goal','request_time'} or r['kind']!='start':raise ValueError('Invalid local request')
            if type(r['request_time']) not in (int,float) or not -.1<=time.time()-r['request_time']<2 or r['request_time']<=self.last_request:raise ValueError('Stale/replayed local start')
            if not isinstance(r['run_id'],str) or not 16<=len(r['run_id'])<=64:raise ValueError('Invalid run id')
            if self.state=='ACTIVE':raise ValueError('Already active')
            _,_,_,p=self.inputs()
            if r['epoch']!=self.memory.get('epoch') or r['goal']!=self.memory.get('goal'):raise ValueError('Selected local goal changed')
            if not .15<math.dist(p,r['goal'])<=2.5:raise ValueError('First stage goal must be 0.15..2.5m away')
            self.run_id=r['run_id'];self.target=list(r['goal']);self.epoch=r['epoch'];self.state='ACTIVE';self.reason='Tracking full observed path'
            self.path=[tuple(p[:2]) for p in self.memory['route']['points']]
            self.started=time.monotonic();self.lease_rx=self.started;self.last_request=r['request_time']
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            # Return the requesting id so the coordinator can report rejection.
            if self.state!='ACTIVE':
                self.run_id=r.get('run_id') if isinstance(locals().get('r'),dict) else None
                self.state='REJECTED';self.reason=str(exc)

    def tick(self):
        ready=False;v=w=0.
        try:
            a,b,done,_=self.inputs();ready=True
            if self.state in ('IDLE','READY'):
                self.state='READY';self.reason='Full observed path ready; explicit start required'
            if self.state=='ACTIVE':
                if time.monotonic()-self.lease_rx>.3:raise ValueError('Local execution session lease expired')
                if time.monotonic()-self.started>60:raise ValueError('Local task timeout')
                if done:self.state='SUCCEEDED';self.reason='Local goal reached; holding zero'
                else:v,w=a,b
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self.reason=str(exc)
            if self.state=='ACTIVE':self.state='STOPPED'
            elif self.state=='READY':self.state='IDLE'
        self.pub.publish(String(data=json.dumps(dict(stamp=self.get_clock().now().nanoseconds/1e9,ready=ready,
            state=self.state,reason=self.reason,run_id=self.run_id,epoch=self.memory.get('epoch'),goal=self.memory.get('goal'),
            v=v,w=w,gps_required=False))))


def main():
    rclpy.init();node=LocalNavigation()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
