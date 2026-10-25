"""Real sensor-derived navigation reference; never commands robot modes/velocity.

navigation_base is a VIRTUAL yaw-only reference under the IMU, not vendor base_link.
GPS+manual bearing registration is an initial anchor, NOT continuous GNSS fusion.
"""
import json
import math
import time
import uuid
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from d1h_nav2.navigation_reference import heading,registration
from d1h_nav2.osm_roads import project


def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9


class NavigationReference(Node):
    def __init__(self):
        super().__init__('navigation_reference')
        for k,v in {'profile':'','sensor_height_m':.46}.items():self.declare_parameter(k,v)
        path=self.get_parameter('profile').value
        p=json.load(open(path,encoding='utf-8')) if path else {}
        self.datum=(float(p.get('datum_latitude_deg',34.125)),float(p.get('datum_longitude_deg',108.829)))
        self.height=float(self.get_parameter('sensor_height_m').value)
        if not .15<=self.height<=2 or not -80<=self.datum[0]<=80 or not -180<=self.datum[1]<=180:raise ValueError('Invalid height/datum')
        self.epoch=uuid.uuid4().hex;self.anchor=None;self.odom=None;self.odom_rx=-math.inf
        self.odoms=[];self.gps=[];self.cloud={};self.cloud_rx=-math.inf;self.memory={};self.memory_rx=-math.inf
        self.reason='waiting for sensors and stationary GPS/bearing registration'
        self.tf=TransformBroadcaster(self)
        self.pub=self.create_publisher(Odometry,'/odometry/local',5)
        self.status=self.create_publisher(String,'/real_navigation/safety_status',5)
        self.subs=[self.create_subscription(Odometry,'/navigation/lio/odometry',self.on_odom,10),
            self.create_subscription(NavSatFix,'/hardware/gnss/fix',self.on_gps,qos_profile_sensor_data),
            self.create_subscription(String,'/navigation/reference/register',self.on_register,1)]
        for topic,key in [('/navigation/cloud_filter/status','cloud'),('/navigation/memory/status','memory')]:
            self.subs.append(self.create_subscription(String,topic,lambda m,k=key:self.on_status(k,m),5))
        self.timer=self.create_timer(.1,self.health)

    def on_status(self,key,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Expected status object')
            setattr(self,key,value);setattr(self,key+'_rx',time.monotonic())
        except (ValueError,TypeError):setattr(self,key,{})

    def on_gps(self,msg):
        if msg.status.status>=0 and all(math.isfinite(v) for v in (msg.latitude,msg.longitude)) and -90<=msg.latitude<=90 and -180<=msg.longitude<=180:
            self.gps.append(msg);self.gps=self.gps[-10:]

    def fresh_pose(self):
        return self.odom is not None and time.monotonic()-self.odom_rx<.4 and -.05<=self.get_clock().now().nanoseconds/1e9-stamp(self.odom)<.4

    def on_register(self,msg):
        try:
            if len(self.get_publishers_info_by_topic('/navigation/lio/odometry'))>1 or any(p.node_name!='navigation_reference' for p in self.get_publishers_info_by_topic('/odometry/local')):
                raise ValueError('Conflicting odometry publisher; stop duplicate localization instances')
            r=json.loads(msg.data)
            if set(r)!={'heading_deg','lio_stamp','gps_stamp','request_time'}:raise ValueError('Invalid registration schema')
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in r.values()):raise ValueError('Non-finite registration')
            if not -.1<=time.time()-r['request_time']<=3 or not self.fresh_pose():raise ValueError('Stale registration/pose')
            o=next((o for o in reversed(self.odoms) if abs(stamp(o)-r['lio_stamp'])<1e-6),None)
            g=next((g for g in reversed(self.gps) if abs(stamp(g)-r['gps_stamp'])<1e-6),None)
            now=self.get_clock().now().nanoseconds/1e9
            if o is None or g is None or abs(stamp(o)-stamp(g))>.3 or not -.05<=now-stamp(g)<2 or not -.05<=now-stamp(o)<1:
                raise ValueError('No fresh matched GPS/LIO registration samples')
            q=o.pose.pose.orientation;p=o.pose.pose.position
            self.anchor=registration(project(g.latitude,g.longitude,*self.datum),(p.x,p.y),heading((q.x,q.y,q.z,q.w)),r['heading_deg'])
            self.epoch=uuid.uuid4().hex
            self.reason='GPS/manual-bearing anchor registered; no continuous GPS correction'
        except (ValueError,TypeError,KeyError) as exc:self.reason='Registration rejected: '+str(exc)

    def on_odom(self,msg):
        now=time.monotonic();p=msg.pose.pose.position;q=msg.pose.pose.orientation
        try:
            outputs=self.get_publishers_info_by_topic('/odometry/local')
            if len(self.get_publishers_info_by_topic('/navigation/lio/odometry'))>1 or len(outputs)>1 or any(p.node_name!='navigation_reference' for p in outputs):
                raise ValueError('Conflicting odometry publisher; stop duplicate localization instances')
            if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu':raise ValueError('Unexpected native LIO frames')
            yaw=heading((q.x,q.y,q.z,q.w))
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):raise ValueError('Invalid position')
            age=self.get_clock().now().nanoseconds/1e9-stamp(msg)
            if not -.05<=age<.4:raise ValueError('LIO measurement expired')
            if self.odom:
                old=self.odom;op=old.pose.pose.position;dt=stamp(msg)-stamp(old)
                if dt<=0 or dt>1 or now-self.odom_rx>1 or math.dist((p.x,p.y,p.z),(op.x,op.y,op.z))>.5+2*max(0,dt):
                    self.anchor=None;self.odoms=[];self.epoch=uuid.uuid4().hex
                    self.reason='LIO discontinuity: registration discarded; register again while stationary'
            self.odom=msg;self.odom_rx=now;self.odoms=(self.odoms+[msg])[-20:]
            transforms=[]
            def tf(parent,child,xyz,quat):
                t=TransformStamped();t.header.stamp=msg.header.stamp;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
                t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,quat)
                transforms.append(t)
            # Fixed datum ground plane for planar navigation; body reference is projected IMU.
            tf('odom','lio_odom',(0,0,self.height),(0,0,0,1))
            tf('lio_odom','lio_imu',(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            tf('odom','navigation_base',(p.x,p.y,0),(0,0,math.sin(yaw/2),math.cos(yaw/2)))
            tf('lio_imu','navigation_sensor',(.04165,.02326,-.0284),(0,0,0,1))
            if self.anchor:
                x,y,a=self.anchor;tf('map','odom',(x,y,0),(0,0,math.sin(a/2),math.cos(a/2)))
            self.tf.sendTransform(transforms)
            out=copy.deepcopy(msg);out.header.frame_id='odom';out.child_frame_id='navigation_base'
            out.pose.pose.position.z=0.;out.pose.pose.orientation.x=0.;out.pose.pose.orientation.y=0.
            out.pose.pose.orientation.z=math.sin(yaw/2);out.pose.pose.orientation.w=math.cos(yaw/2)
            x,y,z,w=q.x,q.y,q.z,q.w
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            c,s=math.cos(yaw),math.sin(yaw)
            basis=np.array([[c,s,0],[-s,c,0],[0,0,1]])@rotation
            v=msg.twist.twist.linear;g=msg.twist.twist.angular
            linear=basis@np.array([v.x,v.y,v.z]);angular=basis@np.array([g.x,g.y,g.z])
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,linear)
            out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,angular)
            transform=np.zeros((6,6));transform[:3,:3]=basis;transform[3:,3:]=basis
            out.twist.covariance=(transform@np.asarray(msg.twist.covariance).reshape(6,6)@transform.T).reshape(-1).tolist()
            if not np.isfinite(linear).all() or not np.isfinite(angular).all() or not np.isfinite(out.twist.covariance).all():raise ValueError('Invalid twist/covariance')
            # Covariance belongs to the native sensor estimate, not a calibrated body model.
            out.pose.covariance[14]=max(out.pose.covariance[14],.1)
            out.pose.covariance[21]=max(out.pose.covariance[21],.1);out.pose.covariance[28]=max(out.pose.covariance[28],.1)
            self.pub.publish(out)
        except ValueError as exc:
            self.anchor=None;self.odom_rx=-math.inf;self.reason=str(exc)

    def health(self):
        now=time.monotonic();pose=self.fresh_pose()
        if not pose and self.anchor is not None:
            self.anchor=None;self.epoch=uuid.uuid4().hex;self.reason='LIO expired: registration discarded; register again'
        age=self.cloud.get('measurement_age_sec')
        cloud=now-self.cloud_rx<.6 and self.cloud.get('ready') is True and type(age) in (int,float) and -.05<=age<.5
        t=self.memory.get('source_stamp')
        memory=now-self.memory_rx<1 and self.memory.get('ready') is True and self.memory.get('frame')=='lio_odom' and type(t) in (int,float) and -.05<=self.get_clock().now().nanoseconds/1e9-t<1.
        ready=pose and cloud and memory and self.anchor is not None
        self.status.publish(String(data=json.dumps(dict(stamp=self.get_clock().now().nanoseconds/1e9,
            navigation_ready=ready,navigation_base_ready=pose,base_link_ready=False,map_alignment_verified=self.anchor is not None,
            reference_epoch=self.epoch,registered=self.anchor is not None,reason=self.reason,
            datum_latitude_deg=self.datum[0],datum_longitude_deg=self.datum[1],reference_frame='navigation_base',
            model='virtual ground-projected IMU; +X operator-confirmed forward; not vendor base_link',
            anchor_kind='GPS + manual initial bearing, not continuous fusion'))))


def main():
    rclpy.init();node=NavigationReference()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
