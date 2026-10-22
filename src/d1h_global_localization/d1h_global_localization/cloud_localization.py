"""Experimental scan-to-prior localization; never consumes Gazebo truth."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from .icp import downsample,register


def load_map(path):
    with open(path,'rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Invalid PCD header')
            if line.strip()==b'DATA binary':
                break
        # This project-generated map has XYZ float32 only.
        return np.frombuffer(stream.read(),dtype='<f4').reshape(-1,3).copy()


def cloud(points,stamp):
    out = PointCloud2()
    out.header.frame_id='map'; out.header.stamp=stamp
    out.height=1;out.width=len(points)
    out.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')]
    out.point_step=12;out.row_step=12*len(points);out.is_dense=True
    out.data=np.asarray(points,dtype='<f4').tobytes()
    return out


class CloudLocalization(Node):
    def __init__(self):
        super().__init__('cloud_localization')
        for key,value in dict(map_path='',rate_hz=2.,initial_x=0.,initial_y=0.,initial_z=.15,
                              initial_roll=0.,initial_pitch=0.,initial_yaw=0.,max_scan_age_sec=1.,
                              ground_exclusion_m=.16).items():
            self.declare_parameter(key,value)
        self.reference=downsample(load_map(self.p('map_path')),.08)
        self.reference=self.reference[self.reference[:,2]>=self.p('ground_exclusion_m')]
        self.tree=cKDTree(self.reference)
        self.estimate=np.eye(4)
        self.reset()
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.broadcaster=TransformBroadcaster(self)
        self.latest=None;self.last_stamp=None;self.last_good=None;self.failures=0
        self.sub=self.create_subscription(PointCloud2,'/livox/lidar/pointcloud',self.on_scan,qos_profile_sensor_data)
        self.initial=self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initial,10)
        self.odom=self.create_publisher(Odometry,'/localization/odometry',5)
        self.aligned=self.create_publisher(PointCloud2,'/localization/aligned_cloud',2)
        self.status=self.create_publisher(String,'/localization/status',5)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(PointCloud2,'/localization/reference_map',qos)
        self.map_pub.publish(cloud(self.reference,self.get_clock().now().to_msg()))
        self.timer=self.create_timer(1/self.p('rate_hz'),self.process)
        self.watchdog=self.create_timer(.5,self.check_stale)
        self.get_logger().info(f'ICP prototype: {len(self.reference)} map points; manual initial pose required; NO Gazebo odometry subscription')

    def p(self,key):
        return self.get_parameter(key).value

    def reset(self):
        self.estimate[:3,:3]=Rotation.from_euler('xyz',[self.p('initial_'+k) for k in ('roll','pitch','yaw')]).as_matrix()
        self.estimate[:3,3]=[self.p('initial_'+k) for k in 'xyz']

    def on_initial(self,msg):
        if msg.header.frame_id!='map':
            self.get_logger().warning('Initial pose must be in map frame');return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if np.linalg.norm([q.x,q.y,q.z,q.w])<.5:
            return
        self.estimate[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        self.estimate[:3,3]=[p.x,p.y,p.z]
        self.last_good=None;self.failures=0

    def on_scan(self,msg):
        self.latest=msg

    def report(self,state,**values):
        self.status.publish(String(data=json.dumps(dict(state=state,failures=self.failures,**values))))

    def check_stale(self):
        if self.latest is None or (self.get_clock().now()-Time.from_msg(self.latest.header.stamp)).nanoseconds/1e9>self.p('max_scan_age_sec'):
            self.report('STALE_NO_VALID_LOCALIZATION')

    def process(self):
        msg=self.latest
        if msg is None:return
        stamp=Time.from_msg(msg.header.stamp).nanoseconds
        if stamp==self.last_stamp:return
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        if not -.1<=age<=self.p('max_scan_age_sec'):return
        try:
            tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except TransformException:return
        self.last_stamp=stamp
        fields={f.name:f for f in msg.fields}
        if not all(k in fields and fields[k].datatype==PointField.FLOAT32 for k in 'xyz'):return
        dtype=np.dtype(dict(names=list('xyz'),formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                            offsets=[fields[k].offset for k in 'xyz'],itemsize=msg.point_step))
        raw=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
        points=np.column_stack([raw[k].ravel() for k in 'xyz'])
        ranges=np.linalg.norm(points,axis=1)
        points=points[np.isfinite(points).all(axis=1)&(ranges>.6)&(ranges<20)]
        t,q=tf.transform.translation,tf.transform.rotation
        points=points@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix().T+[t.x,t.y,t.z]
        # Use the last estimated 6DoF pose, never Gazebo truth, for map-height
        # ground rejection. Large initialization errors still require reinit.
        estimated_world=points@self.estimate[:3,:3].T+self.estimate[:3,3]
        points=points[estimated_world[:,2]>=self.p('ground_exclusion_m')]
        started=time.monotonic()
        candidate,quality=register(points,self.tree,self.estimate)
        delta=np.linalg.norm(candidate[:3,3]-self.estimate[:3,3])
        angle=Rotation.from_matrix(candidate[:3,:3]@self.estimate[:3,:3].T).magnitude()
        accepted=quality['accepted'] and delta<1. and angle<.8
        quality['accepted']=bool(accepted)
        if not accepted:
            self.failures+=1
            self.report('LOST_REINITIALIZE' if self.failures>=3 else 'REJECTED',**quality)
            return
        self.estimate=candidate;self.failures=0;self.last_good=stamp
        rotation=Rotation.from_matrix(candidate[:3,:3]).as_quat()
        odom=Odometry();odom.header.frame_id='map';odom.header.stamp=msg.header.stamp
        odom.child_frame_id='localization_base_link'
        p=odom.pose.pose.position;p.x,p.y,p.z=map(float,candidate[:3,3])
        q=odom.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,rotation)
        # No calibrated covariance or velocity estimate: prototype not for Nav2.
        odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=1.
        odom.pose.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=1.
        self.odom.publish(odom)
        transform=TransformStamped();transform.header=odom.header;transform.child_frame_id=odom.child_frame_id
        transform.transform.translation.x=p.x;transform.transform.translation.y=p.y;transform.transform.translation.z=p.z
        transform.transform.rotation=q;self.broadcaster.sendTransform(transform)
        self.aligned.publish(cloud(points@candidate[:3,:3].T+candidate[:3,3],msg.header.stamp))
        self.report('TRACKING_PROTOTYPE',**quality,processing_sec=time.monotonic()-started,
                    position=candidate[:3,3].tolist())


def main():
    rclpy.init();node=CloudLocalization()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()
