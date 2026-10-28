"""Real deskewed LIO scan -> ground-rejected nav + bounded clearing cloud."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from d1h_nav2.patchwork_ground import PatchworkGround
from d1h_nav2.ground_health import STATUS_PERIOD_SEC


def stamp_key(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


class RealCloudFilter(Node):
    def __init__(self):
        super().__init__('real_navigation_cloud_filter')
        self.declare_parameter('sensor_height_m',0.)
        for name,value in {'patchwork_th_dist_m':.02,'patchwork_th_seeds_m':.04,'patchwork_min_range_m':.3,
                           'patchwork_max_range_m':6.,'patchwork_min_ground_points':20}.items():self.declare_parameter(name,value)
        self.filter=None;self.backend_error=None;self.processing_ms=None
        self.initialize_filter()
        self.nav=self.create_publisher(PointCloud2,'/livox/lidar/pointcloud_nav',5)
        self.clear=self.create_publisher(PointCloud2,'/livox/lidar/pointcloud_clear',5)
        self.status=self.create_publisher(String,'/navigation/cloud_filter/status',5)
        self.odom={};self.pending=None;self.last=0;self.reason='waiting for matched real LIO scan/odometry';self.last_success=0
        self.ground_plane=None;self.previous_odom=None;self.estimated_height=None
        self.subs=[self.create_subscription(Odometry,'/navigation/lio/odometry',self.on_odom,10),
                   self.create_subscription(PointCloud2,'/navigation/lio/deskewed_cloud',self.on_cloud,qos_profile_sensor_data)]
        self.timer=self.create_timer(STATUS_PERIOD_SEC,self.health)
    def initialize_filter(self):
        try:
            p=lambda n:self.get_parameter(n).value
            self.filter=PatchworkGround(p('sensor_height_m'),p('patchwork_th_dist_m'),p('patchwork_th_seeds_m'),
                                       p('patchwork_min_range_m'),p('patchwork_max_range_m'),p('patchwork_min_ground_points'))
            self.backend_error=None
        except (ValueError,RuntimeError) as exc:
            self.backend_error=str(exc);self.get_logger().error(self.backend_error)
    def on_odom(self,msg):
        if self.previous_odom is not None:
            old=self.previous_odom;dt=(stamp_key(msg)-stamp_key(old))/1e9
            p=msg.pose.pose.position;q=old.pose.pose.position
            jump=np.linalg.norm([p.x-q.x,p.y-q.y,p.z-q.z])
            if dt<=0 or dt>1. or msg.header.frame_id!=old.header.frame_id or jump>.5+2*dt:
                self.ground_plane=None;self.odom.clear();self.pending=None;self.last=0;self.last_success=0
                self.initialize_filter()
                self.reason='LIO discontinuity: require measured-height ground initialization again'
        self.previous_odom=msg
        self.odom[stamp_key(msg)]=msg
        while len(self.odom)>10:self.odom.pop(next(iter(self.odom)))
        self.process()
    def on_cloud(self,msg):self.pending=msg;self.process()
    def process(self):
        msg=self.pending
        if msg is None:return
        key=stamp_key(msg);odom=self.odom.get(key)
        if odom is None:self.reason='waiting for exact acquisition-time LIO odometry';return
        self.pending=None
        age=(self.get_clock().now().nanoseconds-key)/1e9
        if key<=self.last or not -.05<=age<=.5 or msg.header.frame_id!=odom.header.frame_id or msg.header.frame_id!='lio_odom' or odom.child_frame_id!='lio_imu':
            self.last_success=0;self.reason='stale/nonincreasing cloud or mismatched frame';return
        self.last=key
        if self.backend_error:
            self.last_success=0;self.reason=self.backend_error;return
        started=time.monotonic()
        try:
            raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
            if isinstance(raw,np.ndarray) and raw.dtype.names:
                points=np.column_stack([raw[n].ravel() for n in ('x','y','z')])
            else:points=np.asarray(list(raw),dtype=float).reshape(-1,3)
            if len(points)>100000:raise ValueError('scan exceeds bounded 100k-point budget')
            t=odom.pose.pose.position
            nav,clear,plane=self.filter.split(points,(t.x,t.y,t.z))
            fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
            def output(xyz):
                out=PointCloud2();out.header=msg.header;out.height=1;out.width=len(xyz);out.fields=fields
                out.is_bigendian=False;out.point_step=12;out.row_step=12*len(xyz);out.is_dense=True;out.data=xyz.tobytes();return out
            self.nav.publish(output(nav));self.clear.publish(output(clear))
            self.ground_plane=plane
            self.estimated_height=float(t.z-(plane[0]*t.x+plane[1]*t.y+plane[2]))
            self.last_success=key;self.reason=f'ground accepted; nav={len(nav)} clear={len(clear)}'
        except (ValueError,TypeError,IndexError,RuntimeError) as exc:self.last_success=0;self.reason=str(exc)
        finally:self.processing_ms=(time.monotonic()-started)*1000
    def health(self):
        age=(self.get_clock().now().nanoseconds-self.last_success)/1e9 if self.last_success else None
        self.status.publish(String(data=json.dumps({'ready':age is not None and age<.5,'reason':self.backend_error or self.reason,
            'measurement_age_sec':age,'frame':'native LIO frame; base_link TF not supplied here',
            'estimated_imu_height_m':self.estimated_height,'ground_initialized':self.ground_plane is not None,
            'backend':'official_patchworkpp','processing_ms':self.processing_ms,
            'parameters':self.filter.configuration if self.filter else {},
            'diagnostics':self.filter.diagnostics if self.filter else {},
            'scope':'locally planar trials only; sensor-radius exclusion, not qualified full-body self-filter'})))


def main():
    rclpy.init();node=RealCloudFilter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
