"""Native LIO observation memory + candidate paths. NEVER publishes velocity/TF."""
import json
import math
import time
import uuid
from array import array
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry,OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from d1h_nav2.observed_memory import ObservedMemory


def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


class LioMemoryNode(Node):
    def __init__(self):
        super().__init__('lio_observed_memory')
        for k,v in {'resolution_m':.25,'retention_sec':300.,'footprint_radius_m':.25}.items():self.declare_parameter(k,v)
        self.memory=ObservedMemory(self.get_parameter('resolution_m').value,self.get_parameter('retention_sec').value,
                                   radius=self.get_parameter('footprint_radius_m').value)
        self.bundles={};self.previous=None;self.frame='';self.last_scan=0;self.origin=(0.,0.);self.goal=None
        self.epoch=uuid.uuid4().hex;self.reason='waiting for exact-time LIO + nav + clear'
        self.ground={};self.ground_rx=-math.inf;self.scan_rx=-math.inf
        self.status=self.create_publisher(String,'/navigation/memory/status',1)
        self.grid_pub=self.create_publisher(OccupancyGrid,'/navigation/memory/grid',1)
        self.path_pub=self.create_publisher(Path,'/navigation/memory/path_candidate',1)
        self.subs=[self.create_subscription(Odometry,'/navigation/lio/odometry',self.on_odom,10),
            self.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',lambda m:self.cache('nav',m),qos_profile_sensor_data),
            self.create_subscription(PointCloud2,'/livox/lidar/pointcloud_clear',lambda m:self.cache('clear',m),qos_profile_sensor_data),
            self.create_subscription(String,'/navigation/cloud_filter/status',self.on_ground,5),
            self.create_subscription(String,'/navigation/memory/goal_candidate',self.on_goal,1),
            self.create_subscription(String,'/navigation/memory/reset',self.on_reset,1)]
        self.timer=self.create_timer(.5,self.tick)

    def cache(self,name,msg):
        self.bundles.setdefault(stamp(msg),{})[name]=msg
        while len(self.bundles)>15:self.bundles.pop(next(iter(self.bundles)))

    def reset(self):
        self.memory.cells.clear();self.memory.time=None;self.goal=None;self.bundles.clear();self.last_scan=0
        self.scan_rx=-math.inf
        self.epoch=uuid.uuid4().hex;self.reason='LIO reset/gap: memory and goal cleared'
        # Explicitly invalidate downstream cached candidates/grids immediately.
        path=Path();path.header.frame_id=self.frame;self.path_pub.publish(path)
        self.publish_grid()  # Unknown cells, zero source stamp: not new sensor evidence.

    def on_odom(self,msg):
        p=msg.pose.pose.position;now=time.monotonic()
        if self.previous:
            old,rx=self.previous;dt=(stamp(msg)-stamp(old))/1e9;q=old.pose.pose.position
            if dt<=0 or dt>1 or now-rx>1 or old.header.frame_id!=msg.header.frame_id or math.dist((p.x,p.y,p.z),(q.x,q.y,q.z))>.5+2*max(0,dt):self.reset()
        self.previous=(msg,now);self.frame=msg.header.frame_id
        self.cache('odom',msg)

    def on_reset(self,msg):
        try:
            request=json.loads(msg.data)
            if set(request)!={'epoch','request_time'} or request['epoch']!=self.epoch:raise ValueError('Reset epoch mismatch')
            t=request['request_time']
            if type(t) not in (int,float) or not math.isfinite(t) or not -.1<=time.time()-t<=3:raise ValueError('Reset request expired')
            self.reset();self.reason='Operator cleared observation memory and goal; LIO odometry unchanged'
        except (ValueError,TypeError,KeyError) as exc:self.reason='Reset rejected: '+str(exc)

    def on_ground(self,msg):
        try:self.ground=json.loads(msg.data);self.ground_rx=time.monotonic()
        except ValueError:self.ground={}

    def ready(self):
        return time.monotonic()-self.scan_rx<1 and time.monotonic()-self.ground_rx<1 and self.ground.get('ready') is True

    def on_goal(self,msg):
        try:
            goal=json.loads(msg.data)
            if set(goal)!={'x','y','frame','epoch','request_time'}:raise ValueError('Invalid goal schema')
            if not self.ready() or goal['frame']!=self.frame or goal['epoch']!=self.epoch:raise ValueError('Stale/mismatched memory epoch')
            if any(type(goal[k]) not in (int,float) or not math.isfinite(goal[k]) for k in ('x','y','request_time')):raise ValueError('Invalid goal numbers')
            if not -.1<=time.time()-goal['request_time']<=3 or math.dist(self.origin,(goal['x'],goal['y']))>100:raise ValueError('Stale/too distant goal')
            self.goal=(goal['x'],goal['y'],time.monotonic());self.reason='memory goal accepted for candidate planning only'
        except (ValueError,TypeError,KeyError) as exc:self.reason='Goal rejected: '+str(exc)

    @staticmethod
    def points(msg):
        if msg.width*msg.height>100000:raise ValueError('Point budget exceeded')
        raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
        if isinstance(raw,np.ndarray) and raw.dtype.names:return np.column_stack([raw[n].ravel() for n in ('x','y')])
        return np.asarray(list(raw),dtype=float).reshape(-1,3)[:,:2]

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.memory.time is not None and now<self.memory.time:self.reset()
        keys=[k for k,b in self.bundles.items() if len(b)==3 and k>self.last_scan]
        if keys and time.monotonic()-self.ground_rx<1 and self.ground.get('ready') is True:
            k=max(keys);b=self.bundles[k]
            try:
                if not -.05<=now-k/1e9<=.5 or any(m.header.frame_id!=self.frame for m in b.values()):raise ValueError('Stale/mismatched scan')
                p=b['odom'].pose.pose.position;self.origin=(p.x,p.y)
                self.memory.observe(self.origin,self.points(b['nav']),self.points(b['clear']),now)
                self.last_scan=k;self.scan_rx=time.monotonic();self.reason='observed-space memory updated; terrain not qualified'
                self.bundles={t:v for t,v in self.bundles.items() if t>k}
            except (ValueError,TypeError,IndexError) as exc:self.scan_rx=-math.inf;self.reason=str(exc)
        self.memory.expire(now)
        ready=self.ready();route=None
        if self.goal and time.monotonic()-self.goal[2]>300:self.goal=None
        if ready and self.goal:
            try:route=self.memory.route(self.origin,self.goal[:2],now)
            except ValueError as exc:self.reason=str(exc)
        path=Path();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        if route:
            for x,y in route['points']:
                p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;path.poses.append(p)
        self.path_pub.publish(path)  # Empty path explicitly invalidates an older candidate.
        if ready:self.publish_grid()
        cells=sorted(self.memory.cells.items());stride=max(1,math.ceil(len(cells)/3000))
        view=[[*self.memory.point(c),v[0]] for c,v in cells[::stride]]
        self.status.publish(String(data=json.dumps({'ready':ready,'frame':self.frame,'epoch':self.epoch,
            'source_stamp':self.last_scan/1e9,'resolution_m':self.memory.resolution,'cells':view,
            'cell_count':len(cells),'retention_sec':self.memory.retention,'goal':self.goal[:2] if self.goal else None,
            'route':route,'reason':self.reason,'candidate_only':True,'base_link_ready':False,
            'scope':'planar visibility memory; no pit/terrain guarantee; no motor output'})))

    def publish_grid(self):
        r=self.memory.resolution;cx,cy=self.memory.cell(self.origin);half=int(40/r);size=half*2
        xmin,ymin=cx-half,cy-half
        data=np.full((size,size),-1,dtype=np.int8)
        for (x,y),(value,_) in self.memory.cells.items():
            if xmin<=x<xmin+size and ymin<=y<ymin+size:data[y-ymin,x-xmin]=value
        msg=OccupancyGrid();msg.header.frame_id=self.frame
        msg.header.stamp.sec=self.last_scan//10**9;msg.header.stamp.nanosec=self.last_scan%10**9
        msg.info.resolution=r;msg.info.width=size;msg.info.height=size
        msg.info.origin.position.x=xmin*r;msg.info.origin.position.y=ymin*r;msg.info.origin.orientation.w=1.
        msg.data=array('b',data.tobytes());self.grid_pub.publish(msg)


def main():
    rclpy.init();node=LioMemoryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
