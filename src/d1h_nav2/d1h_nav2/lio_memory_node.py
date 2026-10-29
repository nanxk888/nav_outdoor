"""Native LIO observation memory + candidate paths. NEVER publishes velocity/TF."""
import json
import math
import time
import uuid
import copy
from concurrent.futures import ThreadPoolExecutor
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
from d1h_nav2.footprint import path_free
from d1h_nav2.navigation_reference import body_heading


def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


class LioMemoryNode(Node):
    def __init__(self):
        super().__init__('lio_observed_memory')
        for k,v in {'resolution_m':.1,'retention_sec':0.,'footprint_radius_m':.25,'max_cells':0}.items():self.declare_parameter(k,v)
        self.memory=ObservedMemory(self.get_parameter('resolution_m').value,self.get_parameter('retention_sec').value,
                                   max_cells=self.get_parameter('max_cells').value,
                                   radius=self.get_parameter('footprint_radius_m').value,min_points=3,confirm_frames=2)
        self.bundles={};self.previous=None;self.frame='';self.last_scan=0;self.origin=(0.,0.);self.goal=None
        self.saved_route=None
        self.worker=ThreadPoolExecutor(max_workers=1);self.planning=None;self.next_plan=0.;self.view=[]
        self.diagnostics={};self.tick_started=None;self.plan_started=None
        self.epoch=uuid.uuid4().hex;self.reason='waiting for exact-time LIO + nav + clear'
        self.ground={};self.ground_rx=-math.inf;self.scan_rx=-math.inf
        self.status=self.create_publisher(String,'/navigation/memory/status',1)
        self.display_pub=self.create_publisher(String,'/navigation/memory/display',1)
        self.grid_pub=self.create_publisher(OccupancyGrid,'/navigation/memory/grid',1)
        self.path_pub=self.create_publisher(Path,'/navigation/memory/path_candidate',1)
        self.subs=[self.create_subscription(Odometry,'/navigation/lio/odometry',self.on_odom,10),
            self.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',lambda m:self.cache('nav',m),qos_profile_sensor_data),
            self.create_subscription(PointCloud2,'/livox/lidar/pointcloud_clear',lambda m:self.cache('clear',m),qos_profile_sensor_data),
            self.create_subscription(String,'/navigation/cloud_filter/status',self.on_ground,5),
            self.create_subscription(String,'/navigation/memory/goal_candidate',self.on_goal,1),
            self.create_subscription(String,'/navigation/memory/reset',self.on_reset,1)]
        self.timer=self.create_timer(.2,self.tick)
        self.display_timer=self.create_timer(1.,self.prepare_display)

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

    def prepare_display(self):
        started=time.monotonic()
        nearby=sorted(self.memory.cells.items(),key=lambda item:math.dist(self.memory.point(item[0]),self.origin))[:20000]
        self.view=[[*self.memory.point(c),v[0]] for c,v in nearby]
        self.display_pub.publish(String(data=json.dumps(dict(epoch=self.epoch,cells=self.view))))
        self.diagnostics['display_ms']=(time.monotonic()-started)*1000

    @staticmethod
    def candidate(memory,origin,goal,previous,now):
        if previous and previous.get('partial') is False:
            points=previous['points'];index=min(range(len(points)),key=lambda i:math.dist(origin,points[i]))
            remaining=[origin]+points[index:]
            if math.dist(origin,points[index])<=.4 and path_free(remaining,memory.resolution,memory.radius,
                lambda c:memory.cells.get(c,(-1,0))[0]==0):return dict(previous,points=remaining)
        return memory.route(origin,goal,now)

    def cache(self,name,msg):
        self.bundles.setdefault(stamp(msg),{})[name]=msg
        while len(self.bundles)>15:self.bundles.pop(next(iter(self.bundles)))

    def reset(self):
        self.saved_route=None
        self.view=[];self.next_plan=0.
        self.memory.cells.clear();self.memory.time=None;self.goal=None;self.bundles.clear();self.last_scan=0
        self.memory.hits.clear();self.memory.free_hits.clear()
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
            self.saved_route=None
            self.next_plan=0.
            self.goal=(goal['x'],goal['y'],time.monotonic());self.reason='memory goal accepted for candidate planning only'
        except (ValueError,TypeError,KeyError) as exc:self.reason='Goal rejected: '+str(exc)

    @staticmethod
    def points(msg):
        if msg.width*msg.height>100000:raise ValueError('Point budget exceeded')
        raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
        if isinstance(raw,np.ndarray) and raw.dtype.names:return np.column_stack([raw[n].ravel() for n in ('x','y')])
        return np.asarray(list(raw),dtype=float).reshape(-1,3)[:,:2]

    def tick(self):
        started=time.monotonic()
        self.diagnostics['tick_interval_ms']=(started-self.tick_started)*1000 if self.tick_started else None
        self.tick_started=started
        now=self.get_clock().now().nanoseconds/1e9
        if self.memory.time is not None and now<self.memory.time:self.reset()
        keys=[k for k,b in self.bundles.items() if len(b)==3 and k>self.last_scan]
        if keys and time.monotonic()-self.ground_rx<1 and self.ground.get('ready') is True:
            k=max(keys);b=self.bundles[k]
            try:
                if not -.05<=now-k/1e9<=.5 or any(m.header.frame_id!=self.frame for m in b.values()):raise ValueError('Stale/mismatched scan')
                p=b['odom'].pose.pose.position;self.origin=(p.x,p.y)
                observed=time.monotonic()
                q=b['odom'].pose.pose.orientation
                yaw=body_heading((q.x,q.y,q.z,q.w))
                self.memory.observe(self.origin,self.points(b['nav']),self.points(b['clear']),now,heading=yaw)
                self.diagnostics['observe_ms']=(time.monotonic()-observed)*1000
                self.last_scan=k;self.scan_rx=time.monotonic();self.reason='observed-space memory updated; terrain not qualified'
                self.bundles={t:v for t,v in self.bundles.items() if t>k}
            except (ValueError,TypeError,IndexError) as exc:self.scan_rx=-math.inf;self.reason=str(exc)
        self.memory.expire(now)
        ready=self.ready();route=None
        if self.goal and time.monotonic()-self.goal[2]>300:self.goal=None
        if self.planning and self.planning[0].done():
            self.diagnostics['candidate_wall_ms']=(time.monotonic()-self.plan_started)*1000
            future,epoch,goal=self.planning;self.planning=None
            if epoch==self.epoch and goal==self.goal:
                try:self.saved_route=future.result()
                except ValueError as exc:self.saved_route=None;self.reason=str(exc)
        if ready and self.goal:
            route=self.saved_route
            if self.planning is None and time.monotonic()>=self.next_plan:
                snapshot=copy.copy(self.memory);snapshot.cells=dict(self.memory.cells)
                future=self.worker.submit(self.candidate,snapshot,self.origin,self.goal[:2],self.saved_route,now)
                self.planning=(future,self.epoch,self.goal);self.next_plan=time.monotonic()+1.
                self.plan_started=time.monotonic()
        path=Path();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        if route:
            for x,y in route['points']:
                p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;path.poses.append(p)
        self.path_pub.publish(path)  # Empty path explicitly invalidates an older candidate.
        grid_started=time.monotonic()
        if ready:self.publish_grid()
        self.diagnostics.update(grid_publish_ms=(time.monotonic()-grid_started)*1000,
            scan_age_sec=self.get_clock().now().nanoseconds/1e9-self.last_scan/1e9,
            matched_scan_receive_age_sec=time.monotonic()-self.scan_rx if math.isfinite(self.scan_rx) else None,
            ground_receive_age_sec=time.monotonic()-self.ground_rx if math.isfinite(self.ground_rx) else None,
            ground_reason=self.ground.get('reason'),ground_ready=self.ground.get('ready'),
            incomplete_bundles=sum(len(b)<3 for b in self.bundles.values()),
            pending_components={name:sum(name in b for b in self.bundles.values()) for name in ('odom','nav','clear')},
            planning_pending=self.planning is not None)
        readiness_reason='正常' if ready else ('扫描配对未更新' if time.monotonic()-self.scan_rx>=1 else '地面状态未就绪或过期')
        self.status.publish(String(data=json.dumps({'ready':ready,'frame':self.frame,'epoch':self.epoch,
            'source_stamp':self.last_scan/1e9,'resolution_m':self.memory.resolution,
            'cell_count':len(self.memory.cells),'retention_sec':self.memory.retention,'goal':self.goal[:2] if self.goal else None,
            'footprint_radius_m':self.memory.radius,
            'route':route,'reason':self.reason,'readiness_reason':readiness_reason,'diagnostics':self.diagnostics,'candidate_only':True,'base_link_ready':False,
            'scope':'planar visibility memory; no pit/terrain guarantee; no motor output'})))
        self.diagnostics['tick_ms']=(time.monotonic()-started)*1000

    def publish_grid(self):
        # Keep the existing 20m GPS off-road connector within the evidence grid.
        r=self.memory.resolution;cx,cy=self.memory.cell(self.origin);half=int(24/r);size=half*2
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
