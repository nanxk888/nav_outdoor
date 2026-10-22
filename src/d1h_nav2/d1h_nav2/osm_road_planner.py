"""Real-only OSM ComputePathToPose backend; never publishes motor commands."""
import json
import math
import threading
from array import array
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from tf2_ros import Buffer, TransformListener
from d1h_nav2.osm_roads import RoadGraph, sample


class OsmRoadPlanner(Node):
    def __init__(self):
        super().__init__('osm_road_planner')
        self.declare_parameter('profile', '')
        with open(self.get_parameter('profile').value, encoding='utf-8') as f:
            profile=json.load(f)
        if not profile.get('survey_confirmed'):
            raise ValueError('Road survey / datum confirmation required before enabling real planning')
        self.roads=RoadGraph(profile['osm_file'],profile['datum_latitude_deg'],
                             profile['datum_longitude_deg'],profile['minimum_road_width_m'],
                             profile['approved_way_ids'])
        self.lock=threading.Lock(); self.costmap=None; self.rx=-math.inf
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        group=ReentrantCallbackGroup()
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/map',qos)
        self.sub=self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_map,qos)
        self.server=ActionServer(self,ComputePathToPose,'/roads/compute_path_to_pose',
                                 self.compute,cancel_callback=lambda _:CancelResponse.ACCEPT,
                                 callback_group=group)
        self.validator=self.create_service(IsPathValid,'/roads/is_path_valid',self.validate,
                                           callback_group=group)
        self.grid=self.make_grid()
        self.pub.publish(self.grid)
        self.get_logger().info(f'Real OSM A*: {len(self.roads.edges)} approved segments; no actuator output')

    def make_grid(self):
        resolution=.5
        points=[self.roads.xy[i] for a,b,_ in self.roads.edges for i in (a,b)]
        xmin=min(p[0] for p in points)-5; ymin=min(p[1] for p in points)-5
        w=math.ceil((max(p[0] for p in points)+5-xmin)/resolution)
        h=math.ceil((max(p[1] for p in points)+5-ymin)/resolution)
        if w*h>20000000:raise ValueError('Approved road extent too large; split deployment region')
        data=np.full((h,w),100,dtype=np.int8)
        # Width must be surveyed minimum clear width, not an inferred OSM width.
        radius=self.roads.width/2
        for a,b,_ in self.roads.edges:
            ax,ay=self.roads.xy[a];bx,by=self.roads.xy[b]
            x0=max(0,int((min(ax,bx)-radius-xmin)/resolution));x1=min(w,math.ceil((max(ax,bx)+radius-xmin)/resolution))
            y0=max(0,int((min(ay,by)-radius-ymin)/resolution));y1=min(h,math.ceil((max(ay,by)+radius-ymin)/resolution))
            yy,xx=np.mgrid[y0:y1,x0:x1]
            xx=xmin+(xx+.5)*resolution; yy=ymin+(yy+.5)*resolution
            t=np.clip(((xx-ax)*(bx-ax)+(yy-ay)*(by-ay))/((bx-ax)**2+(by-ay)**2),0,1)
            # Conservative cell coverage; do not expand free area past surveyed edge.
            mask=(xx-ax-t*(bx-ax))**2+(yy-ay-t*(by-ay))**2 <= max(0,radius-resolution*.71)**2
            data[y0:y1,x0:x1][mask]=0
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.resolution=resolution;msg.info.width=w;msg.info.height=h
        msg.info.origin.position.x=xmin;msg.info.origin.position.y=ymin;msg.info.origin.orientation.w=1.
        msg.data=array('b',data.tobytes())
        return msg

    def on_map(self,msg):
        if msg.header.frame_id!='map' or abs(msg.info.origin.orientation.w-1)>1e-5:return
        with self.lock:self.costmap=msg;self.rx=self.get_clock().now().nanoseconds/1e9

    def snapshot(self):
        with self.lock:
            if self.get_clock().now().nanoseconds/1e9-self.rx>3.:raise ValueError('Global obstacle costmap missing/stale')
            return self.costmap

    @staticmethod
    def clear(grid,a,b):
        for x,y in sample(a,b,min(.2,grid.info.resolution/2)):
            ix=math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
            iy=math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
            if not (0<=ix<grid.info.width and 0<=iy<grid.info.height):return False
            cost=grid.data[iy*grid.info.width+ix]
            if cost<0 or cost>=99:return False
        return True

    def compute(self,handle):
        result=ComputePathToPose.Result()
        try:
            request=handle.request
            if request.goal.header.frame_id!='map':raise ValueError('Goal must use map frame')
            if request.use_start:
                if request.start.header.frame_id!='map':raise ValueError('Start must use map frame')
                p=request.start.pose.position
            else:p=self.tf.lookup_transform('map','base_link',Time()).transform.translation
            grid=self.snapshot();q=request.goal.pose.position
            points=self.roads.route((p.x,p.y),(q.x,q.y),lambda a,b:self.clear(grid,a,b))
            path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
            for a,b in zip(points,points[1:]):
                yaw=math.atan2(b[1]-a[1],b[0]-a[0])
                for x,y in sample(a,b)[:-1]:
                    pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y
                    pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                    path.poses.append(pose)
            path.poses.append(request.goal)
            if handle.is_cancel_requested:handle.canceled()
            else:result.path=path;handle.succeed()
        except Exception as exc:
            self.get_logger().warning(str(exc));handle.abort()
        return result

    def validate(self,request,response):
        response.is_valid=False
        try:
            if request.path.header.frame_id!='map' or not request.path.poses:return response
            grid=self.snapshot();p=self.tf.lookup_transform('map','base_link',Time()).transform.translation
            pts=[(s.pose.position.x,s.pose.position.y) for s in request.path.poses]
            i=min(range(len(pts)),key=lambda i:math.dist(pts[i],(p.x,p.y)))
            pts=pts[i:]
            response.is_valid=all(self.clear(grid,a,b) for a,b in zip(pts,pts[1:])) and self.clear(grid,pts[0],pts[0])
        except Exception:pass
        return response


def main():
    rclpy.init();node=OsmRoadPlanner();executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:executor.spin()
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
