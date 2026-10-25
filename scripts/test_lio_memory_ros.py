"""Native Humble fixture; isolated test domain only, no robot/control publishers."""
import json
import math
import os
import sys
import time
import threading
import types
from array import array
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99,'Use isolated ROS_DOMAIN_ID=94'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String,Header
from nav_msgs.msg import Odometry
from d1h_nav2.lio_memory_node import LioMemoryNode

rclpy.init();memory=LioMemoryNode();fixture=rclpy.create_node('memory_test_fixture')
pubs={name:fixture.create_publisher(typ,topic,10) for name,typ,topic in [
    ('odom',Odometry,'/navigation/lio/odometry'),('nav',PointCloud2,'/livox/lidar/pointcloud_nav'),
    ('clear',PointCloud2,'/livox/lidar/pointcloud_clear'),('ground',String,'/navigation/cloud_filter/status'),
    ('goal',String,'/navigation/memory/goal_candidate'),('reset',String,'/navigation/memory/reset')]}
records=[];sub=fixture.create_subscription(String,'/navigation/memory/status',lambda m:records.append(json.loads(m.data)),10)
executor=SingleThreadedExecutor();executor.add_node(memory);executor.add_node(fixture)
def step(seconds,inputs=True):
    end=time.monotonic()+seconds;next_pub=0.
    while time.monotonic()<end:
        if inputs and time.monotonic()>=next_pub:
            next_pub=time.monotonic()+.1
            h=Header();h.frame_id='lio_odom';h.stamp=fixture.get_clock().now().to_msg()
            o=Odometry();o.header=h;o.child_frame_id='lio_imu';o.pose.pose.orientation.w=1.
            pubs['odom'].publish(o);pubs['nav'].publish(point_cloud2.create_cloud_xyz32(h,[[2.,0.,.3]]))
            pts=[[6*math.cos(a*math.pi/180),6*math.sin(a*math.pi/180),0.] for a in range(360)]
            pubs['clear'].publish(point_cloud2.create_cloud_xyz32(h,pts));pubs['ground'].publish(String(data='{"ready":true}'))
        executor.spin_once(timeout_sec=.01)
try:
    step(3.)
    assert records and records[-1]['ready'],records[-1] if records else 'no status'
    info=records[-1];assert info['cell_count']>50 and len(info['cells'])<=3000
    pubs['goal'].publish(String(data=json.dumps(dict(x=1.,y=1.,frame=info['frame'],epoch=info['epoch'],request_time=time.time()))))
    step(2.)
    assert records[-1]['route'] and records[-1]['route']['candidate_only'],records[-1]['reason']
    assert not records[-1]['base_link_ready']
    step(1.5,False);assert not records[-1]['ready'] and records[-1]['route'] is None
    old=records[-1]['epoch'];step(1.5);assert records[-1]['epoch']!=old and records[-1]['goal'] is None
    assert records[-1]['retention_sec']==300.
    old=memory.epoch;frame=memory.frame
    pubs['reset'].publish(String(data=json.dumps(dict(epoch=old,request_time=time.time()))))
    step(.7,False)
    assert memory.epoch!=old and not memory.memory.cells and memory.goal is None
    assert memory.frame==frame and records[-1]['ready'] is False
    for topic in ('/cmd_vel','/real_navigation/cmd_vel_candidate'):
        assert not any(p.node_name=='lio_observed_memory' for p in fixture.get_publishers_info_by_topic(topic))
    from nav_msgs.msg import OccupancyGrid
    from d1h_nav2.osm_road_planner import OsmRoadPlanner
    grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=fixture.get_clock().now().to_msg()
    grid.info.resolution=.25;grid.info.width=80;grid.info.height=80
    grid.info.origin.position.x=-10.;grid.info.origin.position.y=-10.;grid.info.origin.orientation.w=1.
    grid.data=array('b',[0]*6400)
    fake=types.SimpleNamespace(lock=threading.Lock(),access_grid=grid,get_clock=fixture.get_clock,clear=OsmRoadPlanner.clear)
    assert OsmRoadPlanner.observed_connector(fake,(0.,0.),(3.,0.))
    grid.data[40*80+44]=100;assert not OsmRoadPlanner.observed_connector(fake,(0.,0.),(3.,0.))
    grid.data[40*80+44]=-1;assert not OsmRoadPlanner.observed_connector(fake,(0.,0.),(3.,0.))
    grid.data=array('b',[100]*6400);saved=[]
    fake.grid=grid;fake.pub=types.SimpleNamespace(publish=saved.append);fake.observed_connector=lambda a,b:True
    OsmRoadPlanner.publish_access(fake,(0.,0.),(3.,0.))
    assert 0<list(saved[-1].data).count(0)<100 and list(saved[-1].data).count(100)>6300
    fake.observed_connector=lambda a,b:False;OsmRoadPlanner.expire_access(fake)
    assert fake.access_segment is None and list(saved[-1].data).count(100)==6400
    print('PASS: real ROS matched clouds -> bounded memory -> candidate path; stale input hides route; reset clears goal; no velocity output')
    print('PASS: off-road access rejects obstacles/unknown and carves only a narrow verified corridor')
    print('PASS: 300-second default; explicit memory reset clears cells/goal without resetting odometry')
finally:
    executor.shutdown();memory.destroy_node();fixture.destroy_node();rclpy.shutdown()
