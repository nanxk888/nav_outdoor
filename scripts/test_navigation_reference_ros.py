"""Isolated native ROS reference/registration/expiry check; no motors."""
import os
import sys
import math
import time
import json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99,'Use isolated domain'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from d1h_nav2.navigation_reference_node import NavigationReference,stamp

rclpy.init();reference=NavigationReference();fixture=rclpy.create_node('reference_test_fixture')
executor=SingleThreadedExecutor();executor.add_node(reference);executor.add_node(fixture)
pubs={k:fixture.create_publisher(t,topic,10) for k,t,topic in [
    ('odom',Odometry,'/navigation/lio/odometry'),('gps',NavSatFix,'/hardware/gnss/fix'),
    ('ground',String,'/navigation/cloud_filter/status'),('memory',String,'/navigation/memory/status'),
    ('register',String,'/navigation/reference/register')]}
records=[];outputs=[]
subs=[fixture.create_subscription(String,'/real_navigation/safety_status',lambda m:records.append(json.loads(m.data)),10),
      fixture.create_subscription(Odometry,'/odometry/local',outputs.append,10)]
buffer=Buffer();listener=TransformListener(buffer,fixture)
latest={}
def step(seconds,inputs=True):
    end=time.monotonic()+seconds;next_pub=0.
    while time.monotonic()<end:
        if inputs and time.monotonic()>=next_pub:
            next_pub=time.monotonic()+.1
            o=Odometry();o.header.frame_id='lio_odom';o.child_frame_id='lio_imu';o.header.stamp=fixture.get_clock().now().to_msg()
            o.pose.pose.position.x=3.;o.pose.pose.position.y=4.
            o.pose.pose.orientation.y=math.sin(-math.pi/12);o.pose.pose.orientation.w=math.cos(math.pi/12)
            o.twist.twist.linear.x=.1
            g=NavSatFix();g.header.stamp=o.header.stamp;g.status.status=0;g.latitude=34.125;g.longitude=108.829
            latest.update(odom=o,gps=g)
            pubs['odom'].publish(o);pubs['gps'].publish(g)
            pubs['ground'].publish(String(data='{"ready":true,"measurement_age_sec":0.02}'))
            pubs['memory'].publish(String(data=json.dumps(dict(ready=True,frame='lio_odom',source_stamp=stamp(o)))))
        executor.spin_once(timeout_sec=.01)
try:
    step(2.)
    assert records and records[-1]['navigation_base_ready'] and not records[-1]['registered']
    assert outputs[-1].child_frame_id=='navigation_base' and outputs[-1].pose.pose.orientation.y==0.
    assert abs(outputs[-1].twist.twist.linear.x-.1*math.cos(math.pi/6))<1e-6
    pubs['register'].publish(String(data=json.dumps(dict(heading_deg=0.,lio_stamp=stamp(latest['odom']),gps_stamp=stamp(latest['gps']),request_time=time.time()))))
    step(1.)
    assert records[-1]['navigation_ready'],records[-1]
    tf=buffer.lookup_transform('map','navigation_base',Time()).transform
    assert abs(tf.translation.x)<1e-5 and abs(tf.translation.y)<1e-5
    assert not records[-1]['base_link_ready'],'must not pretend virtual reference is vendor base_link'
    step(.8,False);assert not records[-1]['navigation_ready'] and not records[-1]['registered']
    step(1.);assert records[-1]['navigation_base_ready'] and not records[-1]['registered'],'no automatic re-registration'
    assert not fixture.get_publishers_info_by_topic('/real_navigation/cmd_vel_candidate')
    print('PASS: tilted +X projection, real odom/TF, matched GPS anchor, live readiness, expiry invalidates registration; no velocity output')
finally:
    executor.shutdown();reference.destroy_node();fixture.destroy_node();rclpy.shutdown()
