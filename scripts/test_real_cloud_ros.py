"""Isolated Humble point-cloud wiring test, no physical sensor publications."""
import os
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from d1h_nav2.real_cloud_filter import RealCloudFilter
import navigation_web as dashboard


def main():
    if not 80<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=99:raise SystemExit('Use isolated domain 80..99')
    rclpy.init();node=RealCloudFilter();probe=rclpy.create_node('real_cloud_fixture')
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(probe)
    outputs=[];clears=[]
    subscriptions=[probe.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',outputs.append,5),
                   probe.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',dashboard.cloud,5),
                   probe.create_subscription(PointCloud2,'/livox/lidar/pointcloud_clear',clears.append,5)]
    odom_pub=probe.create_publisher(Odometry,'/navigation/lio/odometry',5)
    cloud_pub=probe.create_publisher(PointCloud2,'/navigation/lio/deskewed_cloud',5)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    def send(height=.6,origin_z=0.):
        stamp=probe.get_clock().now().to_msg()
        odom=Odometry();odom.header.frame_id='lio_odom';odom.header.stamp=stamp
        odom.child_frame_id='lio_imu';odom.pose.pose.orientation.w=1.
        odom.pose.pose.position.z=origin_z
        x,y=np.meshgrid(np.linspace(-2,2,35),np.linspace(-2,2,35))
        points=np.vstack([np.c_[x.ravel(),y.ravel(),np.full(x.size,-height)],[[1,0,-height+.05]]])
        cloud=point_cloud2.create_cloud_xyz32(odom.header,points)
        # Cloud-first ordering exercises matching by acquisition stamp.
        cloud_pub.publish(cloud);odom_pub.publish(odom);return stamp
    try:
        node.set_parameters([Parameter('sensor_height_m',value=.6)])
        spin(1);stamp=send();spin(.3)
        assert outputs and clears,'missing output topics'
        assert outputs[-1].header.stamp==stamp and outputs[-1].header.frame_id=='lio_odom'
        assert outputs[-1].width==1 and clears[-1].width>1000,'ground/obstacle classification'
        preview=dashboard.samples['nav_cloud']['value']
        assert preview['valid'] and preview['frame']=='lio_odom' and len(preview['points'])==1,'web preview serialization'
        stamp=send(origin_z=.3);spin(.3)
        assert outputs[-1].header.stamp==stamp and outputs[-1].width==1,'body rising must not shift ground'
        before=(len(outputs),len(clears));send(height=2.5);spin(.3)
        assert (len(outputs),len(clears))==before,'invalid plane must not publish clearing'
        print('PASS: real Humble PointCloud2 parsing, matched timestamps, ground removal, 5cm obstacle and fail-closed clearing')
    finally:
        executor.shutdown();node.destroy_node();probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
