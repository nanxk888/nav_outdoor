"""ROS PointCloud2/odometry -> actual Patchwork++ -> unchanged output topics."""
import os,sys,time,json
from pathlib import Path
assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'.local/patchworkpp'),str(root/'src/d1h_nav2')]
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from d1h_nav2.real_cloud_filter import RealCloudFilter
from test_patchwork_ground import PatchworkTests

rclpy.init(args=['--ros-args','-p','sensor_height_m:=0.46'])
node=RealCloudFilter();fixture=rclpy.create_node('patchwork_interface_test');received={}
subs=[fixture.create_subscription(PointCloud2,'/livox/lidar/pointcloud_'+name,
    lambda m,n=name:received.update({n:m}),5) for name in ('nav','clear')]
subs.append(fixture.create_subscription(String,'/navigation/cloud_filter/status',lambda m:received.update(status=json.loads(m.data)),5))
try:
    floor,wall=PatchworkTests().scene()
    for height_change in (0.,.2,.35):
        for _ in range(5):
            odom=Odometry();odom.header.frame_id='lio_odom';odom.header.stamp=node.get_clock().now().to_msg()
            odom.child_frame_id='lio_imu';odom.pose.pose.orientation.w=1.;odom.pose.pose.position.z=height_change
            node.on_odom(odom)
            node.on_cloud(point_cloud2.create_cloud_xyz32(odom.header,np.r_[floor,wall]));node.health()
            deadline=time.monotonic()+.08
            while time.monotonic()<deadline:rclpy.spin_once(fixture,timeout_sec=.005)
        assert node.last_success and node.ground_plane is not None,node.reason
        assert received['nav'].header.frame_id=='lio_odom' and received['nav'].width>100,received
        assert received['clear'].width>1000
        assert received['status']['backend']=='official_patchworkpp'
        assert abs(received['status']['estimated_imu_height_m']-(.46+height_change))<.02
    print('PASS: actual native Patchwork++ via ROS cloud filter; nav/clear frames and dynamic height tracking')
finally:
    node.destroy_node();fixture.destroy_node();rclpy.shutdown()
