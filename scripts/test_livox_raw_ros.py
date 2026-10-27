"""Isolated ROS loopback test, synthetic 20k-point scans; no hardware or motion."""
import os
import time
os.environ.update(ROS_DOMAIN_ID='94',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('CYCLONEDDS_URI','ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):os.environ.pop(key,None)
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from sensor_msgs.msg import Imu
from ensure_livox_and_lio import lidar_header,SensorGate


def main():
    rclpy.init();node=rclpy.create_node('synthetic_livox_raw_regression');gate=SensorGate()
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    lidar=node.create_publisher(CustomMsg,'/test_guard/lidar',qos)
    imu=node.create_publisher(Imu,'/test_guard/imu',qos)
    m=CustomMsg();m.header.frame_id='livox_frame';m.point_num=20000;m.points=[CustomPoint()]*20000
    def scan():m.header.stamp=node.get_clock().now().to_msg();lidar.publish(m)
    def inertial():
        msg=Imu();msg.header.frame_id='livox_frame';msg.header.stamp=node.get_clock().now().to_msg();imu.publish(msg)
    def raw(data):
        stamp,frame,count=lidar_header(data)
        assert frame=='livox_frame' and count==20000
        gate.observe('lidar',stamp,node.get_clock().now().nanoseconds/1e9,time.monotonic())
    def imu_received(msg):gate.observe('imu',msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,node.get_clock().now().nanoseconds/1e9,time.monotonic())
    subs=[node.create_subscription(CustomMsg,'/test_guard/lidar',raw,qos,raw=True),node.create_subscription(Imu,'/test_guard/imu',imu_received,qos)]
    timers=[node.create_timer(.1,scan),node.create_timer(.005,inertial)]
    try:
        end=time.monotonic()+10
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if gate.ready(node.get_clock().now().nanoseconds/1e9,time.monotonic()):
                print('PASS: real ROS raw 20k-point scans + IMU satisfy startup gate:',gate.detail(time.monotonic()));return
        raise AssertionError('Raw ROS startup failed: '+str(gate.detail(time.monotonic())))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
