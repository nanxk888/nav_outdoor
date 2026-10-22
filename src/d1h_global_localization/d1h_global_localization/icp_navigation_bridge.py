"""Simulation-only Nav2 adapter for ICP poses, NOT a continuous LIO estimator.

map=odom here. Bounded short extrapolation fills TF between 2Hz matches.
Never consumes Gazebo truth. Navigation must stop when ICP becomes invalid.
"""
import copy
import json
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster


class IcpNavigationBridge(Node):
    def __init__(self):
        super().__init__('icp_navigation_bridge')
        self.tf=TransformBroadcaster(self);self.fixed=StaticTransformBroadcaster(self)
        static=TransformStamped();static.header.frame_id='map';static.child_frame_id='odom';static.transform.rotation.w=1.
        self.fixed.sendTransform(static)
        self.publisher=self.create_publisher(Odometry,'/odometry/local',5)
        self.input=self.create_subscription(Odometry,'/localization/odometry',self.on_pose,5)
        self.status_sub=self.create_subscription(String,'/localization/status',self.on_status,5)
        self.latest=None;self.velocity=np.zeros(3);self.omega=np.zeros(3)
        self.good=False;self.status_time=-1e9
        self.timer=self.create_timer(1/30,self.publish)
        self.get_logger().warning('EXPERIMENTAL SIMULATION ICP bridge: map=odom; no independent LIO, extrapolation <=0.6s; stop on invalid status')

    def on_status(self,msg):
        try:values=json.loads(msg.data)
        except (ValueError,TypeError):return
        self.good=values.get('state')=='TRACKING_PROTOTYPE' and values.get('accepted') is True
        self.status_time=self.get_clock().now().nanoseconds/1e9

    def on_pose(self,msg):
        if msg.header.frame_id!='map':return
        if self.latest is not None:
            old=self.latest
            dt=(rclpy.time.Time.from_msg(msg.header.stamp)-rclpy.time.Time.from_msg(old.header.stamp)).nanoseconds/1e9
            if 0<dt<1.2:
                a,b=old.pose.pose,msg.pose.pose
                va=np.array([b.position.x-a.position.x,b.position.y-a.position.y,b.position.z-a.position.z])/dt
                qa,qb=a.orientation,b.orientation
                ra=Rotation.from_quat([qa.x,qa.y,qa.z,qa.w]);rb=Rotation.from_quat([qb.x,qb.y,qb.z,qb.w])
                wa=(rb*ra.inv()).as_rotvec()/dt
                # Ignore gross pose jumps instead of extrapolating them.
                self.velocity=np.clip(va,-.30,.30)
                self.omega=np.clip(wa,-.5,.5)
            else:self.velocity[:]=0;self.omega[:]=0
        self.latest=copy.deepcopy(msg)
        self.send(msg,stamp=msg.header.stamp,age=0.)

    def send(self,msg,stamp,age):
        out=copy.deepcopy(msg);out.header.frame_id='odom';out.header.stamp=stamp;out.child_frame_id='base_link'
        p,q=out.pose.pose.position,out.pose.pose.orientation
        displacement=self.velocity*age
        p.x+=float(displacement[0]);p.y+=float(displacement[1]);p.z+=float(displacement[2])
        r=Rotation.from_rotvec(self.omega*age)*Rotation.from_quat([q.x,q.y,q.z,q.w])
        q.x,q.y,q.z,q.w=map(float,r.as_quat())
        local_v=r.inv().apply(self.velocity);local_w=r.inv().apply(self.omega)
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,local_v)
        out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,local_w)
        tf=TransformStamped();tf.header=out.header;tf.child_frame_id='base_link'
        tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.translation.z=p.z;tf.transform.rotation=q
        self.tf.sendTransform(tf);self.publisher.publish(out)

    def publish(self):
        if self.latest is None:return
        now=self.get_clock().now();seconds=now.nanoseconds/1e9
        age=(now-rclpy.time.Time.from_msg(self.latest.header.stamp)).nanoseconds/1e9
        if self.good and seconds-self.status_time<1. and 0<=age<=.6:
            self.send(self.latest,now.to_msg(),age)


def main():
    rclpy.init();node=IcpNavigationBridge()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()
