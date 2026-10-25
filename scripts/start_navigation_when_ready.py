"""Wait for real registered inputs, then replace self with owned Nav2 launch."""
import argparse
import json
import os
import rclpy
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--profile',required=True);a=parser.parse_args()
    rclpy.init();node=rclpy.create_node('navigation_input_waiter');ready=False
    def status(msg):
        nonlocal ready
        try:
            s=json.loads(msg.data);age=node.get_clock().now().nanoseconds/1e9-s['stamp']
            ready=s.get('navigation_ready') is True and -.05<=age<.5
        except (ValueError,KeyError,TypeError):ready=False
    sub=node.create_subscription(String,'/real_navigation/safety_status',status,5)
    print('[WAIT] Nav2 waiting for fresh LIO/cloud/memory and stationary GPS + heading registration in web.',flush=True)
    try:
        while rclpy.ok() and not ready:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:return
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if ready:
        print('[START] Registered sensor inputs ready; launching Nav2. No robot mode change.',flush=True)
        os.execvp('ros2',['ros2','launch','d1h_nav2','real_osm_navigation.launch.py','profile:='+a.profile])


if __name__=='__main__':main()
