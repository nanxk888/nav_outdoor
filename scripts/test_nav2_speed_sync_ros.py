"""Isolated ROS parameter-service integration; synthetic nodes, no hardware."""
import os
import sys
import json
import time
from pathlib import Path
assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99,'Use an isolated ROS_DOMAIN_ID=94'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from d1h_nav2.nav2_speed_sync import Nav2SpeedSync,targets


def main():
    rclpy.init();controller=Node('controller_server');smoother=Node('velocity_smoother');fixture=Node('speed_sync_fixture')
    for name,node in (('controller_server',controller),('velocity_smoother',smoother)):
        for key,value in targets(dict(linear_mps=.15,angular_rps=.35))[name].items():node.declare_parameter(key,value)
    reject=[False]
    controller.add_on_set_parameters_callback(lambda params:SetParametersResult(successful=not reject[0],reason='fixture rejection' if reject[0] else ''))
    sync=Nav2SpeedSync();executor=SingleThreadedExecutor();nodes=[controller,smoother,fixture,sync]
    for node in nodes:executor.add_node(node)
    pub=fixture.create_publisher(String,'/navigation/speed_limits',1);status={}
    sub=fixture.create_subscription(String,'/real_navigation/speed_status',lambda m:status.update(json.loads(m.data)),1)
    def wait(limits,expect_ready=True):
        deadline=time.monotonic()+8;next_pub=0
        while time.monotonic()<deadline:
            if time.monotonic()>=next_pub:pub.publish(String(data=json.dumps(limits)));next_pub=time.monotonic()+.2
            executor.spin_once(timeout_sec=.01)
            if expect_ready and status.get('ready') and status.get('applied')==limits:return
            if not expect_ready and status.get('state')=='ERROR':return
        raise AssertionError(status)
    try:
        high=dict(linear_mps=1,angular_rps=.5);wait(high)
        assert controller.get_parameter('FollowPath.desired_linear_vel').value==1.
        assert list(smoother.get_parameter('max_velocity').value)==[1.,0.,.5]
        low=dict(linear_mps=.1,angular_rps=.1);wait(low)
        assert list(smoother.get_parameter('min_velocity').value)==[0.,0.,-.1]
        reject[0]=True;wait(dict(linear_mps=.8,angular_rps=.4),False)
        assert not status['ready'] and status['applied'] is None and 'fixture rejection' in status['reason'],status
        reject[0]=False;wait(dict(linear_mps=.3,angular_rps=.2))
        print('PASS: Nav2 speed parameter RPC + readback, 1m/s, live reduction, rejection and explicit request recovery')
    finally:
        executor.shutdown()
        for node in nodes:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
