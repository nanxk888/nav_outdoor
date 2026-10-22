"""Run in an isolated ROS_DOMAIN_ID; synthetic GPS messages, no hardware."""
import json
import pathlib
import sys
import threading
import time
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'/'d1h_global_localization'))
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from d1h_global_localization.real_gnss import RealGnss


def main():
    rclpy.init();gate=RealGnss();test=rclpy.create_node('gnss_test')
    publisher=test.create_publisher(NavSatFix,'/hardware/gnss/fix',10)
    received=[];states=[]
    sub=test.create_subscription(NavSatFix,'/localization/gnss/fix_filtered',received.append,10)
    status=test.create_subscription(String,'/localization/gnss/status',lambda m:states.append(json.loads(m.data)),10)
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(gate);executor.add_node(test)
    thread=threading.Thread(target=executor.spin);thread.start()
    def wait(predicate,timeout=5.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.02)
        raise AssertionError('Timed out')
    def send(status_code):
        fix=NavSatFix();fix.header.frame_id='gps_antenna';fix.header.stamp=test.get_clock().now().to_msg()
        fix.status.status=status_code;fix.latitude=34.125;fix.longitude=108.829;fix.altitude=400.
        fix.position_covariance=[4.,0.,0.,0.,4.,0.,0.,0.,9.];fix.position_covariance_type=2
        publisher.publish(fix)
    try:
        wait(lambda:publisher.get_subscription_count()>0 and gate.pub.get_subscription_count()>0)
        send(0);wait(lambda:bool(received) and any(s['state']=='GNSS_READY' for s in states))
        assert received[0].header.frame_id=='gps_antenna'
        assert received[0].altitude==400.
        send(-1);wait(lambda:any(s['reason']=='no_fix' for s in states))
        assert len(received)==1
        wait(lambda:states[-1]['state']=='GNSS_STALE')
        assert states[-1]['robot_localization_ready'] is False
        print('PASS: valid fix forwarded; invalid fix rejected; stale status; no fused-localization readiness')
    finally:
        executor.shutdown();thread.join();test.destroy_node();gate.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
