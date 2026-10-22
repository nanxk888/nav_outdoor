"""Stationary analytic CustomMsg/IMU smoke test. Not hardware accuracy evidence."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import String


def main():
    root=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='navigation-lio-test-') as tmp:
        profile=yaml.safe_load((root/'src/d1h_fast_lio/config/navigation_lio.yaml').read_text())
        params=profile['/**']['ros__parameters']
        params['extrin_calib']['extrinsic_T']=[0.,0.,0.]
        params['common']['lid_topic']='/fixture/lidar'
        params['common']['imu_topic']='/fixture/imu'
        config=Path(tmp)/'params.yaml';config.write_text(yaml.safe_dump(profile))
        log=Path(tmp)/'node.log'
        rclpy.init();node=rclpy.create_node('navigation_lio_fixture')
        pc=node.create_publisher(CustomMsg,'/fixture/lidar',4)
        pi=node.create_publisher(Imu,'/fixture/imu',400)
        odoms=[];states=[];scan_ends=[]
        so=node.create_subscription(Odometry,'/navigation/lio/odometry',odoms.append,10)
        ss=node.create_subscription(String,'/navigation/lio/status',lambda s:states.append(json.loads(s.data)),10)
        geometry=[]
        # Five static surfaces surrounding sensor. No simulator truth feeds the estimator.
        for i in range(19):
            for j in range(17):
                a=-2.25+i*.25;b=-1.+j*.25
                geometry.extend([(3.,a,b),(-3.,a,b),(a,3.,b),(a,-3.,b),(a,b,-1.)])
        def stamp(value):
            from builtin_interfaces.msg import Time
            ns=round(value*1e9);return Time(sec=ns//1000000000,nanosec=ns%1000000000)
        with log.open('w') as output:
            process=subprocess.Popen(['ros2','run','d1h_fast_lio','lio_odometry','--ros-args',
                '--params-file',str(config),'--log-level','warn'],stdout=output,stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                start=time.monotonic();next_cloud=start;test_started=None
                # Startup/DDS discovery is not part of the six-second workload.
                # Keep feeding valid inputs while waiting; do not weaken output checks.
                while test_started is None or time.monotonic()-test_started<6.:
                    if process.poll() is not None:raise AssertionError(log.read_text())
                    if test_started is None:
                        if pc.get_subscription_count()>0 and pi.get_subscription_count()>0:
                            test_started=time.monotonic()
                        elif time.monotonic()-start>15.:
                            raise AssertionError('DDS/estimator startup timeout: '+log.read_text())
                    now=node.get_clock().now().nanoseconds/1e9
                    imu=Imu();imu.header.frame_id='livox_frame';imu.header.stamp=stamp(now)
                    imu.linear_acceleration.z=1.;pi.publish(imu)
                    if time.monotonic()>=next_cloud:
                        cloud=CustomMsg();cloud.header.frame_id='livox_frame';cloud.header.stamp=stamp(now-.1)
                        cloud.timebase=round((now-.1)*1e9)
                        for i,(x,y,z) in enumerate(geometry):
                            point=CustomPoint();point.x=float(x);point.y=float(y);point.z=float(z)
                            point.line=i%4;point.tag=0x10;point.offset_time=round(i/(len(geometry)-1)*95000000)
                            cloud.points.append(point)
                        cloud.point_num=len(cloud.points);pc.publish(cloud)
                        scan_ends.append(now-.005);next_cloud+=.1
                    rclpy.spin_once(node,timeout_sec=.001)
                    time.sleep(.003)
                assert len(odoms)>=10,(len(odoms),log.read_text())
                assert states and states[-1]['base_link_ready'] is False
                times=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in odoms]
                assert all(b>a for a,b in zip(times,times[1:]))
                assert all(min(abs(t-end) for end in scan_ends)<2e-6 for t in times), 'State must use measured scan-end time'
                assert all(m.header.frame_id=='lio_odom' and m.child_frame_id=='lio_imu' for m in odoms)
                assert all(m.pose.covariance[0]>0 and m.twist.covariance[35]>0 for m in odoms)
                deviation=max(math.sqrt(m.pose.pose.position.x**2+m.pose.pose.position.y**2+m.pose.pose.position.z**2) for m in odoms)
                assert deviation<.2,('stationary analytic deviation',deviation)
                # Stop inputs: the process must fail-stop without replaying old odometry.
                deadline=time.monotonic()+3.
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                assert process.poll() is not None and process.returncode!=0,log.read_text()
                print('PASS: analytic static outputs=',len(odoms),'max_position_norm_m=',deviation,
                      '; measurement time, frames, covariance, no-camera input and input-loss exit verified')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
