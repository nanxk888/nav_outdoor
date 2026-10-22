"""Real LC29H ROS adapter. No serial access, TF or motor control."""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from d1h_global_localization.gnss_quality import GnssGate, antenna_enu


class RealGnss(Node):
    def __init__(self):
        super().__init__('real_gnss_gate')
        defaults=dict(input_topic='/hardware/gnss/fix',max_age_sec=1.5,future_tolerance_sec=.1,
                      minimum_std_m=1.,maximum_horizontal_std_m=10.,allow_unknown_covariance=False,
                      fallback_std_m=5.,maximum_robot_speed_mps=2.,jump_margin_m=10.,
                      enable_enu_preview=False,datum_confirmed=False,
                      datum_latitude_deg=0.,datum_longitude_deg=0.)
        for name,value in defaults.items():self.declare_parameter(name,value)
        p=lambda name:self.get_parameter(name).value
        self.gate=GnssGate(p('max_age_sec'),p('future_tolerance_sec'),p('minimum_std_m'),
            p('maximum_horizontal_std_m'),p('allow_unknown_covariance'),p('fallback_std_m'),
            p('maximum_robot_speed_mps'),p('jump_margin_m'))
        self.preview=p('enable_enu_preview');self.datum=(p('datum_latitude_deg'),p('datum_longitude_deg'))
        if self.preview and (not p('datum_confirmed') or not all(math.isfinite(v) for v in self.datum)
                             or not (-90<self.datum[0]<90 and -180<=self.datum[1]<=180)):
            raise ValueError('ENU preview requires confirmed WGS84 datum matching real navigation profile')
        self.pub=self.create_publisher(NavSatFix,'/localization/gnss/fix_filtered',10)
        self.status=self.create_publisher(String,'/localization/gnss/status',10)
        self.points=self.create_publisher(PointStamped,'/localization/gnss_antenna_enu',10)
        self.sub=self.create_subscription(NavSatFix,p('input_topic'),self.on_fix,qos_profile_sensor_data)
        self.rx=None;self.last_stamp=None;self.accepted=False;self.reason='waiting_for_fix';self.count=0
        self.timer=self.create_timer(.5,self.publish_status)
        self.get_logger().info('LC29H gate: host receive timestamps; GPS position only, NOT fused robot localization')

    def on_fix(self,msg):
        self.rx=time.monotonic();self.last_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        ok,reason,cov,fallback=self.gate.check(now=self.get_clock().now().nanoseconds/1e9,
            stamp=self.last_stamp,frame=msg.header.frame_id,status=msg.status.status,
            lat=msg.latitude,lon=msg.longitude,covariance=msg.position_covariance,
            covariance_type=msg.position_covariance_type)
        self.accepted=ok;self.reason=reason
        if ok:
            clean=copy.deepcopy(msg);clean.position_covariance=cov
            # Flooring is also a modeling assumption, not exact receiver variance.
            clean.position_covariance_type=NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.pub.publish(clean);self.count+=1
            if self.preview:
                point=PointStamped();point.header=copy.deepcopy(msg.header);point.header.frame_id='map'
                point.point.x,point.point.y=antenna_enu(msg.latitude,msg.longitude,*self.datum)
                # Horizontal antenna position only, z=0 is not measured altitude.
                self.points.publish(point)
        self.publish_status()

    def publish_status(self):
        age=(self.get_clock().now().nanoseconds/1e9-self.last_stamp if self.last_stamp is not None else None)
        wall_age=time.monotonic()-self.rx if self.rx is not None else None
        fresh=(age is not None and -self.gate.future_tolerance<=age<=self.gate.max_age
               and wall_age<=self.gate.max_age)
        state='GNSS_READY' if fresh and self.accepted else ('GNSS_REJECTED' if fresh else 'GNSS_STALE')
        self.status.publish(String(data=json.dumps(dict(state=state,accepted=bool(fresh and self.accepted),
            reason=self.reason if fresh else 'no_fresh_fix',age_sec=age,accepted_count=self.count,
            robot_localization_ready=False,rtk_quality='not_inferred_from_NavSatStatus'))))


def main():
    rclpy.init();node=RealGnss()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
