"""Domain-0 GNSS topic adapter. Preserve measurements, including NO_FIX/NaN."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class GnssTopicAdapter(Node):
    def __init__(self):
        super().__init__('nxk_gnss_topic_adapter')
        self.declare_parameter('input_topic','/meridian/observation/gnss/fix')
        self.declare_parameter('output_topic','/hardware/gnss/fix')
        source=self.get_parameter('input_topic').value
        target=self.get_parameter('output_topic').value
        if not source.startswith('/') or not target.startswith('/') or source==target:
            raise ValueError('GNSS adapter requires distinct absolute input/output topics')
        self.target=target;self.received=False
        self.pub=self.create_publisher(NavSatFix,target,5)
        self.sub=self.create_subscription(NavSatFix,source,self.forward,qos_profile_sensor_data)
        self.get_logger().info(f'GNSS adapter waiting: {source} -> {target}; Domain 0; no driver started')

    def forward(self,msg):
        # Do not create competing fixes on the internal canonical topic.
        others=[p for p in self.get_publishers_info_by_topic(self.target)
                if p.node_name!=self.get_name() or p.node_namespace!=self.get_namespace()]
        if others:
            raise RuntimeError('Conflicting publisher on '+self.target+'; stop legacy GNSS bridge/duplicate adapter')
        self.pub.publish(msg)
        if not self.received:
            self.received=True
            self.get_logger().info('GNSS first message forwarded; fix status='+str(msg.status.status)+
                                   ' (-1 means received but no satellite fix)')


def main():
    rclpy.init();node=GnssTopicAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
