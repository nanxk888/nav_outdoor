"""Hardware-free adapter callback tests; no ROS publisher is created."""
import sys,types,unittest,math,importlib.util
from pathlib import Path
from unittest.mock import Mock,patch

root=Path(__file__).resolve().parents[1]
fake_modules={
    'rclpy':types.ModuleType('rclpy'),
    'rclpy.node':types.SimpleNamespace(Node=object),
    'rclpy.qos':types.SimpleNamespace(qos_profile_sensor_data=object()),
    'sensor_msgs':types.ModuleType('sensor_msgs'),
    'sensor_msgs.msg':types.SimpleNamespace(NavSatFix=object),
}
with patch.dict(sys.modules,fake_modules):
    spec=importlib.util.spec_from_file_location('adapter',root/'src/d1h_nav2/d1h_nav2/gnss_topic_adapter.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class AdapterTests(unittest.TestCase):
    def adapter(self):
        a=object.__new__(module.GnssTopicAdapter);a.target='/hardware/gnss/fix';a.received=False
        a.pub=Mock();a.get_logger=Mock(return_value=Mock())
        a.get_publishers_info_by_topic=Mock(return_value=[])
        a.get_name=lambda:'nxk_gnss_topic_adapter';a.get_namespace=lambda:'/'
        return a
    def test_no_fix_preserved(self):
        a=self.adapter();m=types.SimpleNamespace(status=types.SimpleNamespace(status=-1),latitude=math.nan)
        a.forward(m);a.pub.publish.assert_called_once_with(m)
        self.assertTrue(math.isnan(m.latitude));self.assertEqual(m.status.status,-1)
    def test_valid_fix_preserved(self):
        a=self.adapter();m=types.SimpleNamespace(status=types.SimpleNamespace(status=0),latitude=34.1,longitude=108.8)
        a.forward(m);self.assertIs(a.pub.publish.call_args.args[0],m)
    def test_conflict_rejected(self):
        a=self.adapter();a.get_publishers_info_by_topic.return_value=[types.SimpleNamespace(node_name='old_bridge',node_namespace='/')]
        with self.assertRaises(RuntimeError):a.forward(object())
        a.pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
