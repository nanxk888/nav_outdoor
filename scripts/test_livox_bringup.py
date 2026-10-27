"""Offline guard contracts; no ROS, sensor connections, or process signals."""
import json
import os
import struct
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from ensure_livox_and_lio import drivers,compatible,check_network,SensorGate,stop_child,main,lidar_header


class GuardTests(unittest.TestCase):
    def test_metadata_parser_both_endians_and_alignment(self):
        for endian,enc in (('<',b'\0\1\0\0'),('>',b'\0\0\0\0')):
            for frame in ('livox_frame','x','longer_frame_name'):
                name=frame.encode()+b'\0'
                body=struct.pack(endian+'iII',100,123,len(name))+name
                body+=b'\0'*((-len(body))%8)
                body+=struct.pack(endian+'QIB3sI',100,2,0,b'\0'*3,2)+b'\0'*39
                self.assertEqual(lidar_header(enc+body),(100+123/1e9,frame,2))
                with self.assertRaises(ValueError):lidar_header((enc+body)[:-1])
    def test_metadata_parser_against_actual_ros_serializer(self):
        try:
            from rclpy.serialization import serialize_message
            from livox_ros_driver2.msg import CustomMsg,CustomPoint
        except ImportError:self.skipTest('Requires ROS Livox type support')
        for frame in ('livox_frame','x','longer_frame_name'):
            for n in (1,20000):
                m=CustomMsg();m.header.frame_id=frame;m.header.stamp.sec=100;m.header.stamp.nanosec=123
                m.point_num=n;m.points=[CustomPoint()]*n
                self.assertEqual(lidar_header(serialize_message(m)),(100+123/1e9,frame,n))
    def test_rejection_diagnostics_do_not_hide_missing_lidar(self):
        g=SensorGate();g.observe('lidar',100,101,1)
        self.assertEqual(g.detail(1)['lidar']['count'],0)
        self.assertIn('stale',g.detail(1)['lidar']['last_reason'])
        self.assertEqual(g.detail(1)['lidar']['rejected'],1)
    def test_network_failure_never_starts_driver_or_lio(self):
        with tempfile.TemporaryDirectory() as folder:
            xml=Path(folder)/'dds.xml';xml.write_text('<CycloneDDS/>')
            argv=['guard','--setup',folder+'/setup.bash','--config',folder+'/mid.json','--xml',str(xml)]
            with patch('sys.argv',argv),patch('ensure_livox_and_lio.check_network',side_effect=ValueError('link down')),\
                 patch('ensure_livox_and_lio.subprocess.Popen') as spawn,patch('ensure_livox_and_lio.signal.signal'),patch('builtins.print'):
                self.assertEqual(main(),1);spawn.assert_not_called()
    def test_wrapper_not_counted_as_second_driver(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for pid,args in ((1,['/usr/bin/python3','ros2','run','livox_ros_driver2','livox_ros_driver2_node']),
                             (2,['/install/lib/livox_ros_driver2/livox_ros_driver2_node'])):
                d=root/str(pid);d.mkdir();(d/'cmdline').write_bytes(b'\0'.join(a.encode() for a in args));(d/'environ').write_bytes(b'ROS_DOMAIN_ID=0\0')
            self.assertEqual([p for p,_,_ in drivers(root)],[2])
    def test_existing_dds_conflict_rejected(self):
        xml=Path('/config/cyclonedds.xml').resolve();config=Path('/config/mid.json').resolve()
        env=dict(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI=xml.as_uri())
        compatible((12,['driver','user_config_path:='+str(config)],env),xml,config)
        env['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
        with self.assertRaisesRegex(ValueError,'conflicts'):compatible((12,[],env),xml,config)
    def test_network_missing_does_not_modify_network(self):
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'mid.json';config.write_text(json.dumps(dict(MID360=dict(host_net_info=dict(cmd_data_ip='192.168.1.50',point_data_ip='192.168.1.50',imu_data_ip='192.168.1.50')),lidar_configs=[dict(ip='192.168.1.137')])))
            with patch('ensure_livox_and_lio.subprocess.check_output',return_value='[]'):
                with self.assertRaisesRegex(ValueError,'missing or link down'):check_network(config)
    def test_both_streams_must_advance_before_start(self):
        g=SensorGate()
        for i in range(220):
            t=i*.005;g.observe('imu',100+t,100+t,t)
            if i%20==0:g.observe('lidar',100+t,100+t,t)
        self.assertTrue(g.ready(101.1,1.1))
        self.assertFalse(g.ready(102.,2.))
        g.observe('lidar',100.,101.1,1.1)
        self.assertFalse(g.ready(101.1,1.1))
    def test_imu_alone_not_ready(self):
        g=SensorGate()
        for i in range(300):g.observe('imu',100+i*.005,100+i*.005,i*.005)
        self.assertFalse(g.ready(101.5,1.5))
    @unittest.skipUnless(hasattr(os,'killpg'),'Linux process groups')
    def test_cleanup_only_explicit_owned_live_child(self):
        child=Mock();child.pid=123;child.poll.return_value=None
        with patch('ensure_livox_and_lio.os.killpg') as kill:
            stop_child(None);kill.assert_not_called()
            stop_child(child);kill.assert_called_once()
            self.assertEqual(kill.call_args.args[0],123)
            child.wait.assert_called_once()
    @unittest.skipUnless(hasattr(os,'killpg'),'Linux process groups')
    def test_dead_owned_child_not_signalled(self):
        child=Mock();child.poll.return_value=1
        with patch('ensure_livox_and_lio.os.killpg') as kill:
            stop_child(child);kill.assert_not_called()


if __name__=='__main__':unittest.main()
