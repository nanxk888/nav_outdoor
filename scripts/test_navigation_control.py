"""Fake ROS/UDP coordinator regression. No robot connections or ROS publishers."""
import importlib.util
import os
from pathlib import Path
import queue
import sys
import time
import types
import unittest
from unittest.mock import patch


def load_control():
    replacements = {}
    for package, symbol in [('geometry_msgs.msg','Twist'),('std_msgs.msg','String'),
                            ('nav2_msgs.action','NavigateToPose'),('rclpy.action','ActionClient')]:
        module=types.ModuleType(package);setattr(module,symbol,type(symbol,(),{}));replacements[package]=module
    with patch.dict(sys.modules,replacements):
        spec=importlib.util.spec_from_file_location('control_test',Path(__file__).with_name('navigation_control.py'))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod.Control


Control=load_control()


class FakeSocket:
    def recvfrom(self, _): raise BlockingIOError


class ControlTests(unittest.TestCase):
    def instance(self, phase='READY'):
        c=Control.__new__(Control)
        c.node=types.SimpleNamespace(get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=100*10**9)))
        c.sock=FakeSocket();c.requests=queue.Queue(16);c.phase=phase;c.reason='';c.state_since=time.monotonic()
        c.browser=time.monotonic();c.robot_time=time.monotonic();c.cmd_time=0.;c.cmd=(0.,0.)
        c.robot={'fsm':'loco','fsm_fresh':True,'fault':False,'authorized':True};c.nonce='x'
        c.goal_handle=None;c.generation=0;c.pending=False;c.seq=0;c.safety={};c.safety_time=0
        c.sent=[];c.send=lambda *args,**kwargs:c.sent.append((args,kwargs))
        return c

    def test_disabled_without_config(self):
        with patch.dict(os.environ,{},clear=True): c=Control(object())
        self.assertEqual(c.phase,'DISABLED')
        with self.assertRaises(ValueError):c.enqueue('start',{})

    def test_ready_zero(self):
        c=self.instance();c.tick()
        self.assertEqual(c.sent[-1],(('drive',),{'v':0.,'w':0.}))

    def test_stand_four_second_fallback_requires_ack(self):
        c=self.instance('STANDING');c.config={'stand_ready_states':[],'stand_loco_delay_sec':4.}
        c.operation='up';c.stand_sent_at=time.monotonic()-3.9
        c.robot.update(fsm='transform_up',fsm_valid=True,operation='up',last_mode='transform_up')
        c.tick();self.assertEqual(c.phase,'STANDING')
        c.stand_sent_at=time.monotonic()-4.1;c.tick();self.assertEqual(c.phase,'ENTERING_LOCO')

    def test_stand_timer_does_not_use_old_operation(self):
        c=self.instance('STANDING');c.config={'stand_ready_states':[]};c.operation='new'
        c.stand_sent_at=time.monotonic()-5
        c.robot.update(fsm='transform_up',fsm_valid=True,operation='old',last_mode='transform_up')
        c.tick();self.assertEqual(c.phase,'STANDING')

    def test_existing_transform_up_waits_without_repeating_stand(self):
        c=self.instance('STAND_WAIT');c.config={'stand_loco_delay_sec':4.}
        c.robot.update(fsm='transform_up',fsm_valid=True)
        c.state_since=time.monotonic()-3.9;c.tick()
        self.assertEqual(c.phase,'STAND_WAIT');self.assertEqual(c.sent[-1][0],('drive',))
        c.state_since=time.monotonic()-4.1;c.tick()
        self.assertEqual(c.phase,'ENTERING_LOCO')

    def test_arm_existing_transform_up_does_not_reissue_stand(self):
        c=self.instance('ARMING');c.session='s'
        c.robot.update(fsm='transform_up',session='s',active=True)
        c.tick();self.assertEqual(c.phase,'STAND_WAIT')

    def test_acknowledged_stand_sends_zero_not_another_mode(self):
        c=self.instance('STANDING');c.config={'stand_ready_states':[]};c.operation='op'
        c.robot.update(fsm='transform_up',fsm_valid=True,operation='op',last_mode='transform_up')
        c.tick();self.assertEqual(c.sent[-1][0],('drive',''))

    def test_browser_timeout(self):
        c=self.instance();c.browser-=3;c.tick()
        self.assertEqual(c.phase,'STOPPED');self.assertEqual(c.sent[0][0],('stop',))

    def test_robot_timeout(self):
        c=self.instance();c.robot_time-=1;c.tick();self.assertEqual(c.phase,'STOPPED')

    def test_no_fusion_no_navigation(self):
        c=self.instance('NAVIGATING');c.tick();self.assertEqual(c.phase,'STOPPED')

    def test_safety_requires_measurement_time(self):
        c=self.instance();c.safety_time=time.monotonic()
        c.safety={'navigation_ready':True,'base_link_ready':True,'map_alignment_verified':True,'stamp':99.9}
        self.assertTrue(c.safe());c.safety['stamp']=90;self.assertFalse(c.safe())

    def test_no_auto_resume(self):
        c=self.instance('STOPPED');c.tick();self.assertEqual(c.phase,'STOPPED')
        self.assertEqual(c.sent[-1][0],('hello',))

    def test_candidate_timeout(self):
        c=self.instance('NAVIGATING');c.safety_time=time.monotonic()
        c.safety={'navigation_ready':True,'base_link_ready':True,'map_alignment_verified':True,'stamp':99.9}
        c.cmd_time=time.monotonic()-.3;c.tick();self.assertEqual(c.phase,'STOPPED')

    def test_pose_ready_only(self):
        c=self.instance();c.robot['pose_enabled']=True;c.state_since-=1
        c.requests.put(('pose',{'pitch_rad':.1}));c.tick()
        self.assertEqual(c.sent[-1][1]['pose'],{'pitch_rad':.1})

    def test_lower_navigation_rejected(self):
        c=self.instance('NAVIGATING');c.safety_time=time.monotonic()
        c.safety={'navigation_ready':True,'base_link_ready':True,'map_alignment_verified':True,'stamp':99.9}
        c.requests.put(('lower',{'confirm_stationary':True}));c.tick()
        self.assertEqual(c.phase,'NAVIGATING')
        self.assertIn('Stop navigation',c.reason)

    def test_lower_zero_braking(self):
        c=self.instance();c.requests.put(('lower',{'confirm_stationary':True}));c.tick()
        self.assertEqual(c.phase,'LOWER_BRAKING');self.assertEqual(c.sent[-1][0],('drive',))
        c.state_since-=1;c.tick();self.assertEqual(c.phase,'LOWERING')
        c.tick();self.assertEqual(c.sent[-1][0],('drive','transform_down'))
        self.assertTrue(c.sent[-1][1]['confirm_stationary'])


if __name__=='__main__':unittest.main()
