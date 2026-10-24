"""No ROS/hardware: receive-side motion state regression."""
import unittest
from robot_policy import ReceiverPolicy,posture,valid_fsm_event


class PolicyTests(unittest.TestCase):
    def test_latched_fsm_confirmation(self):
        event=('loco',1000,1.)
        self.assertTrue(valid_fsm_event(event,True,0,2.))
        self.assertFalse(valid_fsm_event(event,False,0,2.))
        self.assertFalse(valid_fsm_event(event,True,1001,2.))
        self.assertFalse(valid_fsm_event(event,True,0,1.1))
        self.assertFalse(valid_fsm_event(None,True,0,2.))
    def setup_policy(self):
        p=ReceiverPolicy(True,True);p.feedback('idle',0);p.interfaces(True,True,0)
        packet=dict(protocol=2,kind='arm',nonce=p.nonce,seq=1,session='test',operation='a')
        p.accept(packet,0);return p,packet
    def test_disabled(self):
        p=ReceiverPolicy();p.feedback('idle',0);p.interfaces(True,True,0)
        with self.assertRaises(ValueError):p.accept(dict(protocol=2,kind='arm',nonce=p.nonce,seq=1,session='s'),0)
    def test_modes_once(self):
        p,d=self.setup_policy();d.update(kind='drive',mode='transform_up',seq=2)
        p.accept(d,.01);self.assertEqual(p.tick(.01)[1],'transform_up')
        d['seq']=3;p.accept(d,.02);self.assertEqual(p.tick(.02)[1],'')
    def test_replay(self):
        p,d=self.setup_policy()
        with self.assertRaises(ValueError):p.accept(d,.01)
    def test_sdk_required(self):
        p,d=self.setup_policy();p.interfaces(False,True,.01);p.tick(.01);self.assertFalse(p.active)
    def test_no_auto_resume(self):
        p,d=self.setup_policy();p.tick(.3);self.assertFalse(p.active)
        d.update(kind='drive',seq=2)
        with self.assertRaises(ValueError):p.accept(d,.31)
    def test_lower_needs_confirmation_and_zero_hold(self):
        p,d=self.setup_policy();p.feedback('loco',0)
        d.update(kind='drive',seq=2,mode='transform_down',operation='down',confirm_stationary=True)
        with self.assertRaises(ValueError):p.accept(d,.1)
        # Keep alive at zero until the command hold interval has elapsed.
        for i,t in enumerate((.1,.2,.3,.4,.5),2):
            p.accept(dict(d,seq=i,mode=''),t);p.tick(t)
        p.accept(dict(d,seq=7),.6);self.assertEqual(p.tick(.6)[1],'transform_down')
    def test_pose_validation(self):
        for value in ({'roll_rad':.21},{'pitch_rad':float('nan')},{'height_m':.2},{'pitch_rad':True}):
            with self.assertRaises(ValueError):posture(value)
        self.assertAlmostEqual(sum(v*v for v in posture({'pitch_rad':.2})['xyzw']),1)
    def test_pose_loco_only(self):
        p,d=self.setup_policy()
        with self.assertRaises(ValueError):p.accept(dict(d,kind='drive',seq=2,pose={'pitch_rad':.1}),.6)
    def test_velocity_rejected_while_lowering(self):
        p,d=self.setup_policy();p.feedback('loco',0);p.last_mode='transform_down'
        with self.assertRaises(ValueError):p.accept(dict(d,kind='drive',seq=2,v=.1),.01)

    def test_pose_slew_and_neutral_before_lower(self):
        p,d=self.setup_policy();p.feedback('loco',0)
        for i,t in enumerate((.1,.2,.3,.4,.5),2):
            p.accept(dict(d,kind='drive',seq=i),t);p.tick(t)
        p.accept(dict(d,kind='drive',seq=7,pose={'pitch_rad':.1}),.6)
        p.tick(.6);self.assertAlmostEqual(p.pose['pitch_rad'],.025)
        with self.assertRaises(ValueError):
            p.accept(dict(d,kind='drive',seq=8,v=.1),.61)
        with self.assertRaises(ValueError):
            p.accept(dict(d,kind='drive',seq=8,mode='transform_down',confirm_stationary=True),.61)
        p.halt('test');self.assertEqual(p.pose_target,p.pose)

    def test_old_protocol_rejected(self):
        p,d=self.setup_policy()
        with self.assertRaises(ValueError):p.accept(dict(d,protocol=1,seq=2),.01)


if __name__=='__main__':unittest.main()
