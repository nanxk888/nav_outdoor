"""GNSS gate regression, independent of ROS and hardware."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'/'d1h_global_localization'))
from d1h_global_localization.gnss_quality import GnssGate, antenna_enu


class GnssTests(unittest.TestCase):
    def fix(self,gate=None,**changes):
        args=dict(now=100.,stamp=100.,frame='gps_antenna',status=0,lat=34.125,lon=108.829,
                  covariance=[4.,0.,0.,0.,4.,0.,0.,0.,9.],covariance_type=2)
        args.update(changes)
        return (gate or GnssGate()).check(**args)

    def test_valid(self):self.assertTrue(self.fix()[0])
    def test_invalid_fix(self):self.assertEqual(self.fix(status=-1)[1],'no_fix')
    def test_stale(self):self.assertFalse(self.fix(stamp=98.)[0])
    def test_future(self):self.assertFalse(self.fix(stamp=101.)[0])
    def test_unknown_rejected(self):self.assertFalse(self.fix(covariance_type=0)[0])
    def test_unknown_explicit_fallback(self):
        result=self.fix(GnssGate(allow_unknown=True),covariance_type=0)
        self.assertTrue(result[0]);self.assertEqual(result[2][0],25.);self.assertTrue(result[3])
    def test_zero_covariance_rejected(self):self.assertFalse(self.fix(covariance=[0.]*9)[0])
    def test_non_psd(self):self.assertFalse(self.fix(covariance=[4.,5.,0.,5.,4.,0.,0.,0.,9.])[0])
    def test_variance_floor(self):
        result=self.fix(covariance=[.0001,0.,0.,0.,.0001,0.,0.,0.,.0001])
        self.assertEqual(result[2][0],1.)
    def test_uncertain(self):self.assertFalse(self.fix(covariance=[121.,0.,0.,0.,4.,0.,0.,0.,9.])[0])
    def test_jump_does_not_update_reference(self):
        gate=GnssGate();self.fix(gate)
        self.assertEqual(self.fix(gate,now=101.,stamp=101.,lat=34.135)[1],'position_jump')
        self.assertTrue(self.fix(gate,now=102.,stamp=102.)[0])
    def test_duplicate(self):
        gate=GnssGate();self.fix(gate)
        self.assertFalse(self.fix(gate)[0])
    def test_frame(self):self.assertFalse(self.fix(frame='base_link')[0])
    def test_preview_origin(self):self.assertEqual(antenna_enu(34.,108.,34.,108.),(0.,0.))


if __name__=='__main__':unittest.main()
