import unittest
from navigation_speed import validate_limits,limit_velocity,DEFAULT_LIMITS


class SpeedTests(unittest.TestCase):
    def test_bounds(self):
        self.assertEqual(validate_limits(DEFAULT_LIMITS),DEFAULT_LIMITS)
        self.assertEqual(validate_limits(dict(linear_mps=1.,angular_rps=.5))['linear_mps'],1.)
        for value in (0.,-.1,.09,1.01,float('nan'),True):
            with self.assertRaises(ValueError):validate_limits(dict(linear_mps=value,angular_rps=.2))
        with self.assertRaises(ValueError):validate_limits(dict(linear_mps=.1,angular_rps=.51))
    def test_both_modes_and_curve_preserved(self):
        for v,w in ((.15,.35),(.3,.5),(.2,-.4),(0.,.5),(0.,0.)):
            a,b=limit_velocity(v,w,dict(linear_mps=.08,angular_rps=.2))
            self.assertLessEqual(a,.080001);self.assertLessEqual(abs(b),.200001)
            self.assertAlmostEqual(a*w,b*v)
    def test_invalid_schema(self):
        for value in ({},dict(DEFAULT_LIMITS,extra=1),None):
            with self.assertRaises(ValueError):validate_limits(value)


if __name__=='__main__':unittest.main()
