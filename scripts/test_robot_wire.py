"""Pure protocol tests. Does not open sockets, import ROS, or command hardware."""
import json
import math
import time
import unittest
from robot_wire import encode, decode, velocity


class WireTests(unittest.TestCase):
    key = b'test-only-not-a-deployment-secret-12345'

    def test_roundtrip(self):
        v = {'time': time.time(), 'kind': 'hello'}
        self.assertEqual(decode(encode(v, self.key), self.key), v)

    def test_wrong_key(self):
        with self.assertRaises(ValueError):
            decode(encode({'time':time.time()},self.key),b'other')

    def test_tamper(self):
        p=json.loads(encode({'time':time.time(),'kind':'stop'},self.key))
        p['body']=p['body'].replace('stop','drive')
        with self.assertRaises(ValueError):decode(json.dumps(p).encode(),self.key)

    def test_old_future_nonfinite(self):
        for stamp in (time.time()-1,time.time()+1,'invalid',True):
            with self.assertRaises(ValueError):decode(encode({'time':stamp},self.key),self.key)

    def test_size(self):
        with self.assertRaises(ValueError):decode(b'x'*4097,self.key)

    def test_malformed_envelope(self):
        for packet in (b'[]',b'null',b'{"body":3,"mac":"x"}',b'{"body":"{}","mac":null}'):
            with self.assertRaises(ValueError):decode(packet,self.key)

    def test_speed(self):
        self.assertEqual(velocity(.2,-.3),(.2,-.3))
        self.assertEqual(velocity(1.,.5),(1.,.5))
        for v,w in [(-.1,0),(1.01,0),(0,.51),(math.nan,0),(0,math.inf),(True,0)]:
            with self.assertRaises(ValueError):velocity(v,w)


if __name__ == '__main__':unittest.main()
