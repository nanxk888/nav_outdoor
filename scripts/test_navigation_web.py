"""Local HTTP smoke test with fake ROS imports; never starts a ROS node."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError


def load():
    modules = {'rclpy': types.ModuleType('rclpy')}
    for name, symbols in [('rclpy.qos',['qos_profile_sensor_data']),
                          ('sensor_msgs.msg',['NavSatFix']),('nav_msgs.msg',['Odometry','Path']),
                          ('std_msgs.msg',['String'])]:
        m=types.ModuleType(name)
        for s in symbols:setattr(m,s,object)
        modules[name]=m
    with patch.dict(sys.modules,modules):
        spec=importlib.util.spec_from_file_location('web_under_test',Path(__file__).with_name('navigation_web.py'))
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m


class WebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m=load();cls.m.osm_data=cls.m.load_osm()
        cls.m.control=types.SimpleNamespace(status=lambda:{'enabled':False})
        cls.server=cls.m.ThreadingHTTPServer(('127.0.0.1',0),cls.m.Handler)
        cls.base='http://127.0.0.1:'+str(cls.server.server_port)
        cls.thread=threading.Thread(target=cls.server.serve_forever);cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join()

    def post(self, path, body, origin=True, token=None):
        headers={'Content-Type':'application/json'}
        if token:headers['Authorization']='Bearer '+token
        if origin:headers['Origin']=self.base
        return urlopen(Request(self.base+path,data=json.dumps(body).encode(),headers=headers),timeout=2)

    def test_assets(self):
        for path in ('/','/map.js','/control.js','/viewport.js','/dashboard.js','/dashboard.css'):
            with urlopen(self.base+path,timeout=2) as r:self.assertGreater(len(r.read()),100)

    def test_osm(self):
        with urlopen(self.base+'/api/osm',timeout=2) as r:data=json.load(r)
        self.assertEqual(data['type'],'FeatureCollection');self.assertGreater(len(data['features']),0)

    def test_goal_preview(self):
        with self.post('/api/operator/goal',{'latitude':34.125,'longitude':108.829}) as r:
            self.assertFalse(json.load(r)['executed'])

    def test_invalid_coordinate(self):
        with self.assertRaises(HTTPError) as c:self.post('/api/operator/goal',{'latitude':999,'longitude':1})
        self.assertEqual(c.exception.code,400)

    def test_no_origin(self):
        with self.assertRaises(HTTPError) as c:self.post('/api/operator/heading',{'heading_deg':0},False)
        self.assertEqual(c.exception.code,403)

    def test_no_control_token(self):
        with patch.dict('os.environ',{},clear=True):
            with self.assertRaises(HTTPError) as c:self.post('/api/control/start',{})
        self.assertEqual(c.exception.code,400)

    def test_control_still_requires_same_origin(self):
        with self.assertRaises(HTTPError) as c:self.post('/api/control/start',{},False)
        self.assertEqual(c.exception.code,403)

    def test_state(self):
        with urlopen(self.base+'/api/state',timeout=2) as r:data=json.load(r)
        self.assertTrue(data['read_only']);self.assertFalse(data['control']['enabled'])

    def test_pose_lower_authenticated_validation(self):
        token='local-test-token-not-a-secret-123456789'
        old=self.m.control;received=[]
        self.m.control=types.SimpleNamespace(status=lambda:{'enabled':True},enqueue=lambda kind,value:received.append((kind,value)) or {'queued':True})
        try:
            with patch.dict('os.environ',{'NXK_WEB_TOKEN':token}):
                with self.post('/api/control/pose',{'pitch_rad':.1}) as r:self.assertEqual(r.status,202)
                with self.post('/api/control/lower',{'confirm_stationary':True}) as r:self.assertEqual(r.status,202)
                for path,body in (('pose',{'roll_rad':.21}),('lower',{})):
                    with self.assertRaises(HTTPError) as c:self.post('/api/control/'+path,body,token=token)
                    self.assertEqual(c.exception.code,400)
            self.assertEqual([k for k,_ in received],['pose','lower'])
        finally:self.m.control=old


if __name__=='__main__':unittest.main()
