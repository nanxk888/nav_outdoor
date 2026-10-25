"""Local HTTP smoke test with fake ROS imports; never starts a ROS node."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import struct
import time
import types
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError


def load():
    modules = {'rclpy': types.ModuleType('rclpy')}
    for name, symbols in [('rclpy.qos',['qos_profile_sensor_data']),
                          ('sensor_msgs.msg',['NavSatFix','PointCloud2']),('nav_msgs.msg',['Odometry','Path']),
                          ('std_msgs.msg',['String'])]:
        m=types.ModuleType(name)
        for s in symbols:setattr(m,s,object)
        modules[name]=m
    with patch.dict(sys.modules,modules):
        spec=importlib.util.spec_from_file_location('web_under_test',Path(__file__).with_name('navigation_web.py'))
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m


class WebTests(unittest.TestCase):
    def test_lio_epoch_changes_on_gap_frame_or_reset(self):
        def message(stamp,x=0,frame='lio_odom'):
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)),frame_id=frame),
                child_frame_id='lio_imu',pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=x,y=0,z=0),orientation=types.SimpleNamespace(x=0,y=0,z=0,w=1))))
        self.m.previous_lio=None
        self.m.odom(message(10));first=self.m.samples['lio']['value']['epoch']
        self.m.odom(message(10.1));self.assertEqual(self.m.samples['lio']['value']['epoch'],first)
        self.m.odom(message(12));second=self.m.samples['lio']['value']['epoch'];self.assertNotEqual(second,first)
        self.m.odom(message(12.1,x=3));third=self.m.samples['lio']['value']['epoch'];self.assertNotEqual(second,third)
        self.m.odom(message(12.2,x=3,frame='new'));self.assertNotEqual(third,self.m.samples['lio']['value']['epoch'])

    def test_cloud_preview_layout_and_nan(self):
        field=lambda n,i:types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1)
        stamp=time.time();sec=int(stamp)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='lio_odom',stamp=types.SimpleNamespace(sec=sec,nanosec=int((stamp-sec)*1e9))),
             fields=[field(n,i) for i,n in enumerate(('x','y','z'))],width=1,height=2,point_step=12,row_step=16,is_bigendian=False,
             data=struct.pack('<fff',1.,2.,3.)+b'\0'*4+struct.pack('<fff',float('nan'),0.,0.)+b'\0'*4)
        self.m.last_cloud_receive=-100.;self.m.cloud(msg)
        result=self.m.samples['nav_cloud']['value']
        self.assertTrue(result['valid']);self.assertEqual(result['points'],[[1.,2.,3.]])
        msg.header.stamp.sec-=10;self.m.last_cloud_receive=-100.;self.m.cloud(msg)
        self.assertFalse(self.m.samples['nav_cloud']['value']['valid'])

    def test_preview_points_bounded(self):
        stamp=time.time();sec=int(stamp)
        fields=[types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
        msg=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='lio_odom',stamp=types.SimpleNamespace(sec=sec,nanosec=int((stamp-sec)*1e9))),
             fields=fields,width=20001,height=1,point_step=12,row_step=20001*12,is_bigendian=False,data=struct.pack('<fff',1.,2.,3.)*20001)
        self.m.last_cloud_receive=-100.;self.m.cloud(msg)
        self.assertLessEqual(len(self.m.samples['nav_cloud']['value']['points']),2000)
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
        for path in ('/','/map.js','/geo_alignment.js','/control.js','/viewport.js','/dashboard.js','/dashboard.css'):
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

    def test_local_candidate_goal_requires_ready_epoch(self):
        old_pub=self.m.memory_goal_pub;old_string=self.m.String;received=[]
        self.m.memory_goal_pub=types.SimpleNamespace(publish=lambda m:received.append(json.loads(m.data)))
        self.m.String=types.SimpleNamespace
        try:
            self.m.save('memory',json.dumps(dict(ready=True,frame='lio_odom',epoch='test')))
            self.m.save('lio',dict(frame='lio_odom',position=[0.,0.,0.]))
            with self.post('/api/operator/lio_goal',dict(x=1.,y=2.,frame='lio_odom',epoch='test')) as r:
                reply=json.load(r);self.assertTrue(reply['candidate_only']);self.assertFalse(reply['executed'])
            self.assertEqual(len(received),1)
            with self.assertRaises(HTTPError):self.post('/api/operator/lio_goal',dict(x=1.,y=2.,frame='lio_odom',epoch='old'))
            self.assertEqual(len(received),1)
        finally:self.m.memory_goal_pub=old_pub;self.m.String=old_string

    def test_memory_reset_without_motion(self):
        old_pub=self.m.memory_reset_pub;old_string=self.m.String;received=[]
        self.m.memory_reset_pub=types.SimpleNamespace(publish=lambda m:received.append(json.loads(m.data)))
        self.m.String=types.SimpleNamespace
        try:
            self.m.save('memory',json.dumps(dict(ready=False,frame='lio_odom',epoch='reset-test')))
            with self.post('/api/operator/memory_reset',{}) as r:
                reply=json.load(r);self.assertTrue(reply['queued']);self.assertFalse(reply['odometry_reset'])
            self.assertEqual(received[0]['epoch'],'reset-test')
            with self.assertRaises(HTTPError):self.post('/api/operator/memory_reset',{'reset_odometry':True})
            self.assertEqual(len(received),1)
        finally:self.m.memory_reset_pub=old_pub;self.m.String=old_string

    def test_no_origin(self):
        with self.assertRaises(HTTPError) as c:self.post('/api/operator/heading',{'heading_deg':0},False)
        self.assertEqual(c.exception.code,403)

    def test_reference_registration_queued_not_accepted(self):
        old_pub=self.m.registration_pub;old_string=self.m.String;received=[]
        self.m.registration_pub=types.SimpleNamespace(publish=lambda msg:received.append(json.loads(msg.data)))
        self.m.String=lambda **kw:types.SimpleNamespace(**kw)
        try:
            self.m.save('gps',{'valid':True,'stamp':100.1})
            self.m.save('lio',{'stamp':100.2})
            with self.post('/api/operator/register',{'heading_deg':90}) as response:
                self.assertEqual(response.status,202);self.assertTrue(json.load(response)['queued'])
            self.assertEqual(received[0]['lio_stamp'],100.2)
            self.assertEqual(received[0]['gps_stamp'],100.1)
            with self.assertRaises(HTTPError):self.post('/api/operator/register',{'heading_deg':360})
            self.m.save('gps',{'valid':False,'stamp':100.1})
            with self.assertRaises(HTTPError):self.post('/api/operator/register',{'heading_deg':90})
            self.assertEqual(len(received),1)
        finally:self.m.registration_pub=old_pub;self.m.String=old_string

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
