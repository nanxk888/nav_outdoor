"""Loopback dashboard with explicit guarded control and non-motion candidate requests."""
import json
import math
import os
import queue
import xml.etree.ElementTree as ET
import threading
import time
import struct
import uuid
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix,PointCloud2
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
lock = threading.Lock()
samples = {}
operator_settings = {}
control = None
memory_goal_pub = None
memory_reset_pub = None
registration_pub = None
last_cloud_receive=-100.
lio_epoch=uuid.uuid4().hex
previous_lio=None


def cloud(msg):
    global last_cloud_receive
    now=time.monotonic()
    if now-last_cloud_receive<.45:return
    last_cloud_receive=now
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    age=time.time()-stamp
    fields={f.name:f for f in msg.fields}
    points=[]
    try:
        if not -.1<=age<=.5:raise ValueError('stale cloud measurement')
        count=msg.width*msg.height
        if count>100000 or len(msg.data)>8000000 or msg.point_step<=0:raise ValueError('cloud budget/layout')
        if msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:raise ValueError('truncated cloud')
        formats=[]
        for name in ('x','y','z'):
            f=fields[name]
            if f.datatype not in (7,8) or f.count!=1:raise ValueError('XYZ must be float32/64')
            size=4 if f.datatype==7 else 8
            if f.offset<0 or f.offset+size>msg.point_step:raise ValueError('invalid field offset')
            formats.append((f.offset,('>' if msg.is_bigendian else '<')+('f' if size==4 else 'd')))
        for i in range(0,count,max(1,math.ceil(count/2000))):
            offset=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
            xyz=[struct.unpack_from(fmt,msg.data,offset+off)[0] for off,fmt in formats]
            if all(math.isfinite(v) for v in xyz):points.append([round(v,3) for v in xyz])
        value={'valid':True,'frame':msg.header.frame_id,'stamp':stamp,'measurement_age_sec':age,
               'points':points,'input_points':count,'preview_only':True}
    except (ValueError,KeyError,struct.error) as exc:
        value={'valid':False,'points':[],'reason':str(exc),'preview_only':True}
    save('nav_cloud',value)


def load_osm():
    tree = ET.parse(ROOT / 'map/xd_map.osm').getroot()
    nodes = {n.attrib['id']: [float(n.attrib['lon']), float(n.attrib['lat'])]
             for n in tree.findall('node')}
    features = []
    for way in tree.findall('way'):
        tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
        if 'highway' not in tags and 'building' not in tags:
            continue
        points = [nodes[n.attrib['ref']] for n in way.findall('nd') if n.attrib['ref'] in nodes]
        if len(points) >= 2:
            features.append({'type': 'Feature', 'properties': tags,
                             'geometry': {'type': 'LineString', 'coordinates': points}})
    return json.dumps({'type': 'FeatureCollection', 'features': features}).encode()


osm_data = None


def save(name, value):
    with lock:
        samples[name] = {'received_monotonic': time.monotonic(), 'value': value}


def fix(msg):
    valid = (msg.status.status >= 0 and math.isfinite(msg.latitude)
             and math.isfinite(msg.longitude) and -90 <= msg.latitude <= 90
             and -180 <= msg.longitude <= 180)
    save('gps', {'valid': valid, 'latitude': msg.latitude if valid else None,
                 'longitude': msg.longitude if valid else None,
                 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                 'frame': msg.header.frame_id,
                 'covariance': [v if math.isfinite(v) else None for v in msg.position_covariance],
                 'covariance_type': msg.position_covariance_type})


def odom(msg):
    global lio_epoch, previous_lio
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    now=time.monotonic()
    if previous_lio is not None:
        old_stamp,old_rx,old_frame,old_pos=previous_lio
        dt=stamp-old_stamp
        if dt<=0 or dt>1 or now-old_rx>1 or old_frame!=msg.header.frame_id or math.dist(old_pos,(p.x,p.y,p.z))>.5+2*max(0,dt):
            lio_epoch=uuid.uuid4().hex
    previous_lio=(stamp,now,msg.header.frame_id,(p.x,p.y,p.z))
    save('lio', {'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w],
                 'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
                 'stamp': stamp,'epoch':lio_epoch})


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        # Browser same-origin protection; loopback service is intended for SSH tunnelling.
        if self.headers.get('Origin') != 'http://' + self.headers.get('Host', ''):
            self.send_error(403)
            return
        is_control = self.path.startswith('/api/control/')
        if self.path not in ('/api/operator/goal', '/api/operator/heading','/api/operator/lio_goal','/api/operator/memory_reset','/api/operator/register') and not is_control:
            self.send_error(404)
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024:
                raise ValueError('invalid body length')
            value = json.loads(self.rfile.read(size))
            if self.path=='/api/operator/register':
                if set(value)!={'heading_deg'} or type(value['heading_deg']) not in (int,float) or not math.isfinite(value['heading_deg']) or not 0<=value['heading_deg']<360:
                    raise ValueError('Invalid bearing')
                with lock:g=samples.get('gps');o=samples.get('lio')
                if not g or not o or not g['value']['valid'] or registration_pub is None or any(time.monotonic()-s['received_monotonic']>1 for s in (g,o)):
                    raise ValueError('Fresh GPS/LIO required for registration')
                registration_pub.publish(String(data=json.dumps(dict(heading_deg=value['heading_deg'],lio_stamp=o['value']['stamp'],gps_stamp=g['value']['stamp'],request_time=time.time()))))
                body=json.dumps({'queued':True,'reason':'Registration requested; inspect live reference status for acceptance'}).encode()
                self.send_response(202);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
                self.end_headers();self.wfile.write(body);return
            if self.path=='/api/operator/memory_reset':
                if value!={}:raise ValueError('Reset body must be empty')
                with lock:record=samples.get('memory')
                if not record or time.monotonic()-record['received_monotonic']>1 or memory_reset_pub is None:raise ValueError('Memory node unavailable')
                info=json.loads(record['value'])
                memory_reset_pub.publish(String(data=json.dumps(dict(epoch=info['epoch'],request_time=time.time()))))
                body=json.dumps({'queued':True,'odometry_reset':False,'reason':'Memory and candidate goal reset requested; odometry unchanged'}).encode()
                self.send_response(202);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
                self.end_headers();self.wfile.write(body);return
            if self.path=='/api/operator/lio_goal':
                if set(value)!={'x','y','frame','epoch'} or any(type(value[k]) not in (int,float) or not math.isfinite(value[k]) for k in ('x','y')):
                    raise ValueError('Invalid local goal')
                with lock:record=samples.get('memory');local=samples.get('lio')
                if not record or time.monotonic()-record['received_monotonic']>1:raise ValueError('Memory unavailable')
                info=json.loads(record['value'])
                if not info.get('ready') or info['frame']!=value['frame'] or info['epoch']!=value['epoch'] or memory_goal_pub is None:
                    raise ValueError('Memory epoch not ready')
                if not local or time.monotonic()-local['received_monotonic']>1 or local['value']['frame']!=value['frame'] or math.dist(local['value']['position'][:2],(value['x'],value['y']))>100:
                    raise ValueError('LIO unavailable or candidate goal exceeds 100m')
                request=dict(value,request_time=time.time());memory_goal_pub.publish(String(data=json.dumps(request)))
                with lock:operator_settings['goal_mode']='lio'
                body=json.dumps({'queued':True,'candidate_only':True,'executed':False}).encode()
                self.send_response(202);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
                self.end_headers();self.wfile.write(body);return
            if is_control:
                if self.path=='/api/control/refresh':
                    if value!={}:raise ValueError('Unexpected fields')
                    control.refresh_requested=True
                    body=b'{"queued":true,"executed":false}'
                    self.send_response(202);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
                if not control.status().get('enabled'):
                    raise ValueError('Hardware control configuration is disabled')
                kind = self.path.rsplit('/',1)[-1]
                if kind not in ('start','stop','heartbeat','navigate','lio_navigate','lower','pose','loco','speed_limits'):
                    raise ValueError('invalid control request')
                if kind=='speed_limits':
                    from navigation_speed import validate_limits
                    validate_limits(value)
                elif kind=='pose':
                    from robot_policy import posture
                    posture(value,allow_height=True)
                elif kind in ('lower','loco'):
                    if value!={'confirm_stationary':True}:raise ValueError('Confirm physical standstill first')
                elif value!={}:raise ValueError('Unexpected fields')
                if kind=='lio_navigate':
                    with lock:record=samples.get('memory')
                    if not record or time.monotonic()-record['received_monotonic']>1:raise ValueError('Local memory unavailable')
                    m=json.loads(record['value'])
                    if not m.get('ready') or not m.get('goal') or not m.get('route') or m['route'].get('partial') is not False:
                        raise ValueError('Select a LIO goal with a complete path; frontier paths cannot execute')
                    target=dict(epoch=m['epoch'],goal=m['goal'])
                elif kind == 'navigate':
                    with lock:
                        target = operator_settings.get('latitude')
                        if operator_settings.get('goal_mode')=='lio':raise ValueError('LIO target selected; use Start LIO navigation, not OSM navigation')
                    if not target: raise ValueError('Select target first')
                else: target = value
                try: result = control.enqueue(kind, target)
                except queue.Full: raise ValueError('Control request queue full')
                body = json.dumps(result).encode()
                self.send_response(202)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers(); self.wfile.write(body); return
            keys = ('latitude', 'longitude') if self.path.endswith('goal') else ('heading_deg',)
            if set(value) != set(keys) or any(type(value[k]) not in (int, float) or not math.isfinite(value[k]) for k in keys):
                raise ValueError('invalid fields')
            if 'latitude' in value and not (-90 <= value['latitude'] <= 90 and -180 <= value['longitude'] <= 180):
                raise ValueError('invalid coordinates')
            if 'heading_deg' in value and not 0 <= value['heading_deg'] < 360:
                raise ValueError('heading must be [0,360) degrees clockwise from north')
            with lock:
                operator_settings[keys[0]] = dict(value)
                if self.path.endswith('goal'):operator_settings['goal_mode']='geographic'
            body = json.dumps({'saved': True, 'executed': False, 'value': value,
                               'reason': 'Preview only: localization/navigation adapters not connected'}).encode()
        except (ValueError, TypeError, KeyError):
            self.send_error(400)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/api/state':
            with lock:
                now = time.monotonic()
                payload = {key: {'receive_age_sec': now - val['received_monotonic'],
                                 'value': val['value']} for key, val in samples.items()}
                settings = dict(operator_settings)
            body = json.dumps({'read_only': not control.status()['enabled'], 'control': control.status(), 'samples': payload,
                               'operator_preview': settings}, allow_nan=False).encode()
            mime = 'application/json'
        elif self.path == '/':
            body = (ROOT / 'web/index.html').read_bytes()
            mime = 'text/html; charset=utf-8'
        elif self.path == '/api/osm':
            body = osm_data
            mime = 'application/geo+json'
        elif self.path in ('/map.js','/geo_alignment.js','/control.js','/viewport.js','/dashboard.js','/dashboard.css','/drawer.css'):
            body = (ROOT / 'web' / self.path[1:]).read_bytes()
            mime = 'text/css; charset=utf-8' if self.path.endswith('.css') else 'text/javascript; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


def main():
    global osm_data, control, memory_goal_pub, memory_reset_pub, registration_pub
    osm_data = load_osm()
    rclpy.init()
    node = rclpy.create_node('navigation_web_readonly')
    from navigation_control import Control
    class ResilientControl:
        def __init__(self):
            self.inner=None;self.error='Waiting for robot connection';self.refresh_requested=True;self.last_attempt=-100.
        def status(self):
            return self.inner.status() if self.inner else dict(enabled=False,phase='OFFLINE',reason=self.error,robot={},robot_receive_age_sec=None)
        def enqueue(self,*args):
            if not self.inner:raise ValueError(self.error)
            return self.inner.enqueue(*args)
        def close(self):
            if self.inner:self.inner.close()
        def poll(self):
            now=time.monotonic()
            if self.inner:
                if self.refresh_requested:
                    self.inner.stop('Operator connection refresh; no automatic motion resume')
                    self.inner.send('hello');self.refresh_requested=False
                return
            if not self.refresh_requested and now-self.last_attempt<5:return
            self.last_attempt=now;self.refresh_requested=False
            try:
                candidate=Control(node)
                if candidate.sock:self.inner=candidate
                else:self.error=candidate.reason
            except (OSError,ValueError,KeyError,TypeError) as exc:self.error='Robot control offline: '+str(exc)
    control = ResilientControl()
    reconnect_timer=node.create_timer(1.,control.poll)
    memory_goal_pub=node.create_publisher(String,'/navigation/memory/goal_candidate',1)
    memory_reset_pub=node.create_publisher(String,'/navigation/memory/reset',1)
    registration_pub=node.create_publisher(String,'/navigation/reference/register',1)
    subs = [node.create_subscription(NavSatFix, '/hardware/gnss/fix', fix, qos_profile_sensor_data),
            node.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',cloud,qos_profile_sensor_data),
            node.create_subscription(Odometry, '/navigation/lio/odometry', odom, qos_profile_sensor_data)]
    for topic, key in [('/localization/gnss/status', 'gps_status'),
                       ('/navigation/lio/status', 'lio_status'),
                       ('/navigation/cloud_filter/status','cloud_status'),('/navigation/memory/status','memory'),
                       ('/navigation/memory/display','memory_display'),
                       ('/real_navigation/safety_status','reference'),('/navigation/local_execution/status','local_execution')]:
        subs.append(node.create_subscription(String, topic, lambda m, k=key: save(k, m.data), 10))
    subs.append(node.create_subscription(RosPath, '/plan', lambda m: save('plan', {
        'frame': m.header.frame_id, 'points': [[p.pose.position.x, p.pose.position.y]
        for p in m.poses[::max(1, len(m.poses)//1000)]]}), 1))
    subs.append(node.create_subscription(RosPath,'/navigation/display_plan',lambda m:save('plan',{
        'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses[::max(1,len(m.poses)//1000)]]}),1))
    server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    if os.environ.get('NXK_LOG_LEVEL', 'error').lower() in ('debug','info'):
        print('Dashboard: http://127.0.0.1:8765 (control disabled by default)', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        control.close()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
