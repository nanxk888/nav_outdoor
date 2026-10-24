"""Loopback-only, read-only ROS2 dashboard. No actuator or goal publishers."""
import json
import math
import os
import queue
import xml.etree.ElementTree as ET
import threading
import time
import struct
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
last_cloud_receive=-100.


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
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    save('lio', {'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w],
                 'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
                 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9})


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        # Browser same-origin protection; loopback service is intended for SSH tunnelling.
        if self.headers.get('Origin') != 'http://' + self.headers.get('Host', ''):
            self.send_error(403)
            return
        is_control = self.path.startswith('/api/control/')
        if self.path not in ('/api/operator/goal', '/api/operator/heading') and not is_control:
            self.send_error(404)
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024:
                raise ValueError('invalid body length')
            value = json.loads(self.rfile.read(size))
            if is_control:
                if not control.status().get('enabled'):
                    raise ValueError('Hardware control configuration is disabled')
                kind = self.path.rsplit('/',1)[-1]
                if kind not in ('start','stop','heartbeat','navigate','lower','pose','loco'):
                    raise ValueError('invalid control request')
                if kind=='pose':
                    from robot_policy import posture
                    posture(value,allow_height=True)
                elif kind in ('lower','loco'):
                    if value!={'confirm_stationary':True}:raise ValueError('Confirm physical standstill first')
                elif value!={}:raise ValueError('Unexpected fields')
                if kind == 'navigate':
                    with lock: target = operator_settings.get('latitude')
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
        elif self.path in ('/map.js','/control.js','/viewport.js','/dashboard.js','/dashboard.css'):
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
    global osm_data, control
    osm_data = load_osm()
    rclpy.init()
    node = rclpy.create_node('navigation_web_readonly')
    from navigation_control import Control
    control = Control(node)
    subs = [node.create_subscription(NavSatFix, '/hardware/gnss/fix', fix, qos_profile_sensor_data),
            node.create_subscription(PointCloud2,'/livox/lidar/pointcloud_nav',cloud,qos_profile_sensor_data),
            node.create_subscription(Odometry, '/navigation/lio/odometry', odom, qos_profile_sensor_data)]
    for topic, key in [('/localization/gnss/status', 'gps_status'),
                       ('/navigation/lio/status', 'lio_status'),
                       ('/navigation/cloud_filter/status','cloud_status')]:
        subs.append(node.create_subscription(String, topic, lambda m, k=key: save(k, m.data), 10))
    subs.append(node.create_subscription(RosPath, '/plan', lambda m: save('plan', {
        'frame': m.header.frame_id, 'points': [[p.pose.position.x, p.pose.position.y]
        for p in m.poses[::max(1, len(m.poses)//1000)]]}), 1))
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
