"""One-shot startup observations, not a navigation safety authorization."""
import argparse
import json
import time
import urllib.request


def main():
    import rclpy
    from std_msgs.msg import String
    from sensor_msgs.msg import NavSatFix, PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    from lifecycle_msgs.srv import GetState

    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checks',default='lio,gps,gate,cloud,web')
    parser.add_argument('--timeout',type=float,default=45.)
    args=parser.parse_args()
    pending=set(filter(None,args.checks.split(',')))
    rclpy.init();node=rclpy.create_node('workspace_startup_check')
    subs=[];cloud_seen=set();nav_states={};futures={}
    def done(name,detail):
        if name in pending:
            print('[READY] '+name+': '+detail,flush=True);pending.remove(name)
    def state(msg):
        try:return json.loads(msg.data)
        except (ValueError,TypeError):return {}
    def lio(msg):
        s=state(msg)
        if s.get('state')=='LIO_SENSOR_TRACKING':done('lio','sensor tracking (not base_link localization)')
    def gps(msg):done('gps','Domain 0 received GPS; fix status='+str(msg.status.status)+' (-1 means no fix)')
    def gate(msg):done('gate','gate running: '+str(state(msg).get('state','unknown')))
    def memory(msg):
        if state(msg).get('ready') is True:done('memory','LIO observation memory ready; candidate planning only, no motor output')
    def local_execution(msg):done('local','GPS-free local executor running; explicit full-path start required; '+str(state(msg).get('reason','waiting')))
    registered=False
    def reference(msg):
        nonlocal registered
        s=state(msg);registered=s.get('registered') is True
        done('reference','sensor-derived virtual navigation frame running; '+str(s.get('reason','waiting for registration')))
    ground={'ready':False}
    def cloud_status(msg):ground.update(state(msg))
    def cloud(msg,name):
        age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if -.05<=age<.5:cloud_seen.add(name)
        if len(cloud_seen)==2 and ground.get('ready'):done('cloud','ground accepted; both nav/clear streams received')
    for topic,callback in [('/navigation/lio/status',lio),('/localization/gnss/status',gate),
                           ('/navigation/cloud_filter/status',cloud_status),('/navigation/memory/status',memory),
                           ('/real_navigation/safety_status',reference),('/navigation/local_execution/status',local_execution)]:
        subs.append(node.create_subscription(String,topic,callback,10))
    subs.append(node.create_subscription(NavSatFix,'/hardware/gnss/fix',gps,qos_profile_sensor_data))
    for name in ('nav','clear'):
        subs.append(node.create_subscription(PointCloud2,'/livox/lidar/pointcloud_'+name,
            lambda msg,n=name:cloud(msg,n),qos_profile_sensor_data))
    clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in
             ('planner_server','controller_server','bt_navigator','local_planner/planner_server')}
    deadline=time.monotonic()+args.timeout;next_poll=0.
    try:
        while pending and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()<next_poll:continue
            next_poll=time.monotonic()+1.
            if 'web' in pending:
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8765/api/state',timeout=.3) as response:
                        json.load(response)
                    done('web','HTTP /api/state responding')
                except (OSError,ValueError):pass
            if 'nav' in pending:
                for name,client in clients.items():
                    future=futures.get(name)
                    if future is not None and future.done():
                        try:nav_states[name]=future.result().current_state.id
                        except Exception:nav_states[name]=0
                        futures.pop(name,None)
                    if name not in futures and client.service_is_ready():futures[name]=client.call_async(GetState.Request())
                if all(nav_states.get(name)==3 for name in clients):
                    done('nav','planner/controller/BT/local planner lifecycle ACTIVE; NOT proof of TF, safety or navigation readiness')
        for name in sorted(pending):
            if name=='nav' and not registered:
                print('[WAIT] nav: select initial heading and register fresh GPS/LIO in web; planner startup deferred, no SDK requirement.',flush=True)
                continue
            detail=('; ground='+str(ground.get('reason','no status'))) if name=='cloud' else ''
            print('[NOT READY] '+name+': startup check timed out'+detail+'; inspect configuration/data. No motion authorized.',flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
