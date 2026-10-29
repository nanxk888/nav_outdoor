"""GPS-free rolling Smac + RPP orchestration; velocity only via coordinator."""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose,FollowPath
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from d1h_nav2.local_navigation_node import LocalNavigation
from d1h_nav2.local_navigation import speed_limits
from d1h_nav2.navigation_reference import body_heading
from d1h_nav2.footprint import path_free,body_path_free
from d1h_nav2.arrival import Arrival,approach_cap
from d1h_nav2.heading_align import HeadingAlign


class LioNav2Executor(LocalNavigation):
    def __init__(self):
        super().__init__()
        self.tf=TransformBroadcaster(self);self.odom_pub=self.create_publisher(Odometry,'/lio_navigation/odometry',5)
        self.planner=ActionClient(self,ComputePathToPose,'/lio_nav/compute_path_to_pose')
        self.follower=ActionClient(self,FollowPath,'/lio_nav/follow_path')
        self.backend={};self.backend_rx=-math.inf;self.plan_points=[];self.reference=[]
        self.plan_busy=False;self.follow_active=False;self.follow_handle=None;self.planner_handle=None
        self.follow_revision=0;self.next_plan=0.;self.cmd=(0.,0.);self.cmd_rx=-math.inf;self.blocked_since=None
        self.speed_mismatch_since=None
        self.plan_error=''
        self.arrival=Arrival();self.arrival_stage='TRACKING'
        self.aligner=HeadingAlign();self.motion_phase='WAIT_PATH'
        self.subs.extend([self.create_subscription(String,'/lio_navigation/speed_status',self.backend_status,1),
                          self.create_subscription(String,'/navigation/speed_limits',self.limits_input,1),
                          self.create_subscription(Twist,'/lio_navigation/cmd_vel',self.command,1)])

    def limits_input(self,msg):
        try:self.speed_limits=speed_limits(json.loads(msg.data))
        except (ValueError,TypeError):pass

    def backend_status(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Invalid speed status')
            self.backend=value;self.backend_rx=time.monotonic()
        except ValueError:self.backend={}

    def cache(self,key,msg):
        super().cache(key,msg)
        if key!='odom':return
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        try:yaw=body_heading((q.x,q.y,q.z,q.w))
        except ValueError:return
        t=TransformStamped();t.header=msg.header;t.child_frame_id='lio_navigation_base'
        t.transform.translation.x=p.x;t.transform.translation.y=p.y
        t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
        self.tf.sendTransform(t)
        out=copy.deepcopy(msg);out.child_frame_id='lio_navigation_base';out.pose.pose.position.z=0.
        out.pose.pose.orientation.x=out.pose.pose.orientation.y=0.
        out.pose.pose.orientation.z=math.sin(yaw/2);out.pose.pose.orientation.w=math.cos(yaw/2)
        x,y,z,w=q.x,q.y,q.z,q.w;c,s=math.cos(yaw),math.sin(yaw)
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        B=np.array([[c,s,0],[-s,c,0],[0,0,1]])@R
        for vector in (out.twist.twist.linear,out.twist.twist.angular):vector.x,vector.y,vector.z=map(float,B@np.array([vector.x,vector.y,vector.z]))
        T=np.zeros((6,6));T[:3,:3]=B;T[3:,3:]=B
        out.twist.covariance=(T@np.asarray(msg.twist.covariance).reshape(6,6)@T.T).reshape(-1).tolist()
        self.odom_pub.publish(out)

    def inputs(self,validate_tracking=False):
        value=super().inputs(False)
        if not self.planner.server_is_ready() or not self.follower.server_is_ready():raise ValueError('LIO Smac/RPP backend not active yet')
        matched=time.monotonic()-self.backend_rx<.7 and self.backend.get('ready') and self.backend.get('applied')==self.speed_limits
        if matched:self.speed_mismatch_since=None
        else:
            if self.speed_mismatch_since is None:self.speed_mismatch_since=time.monotonic()
            if self.state!='ACTIVE' or time.monotonic()-self.backend_rx>.7 or self.backend.get('state') in ('ERROR','WAIT') or time.monotonic()-self.speed_mismatch_since>3:
                raise ValueError('LIO Nav2 speed parameters not ready: '+str(self.backend.get('reason','waiting')))
        return value

    def command(self,msg):
        if self.state!='ACTIVE' or not self.follow_active or self.arrival.since is not None:return
        v,w=msg.linear.x,msg.angular.z
        if any(not math.isfinite(x) for x in (v,w)) or v<-.00001 or v>1.00001 or abs(w)>1.50001:
            self.state='STOPPED';self.reason='Invalid LIO Nav2 velocity';return
        scale=min(1.,self.speed_limits['linear_mps']/max(v,1e-9),self.speed_limits['angular_rps']/max(abs(w),1e-9))
        self.cmd=(max(0.,v)*scale,w*scale);self.cmd_rx=time.monotonic()

    def cancel_backend(self):
        self.follow_revision+=1;self.follow_active=False;self.plan_busy=False;self.cmd=(0.,0.);self.cmd_rx=-math.inf
        for h in (self.follow_handle,self.planner_handle):
            if h is not None:h.cancel_goal_async()
        self.follow_handle=self.planner_handle=None

    def request(self,msg):
        old_state=self.state;old_run=self.run_id
        super().request(msg)
        if self.state=='ACTIVE' and (old_state!='ACTIVE' or self.run_id!=old_run):
            self.arrival=Arrival();self.arrival_stage='TRACKING'
            self.aligner=HeadingAlign();self.motion_phase='WAIT_PATH'
            self.reference=list(self.path);self.plan_points=[];self.plan_error='';self.next_plan=0.;self.blocked_since=time.monotonic()
            self.reason='Computing rolling Smac path; RPP waiting'
        elif old_state=='ACTIVE' and self.state!='ACTIVE':self.cancel_backend()

    def free(self,cell):
        i,j=cell;g=self.grid
        return 0<=i<g.info.width and 0<=j<g.info.height and g.data[j*g.info.width+i]==0

    def valid(self,points):
        g=self.grid;r=g.info.resolution;ox,oy=g.info.origin.position.x,g.info.origin.position.y
        return body_path_free([(x-ox,y-oy) for x,y in points],r,self.free)

    def plan(self,position):
        self.plan_busy=True;self.next_plan=time.monotonic()+.2;run=self.run_id
        route=self.memory.get('route')
        if route and route.get('partial') is False:self.reference=route['points']
        nearest=min(range(len(self.reference)),key=lambda i:math.dist(position,self.reference[i]))
        length=0.;local_goal=self.target
        for a,b in zip([position]+self.reference[nearest:],self.reference[nearest:]):
            d=math.dist(a,b)
            if length+d>=5.:
                t=(5.-length)/max(d,1e-9);local_goal=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])];break
            length+=d
        goal=ComputePathToPose.Goal();goal.planner_id='LocalGrid';goal.goal=PoseStamped()
        goal.goal.header.frame_id='lio_odom';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(local_goal[0]);goal.goal.pose.position.y=float(local_goal[1]);goal.goal.pose.orientation.w=1.
        def planned(f):
            if self.run_id!=run or self.state!='ACTIVE' or self.arrival.since is not None:return
            self.plan_busy=False
            if self.aligner.active:return  # Keep the accepted turn path stable.
            try:
                result=f.result()
                if result.status!=4 or not result.result.path.poses:raise ValueError('Smac found no current local path')
                path=result.result.path;points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                if len(points)>4000 or not all(math.isfinite(v) for p in points for v in p):
                    raise ValueError('Smac path exceeds local geometry budget')
                if path.header.frame_id!='lio_odom':raise ValueError('Smac path frame mismatch')
                self.plan_error='';self.plan_points=points;self.follow_revision+=1;revision=self.follow_revision
                request=FollowPath.Goal();request.path=path;request.controller_id='FollowPath';request.goal_checker_id='goal_checker'
                def accepted(done):
                    try:h=done.result()
                    except Exception as exc:self.reason='RPP action response failed: '+str(exc);return
                    if run!=self.run_id or self.state!='ACTIVE' or revision!=self.follow_revision or self.arrival.since is not None:
                        if h.accepted:h.cancel_goal_async()
                        return
                    if not h.accepted:self.reason='RPP rejected local path';return
                    if not self.follow_active:self.cmd_rx=-math.inf
                    self.follow_handle=h;self.follow_active=True
                    def finished(result_future):
                        if run==self.run_id and revision==self.follow_revision:
                            self.follow_active=False;self.next_plan=0.;self.aligner.active=False
                            try:
                                if result_future.result().status!=4:self.plan_error='RPP跟踪结束，重新规划'
                            except Exception:self.plan_error='RPP结果不可用，重新规划'
                    h.get_result_async().add_done_callback(finished)
                self.follower.send_goal_async(request).add_done_callback(accepted)
            except Exception as exc:self.plan_error=str(exc);self.reason=str(exc)
        def planner_accepted(f):
            try:h=f.result()
            except Exception as exc:self.plan_busy=False;self.reason='Smac action response failed: '+str(exc);return
            if self.run_id!=run or self.state!='ACTIVE' or self.arrival.since is not None:
                if h.accepted:h.cancel_goal_async()
                return
            if not h.accepted:self.plan_busy=False;self.reason='Smac rejected request';return
            self.planner_handle=h;h.get_result_async().add_done_callback(planned)
        self.planner.send_goal_async(goal).add_done_callback(planner_accepted)

    def tick(self):
        ready=False;v=w=0.;was_active=self.state=='ACTIVE'
        terminal=self.reason if self.state in ('SUCCEEDED','STOPPED','REJECTED') else None
        input_reason=None
        try:
            _,_,done,position=self.inputs();ready=True;now=time.monotonic()
            if self.state in ('IDLE','READY'):self.state='READY';self.reason='Smac/RPP ready; explicit full-path start required'
            if self.state=='ACTIVE':
                if now-self.lease_rx>.3:raise ValueError('Local execution session lease expired')
                q=self.odom.pose.pose.orientation;yaw=body_heading((q.x,q.y,q.z,q.w))
                distance=math.dist(position,self.target)
                stamp=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec/1e9
                previous_stage=self.arrival_stage
                self.arrival_stage=self.arrival.update(position,yaw,stamp,distance,now)
                if self.arrival_stage!='TRACKING':
                    self.motion_phase='ARRIVAL_BRAKING'
                    if previous_stage=='TRACKING':self.cancel_backend()
                    self.reason='到站停车中：持续零速，等待LIO静止确认'
                    if self.arrival_stage=='SETTLED':
                        self.state='SUCCEEDED';self.reason='到达目标：零速保持，LIO静止已确认（非本体速度验证）'
                    elif self.arrival_stage in ('OVERSHOOT','TIMEOUT'):
                        raise ValueError('到站停车异常：越出0.25米范围或5秒未确认静止，请人工检查')
                else:
                    aligning=self.aligner.update(position,yaw,self.plan_points) if self.follow_active else False
                    if not self.follow_active:self.aligner.active=False
                    self.motion_phase='ALIGN_HEADING' if aligning else 'FOLLOW_PATH'
                    if not aligning and not self.plan_busy and now>=self.next_plan:self.plan(position)
                    points=[]
                    if self.plan_points:
                        k=min(range(len(self.plan_points)),key=lambda i:math.dist(position,self.plan_points[i]))
                        points=[position]+self.plan_points[k:]
                    usable=bool(points) and self.follow_active and now-self.cmd_rx<.3
                    if usable:
                        v,w=self.cmd
                        scale=min(1.,approach_cap(distance)/max(v,1e-9));v*=scale;w*=scale
                        self.blocked_since=None;self.reason='接近目标：减速中' if distance<.8 else '正在导航：Smac规划／RPP跟踪'
                        if aligning:
                            v=0.;self.reason='原地对准路径：误差%.1f°，禁止前进' % math.degrees(self.aligner.error)
                    else:
                        v=w=0.;self.reason='Holding zero; '+(self.plan_error or 'waiting for valid Smac path/RPP velocity')
                        if self.blocked_since is None:self.blocked_since=now
                        self.reason='等待通路／重新规划：'+(self.plan_error or '等待RPP速度')
                        self.motion_phase='ALIGN_WAIT' if aligning else 'WAIT_PATH'
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            input_reason=str(exc)
            self.reason=str(exc)
            if self.state=='ACTIVE':self.state='STOPPED'
            elif self.state=='READY':self.state='IDLE'
            if was_active:self.get_logger().error('导航终止：'+self.reason)
        if terminal is not None:self.reason=terminal
        if self.state!='ACTIVE' and (was_active or self.follow_active or self.plan_busy or self.follow_handle or self.planner_handle):self.cancel_backend()
        self.pub.publish(String(data=json.dumps(dict(stamp=self.get_clock().now().nanoseconds/1e9,ready=ready,
            state=self.state,reason=self.reason,input_reason=input_reason,run_id=self.run_id,epoch=self.memory.get('epoch'),goal=self.memory.get('goal'),
            v=v,w=w,speed_limits=self.speed_limits,gps_required=False,controller='Nav2 RPP',planner='SmacPlanner2D',
            plan=self.plan_points if self.state=='ACTIVE' else [],main_path=self.reference,
            arrival_stage=self.arrival_stage,motion_phase=self.motion_phase,heading_error_deg=math.degrees(self.aligner.error),
            memory_diagnostics=self.memory_diagnostics,
            goal_distance_m=math.dist(position,self.target) if self.target and 'position' in locals() else None))))


def main():
    rclpy.init();node=LioNav2Executor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.cancel_backend();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
