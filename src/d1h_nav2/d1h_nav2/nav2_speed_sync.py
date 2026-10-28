"""Synchronize requested web limits into Nav2, with service ack + readback."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically,GetParameters
from std_msgs.msg import String
from d1h_nav2.local_navigation import speed_limits


def targets(limits):
    v,w=float(limits['linear_mps']),float(limits['angular_rps'])
    return {'controller_server':{'FollowPath.desired_linear_vel':v,'FollowPath.rotate_to_heading_angular_vel':w},
            'velocity_smoother':{'max_velocity':[v,0.,w],'min_velocity':[0.,0.,-w]}}


class Nav2SpeedSync(Node):
    def __init__(self):
        super().__init__('nav2_speed_sync')
        self.desired=dict(linear_mps=.15,angular_rps=.35);self.applied=None
        self.state='WAIT';self.reason='Waiting for Nav2 parameter services'
        self.pending=None;self.retry=0.;self.verify_at=0.
        prefix=self.get_namespace().rstrip('/')
        self.setters={n:self.create_client(SetParametersAtomically,f'{prefix}/{n}/set_parameters_atomically') for n in targets(self.desired)}
        self.getters={n:self.create_client(GetParameters,f'{prefix}/{n}/get_parameters') for n in targets(self.desired)}
        self.declare_parameter('status_topic','/real_navigation/speed_status')
        self.pub=self.create_publisher(String,self.get_parameter('status_topic').value,1)
        self.sub=self.create_subscription(String,'/navigation/speed_limits',self.request,1)
        self.timer=self.create_timer(.1,self.tick)

    def request(self,msg):
        try:
            value=speed_limits(json.loads(msg.data))
            if value!=self.desired:
                self.desired=value;self.applied=None
                self.state='APPLYING';self.reason='Applying requested Nav2 speed limits'
                # Complete old RPCs before writing the newer request, preventing
                # an old inflight set from overwriting a new acknowledged limit.
                self.retry=0.
        except (ValueError,TypeError):pass

    def begin(self,stage,limits):
        jobs={};spec=targets(limits)
        for name,params in spec.items():
            if stage=='set':
                req=SetParametersAtomically.Request()
                req.parameters=[Parameter(k,value=float(v) if isinstance(v,(int,float)) else v).to_parameter_msg() for k,v in params.items()]
                jobs[name]=self.setters[name].call_async(req)
            else:
                req=GetParameters.Request();req.names=list(params)
                jobs[name]=self.getters[name].call_async(req)
        self.pending=(stage,dict(limits),jobs,time.monotonic())

    def fail(self,reason):
        if self.pending:
            for f in self.pending[2].values():
                if not f.done():f.cancel()
        self.pending=None;self.applied=None;self.state='ERROR';self.reason=reason;self.retry=time.monotonic()+1.

    def advance(self):
        now=time.monotonic()
        if self.pending:
            stage,limits,jobs,started=self.pending
            if now-started>2:
                self.fail('Nav2 speed parameter RPC timed out');return
            if not all(f.done() for f in jobs.values()):return
            try:
                for name,f in jobs.items():
                    result=f.result()
                    if stage=='set':
                        if not result.result.successful:raise ValueError(name+': '+result.result.reason)
                    else:
                        expected=list(targets(limits)[name].values())
                        if len(result.values)!=len(expected):raise ValueError(name+': incomplete parameter readback')
                        for actual,value in zip(result.values,expected):
                            got=list(actual.double_array_value) if isinstance(value,list) else actual.double_value
                            if actual.type!=(8 if isinstance(value,list) else 3) or got!=value:
                                raise ValueError(name+': parameter readback differs')
            except Exception as exc:self.fail('Nav2 speed update failed: '+str(exc));return
            self.pending=None
            if limits!=self.desired:return  # Newer request must be applied next.
            if stage=='set':self.begin('get',limits)
            else:
                self.applied=limits;self.state='READY';self.reason='Nav2 controller + smoother speed limits verified';self.verify_at=now+1.
            return
        if now<self.retry:return
        if not all(c.service_is_ready() for c in [*self.setters.values(),*self.getters.values()]):
            self.applied=None;self.state='WAIT';self.reason='Waiting for Nav2 speed parameter services';return
        if self.applied!=self.desired:self.begin('set',self.desired)
        elif now>=self.verify_at:self.begin('get',self.desired)

    def tick(self):
        self.advance()
        self.pub.publish(String(data=json.dumps(dict(stamp=self.get_clock().now().nanoseconds/1e9,
            ready=self.applied==self.desired,state=self.state,reason=self.reason,
            requested=self.desired,applied=self.applied))))


def main():
    rclpy.init();node=Nav2SpeedSync()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
