"""Cache safe paths; separate safety checks from expensive replanning."""
import copy
import math
import time
import threading
from functools import partial
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from d1h_nav2.path_buffer import prune_and_sample, trim_end, length
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


def remaining_length(path, x, y):
    if not path.poses:
        return 0., math.inf
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    index = min(range(len(points)), key=lambda i: math.dist(points[i], (x, y)))
    deviation = math.dist(points[index], (x, y))
    return sum(math.dist(a, b) for a, b in zip(points[index:], points[index+1:])), deviation


class StablePlanner(Node):
    def __init__(self):
        super().__init__('stable_planner')
        self.declare_parameter('robot_frame','base_link')
        self.robot_frame=self.get_parameter('robot_frame').value
        for name, default in dict(global_period_sec=1., local_period_sec=.2,
                                  local_refresh_remaining_m=3.5,
                                  local_goal_shift_m=.03, max_path_deviation_m=.5,
                                  validation_timeout_sec=.3, handover_timeout_sec=6.,
                                  braking_decel_mps2=.5, reaction_time_sec=.4,
                                  stop_margin_m=.25).items():
            self.declare_parameter(name, default)
        self.group = ReentrantCallbackGroup()
        self._scheduler_group = MutuallyExclusiveCallbackGroup()
        self._scheduler_lock = threading.Lock()
        self._waiters, self._cancellations = {}, {}
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Never create/destroy timers during an action. Humble's multi-thread
        # executor can still have a just-destroyed timer in its ready queue.
        self._scheduler_timer = self.create_timer(
            .01, self._poll_pending, callback_group=self._scheduler_group,
            clock=self._steady_clock)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cache = {'global': None, 'local': None}
        self.last_check = {'global': -math.inf, 'local': -math.inf}
        self.count = {'global': 0, 'local': 0}
        self.generation = {'global': 0, 'local': 0}
        self._local_source = None
        self._handover_since = None
        self._handover_anchor = None
        self._prefix_active, self._global_pending = False, False
        self._speed = .5
        self._odom_sub = self.create_subscription(Odometry, '/odometry/local', self._on_odom, 5)
        self._planner_clients, self._validators, self._servers = {}, {}, []
        for kind, prefix in (('global', ''), ('local', '/local_planner')):
            self._planner_clients[kind] = ActionClient(self, ComputePathToPose,
                prefix+'/compute_path_to_pose', callback_group=self.group)
            self._validators[kind] = self.create_client(IsPathValid, prefix+'/is_path_valid',
                                                       callback_group=self.group)
            self._servers.append(ActionServer(self, ComputePathToPose,
                '/navigation/compute_'+kind+'_path',
                execute_callback=partial(self.compute, kind=kind),
                cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group))
        self.status = self.create_publisher(String, '/navigation/replanning_status', 1)
        self.display = self.create_publisher(Path, '/navigation/display_plan', 1)
        self.display_timer = self.create_timer(.2, self.publish_display)
        self.timer = self.create_timer(1., lambda: self.status.publish(String(
            data=(f'actual_plans global={self.count["global"]} local={self.count["local"]} '
                  f'safe_prefix={self._prefix_active} global_pending={self._global_pending}'))))

    def p(self, name):
        return float(self.get_parameter(name).value)

    def _on_odom(self, msg):
        self._speed = max(.5, math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))

    async def safe_prefix(self):
        """Revalidate old geometry against CURRENT lidar map, including segments."""
        if self._local_source is None:
            return None
        try:
            tf = self.buffer.lookup_transform(self._local_source.header.frame_id, self.robot_frame, Time())
        except TransformException:
            return None
        pos = tf.transform.translation
        path, deviation = prune_and_sample(self._local_source, pos.x, pos.y)
        if deviation > self.p('max_path_deviation_m'):
            return None  # No invented connector from robot to a distant old line.
        margin = (self._speed**2/(2*self.p('braking_decel_mps2')) +
                  self._speed*self.p('reaction_time_sec') + self.p('stop_margin_m'))
        valid = await self.validate('local', path)
        if not valid:
            # Find the first unsafe part, not just a valid last waypoint. All
            # requests contain densely sampled prefixes and footprint costs.
            lo, hi = 0, len(path.poses)
            while lo < hi:
                mid = (lo+hi+1)//2
                candidate = copy.deepcopy(path)
                candidate.poses = candidate.poses[:mid]
                if await self.validate('local', candidate):
                    lo = mid
                else:
                    hi = mid-1
            path.poses = path.poses[:lo]
            path = trim_end(path, margin)
        if path.poses and length(path) >= margin and await self.validate('local', path):
            return path
        # No usable forward prefix: hold at the current verified-free pose,
        # never keep moving merely to avoid a blank visualization.
        hold = Path()
        hold.header = copy.deepcopy(self._local_source.header)
        point = PoseStamped()
        point.header = copy.deepcopy(hold.header)
        point.pose.position.x, point.pose.position.y = pos.x, pos.y
        point.pose.orientation = copy.deepcopy(tf.transform.rotation)
        hold.poses = [point]
        return hold if await self.validate('local', hold) else None

    async def fallback(self, kind, goal, cached):
        """Front buffer stays executable while the next request retries back buffer."""
        now = self.get_clock().now().nanoseconds/1e9
        try:
            p = self.buffer.lookup_transform('map', self.robot_frame, Time()).transform.translation
            xy = (p.x, p.y)
            if self._handover_anchor is None or math.dist(xy, self._handover_anchor) >= .2:
                self._handover_anchor, self._handover_since = xy, now
        except TransformException:
            return None
        if self._handover_since is None:
            self._handover_since = now
        if now-self._handover_since > self.p('handover_timeout_sec'):
            return None
        prefix = await self.safe_prefix()
        if prefix is None:
            return None
        if kind == 'global':
            # A stale global line is ONLY a reference for choosing a new local
            # goal, never an executable path. Refuse to reuse a different goal.
            if cached is None or self.goal_key(goal) != self.goal_key(cached['goal']):
                return None
            return copy.deepcopy(cached['result'])
        result = ComputePathToPose.Result()
        result.path = prefix
        return result

    async def wait_ready(self, ready, timeout):
        result, deadline = Future(executor=self.executor), time.monotonic()+timeout
        with self._scheduler_lock:
            self._waiters[result] = (ready, deadline)
        try:
            return await result
        finally:
            with self._scheduler_lock:
                self._waiters.pop(result, None)

    def _poll_pending(self):
        with self._scheduler_lock:
            waiters = list(self._waiters.items())
            cancellations = list(self._cancellations.items())
        now = time.monotonic()
        for future, (ready, deadline) in waiters:
            if future.done():
                continue
            try:
                available = ready()
                if available or now >= deadline:
                    future.set_result(available)
            except Exception as exc:
                future.set_exception(exc)
        for _, state in cancellations:
            handle = state['handle']
            if not state['sent'] and handle.is_cancel_requested:
                state['sent'] = True
                state['backend'].cancel_goal_async()

    async def validate(self, kind, path):
        client = self._validators[kind]
        if not client.service_is_ready():
            return False
        request = IsPathValid.Request()
        request.path = path
        future = client.call_async(request)
        complete = await self.wait_ready(future.done, self.p('validation_timeout_sec'))
        if not complete:
            client.remove_pending_request(future)
            return False  # Never assume a path is safe after a timeout.
        response = future.result()
        return response is not None and response.is_valid

    @staticmethod
    def goal_key(goal):
        p, q = goal.goal.pose.position, goal.goal.pose.orientation
        key = (goal.goal.header.frame_id, p.x, p.y, q.x, q.y, q.z, q.w, goal.planner_id, goal.use_start)
        if goal.use_start:
            key += (str(goal.start.pose),)
        return key

    def local_needs_refresh(self, cached, goal):
        old, new = cached['goal'].goal.pose.position, goal.goal.pose.position
        if goal.goal.header.frame_id != cached['goal'].goal.header.frame_id:
            return True
        if math.hypot(old.x-new.x, old.y-new.y) > self.p('local_goal_shift_m'):
            return True
        def yaw(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        angle = yaw(cached['goal'].goal.pose.orientation)-yaw(goal.goal.pose.orientation)
        if abs(math.atan2(math.sin(angle), math.cos(angle))) > .35:
            return True
        try:
            tf = self.buffer.lookup_transform(cached['result'].path.header.frame_id, self.robot_frame, Time())
        except TransformException:
            return True
        pos = tf.transform.translation
        remaining, deviation = remaining_length(cached['result'].path, pos.x, pos.y)
        # Do not keep a five-metre segment until its endpoint. But when this
        # IS the final target, keep its valid path instead of replanning forever.
        moving_target = math.hypot(old.x-new.x, old.y-new.y) > .25
        return (deviation > self.p('max_path_deviation_m') or
                (moving_target and remaining <= self.p('local_refresh_remaining_m')))

    def publish_display(self):
        # Visualization only: one continuous line, local path first, then the
        # global remainder. Never draw the near-field global shortcut over it.
        local, global_ = self.cache['local'], self.cache['global']
        if local is None:
            empty = Path()
            empty.header.frame_id, empty.header.stamp = 'map', self.get_clock().now().to_msg()
            self.display.publish(empty)
            return
        path = copy.deepcopy(local['result'].path)
        if not path.poses:
            return
        try:
            p = self.buffer.lookup_transform(path.header.frame_id, self.robot_frame, Time()).transform.translation
            i = min(range(len(path.poses)), key=lambda i: math.hypot(
                path.poses[i].pose.position.x-p.x, path.poses[i].pose.position.y-p.y))
            path.poses = path.poses[i:]
        except TransformException:
            return
        if (not self._prefix_active and not self._global_pending and global_ is not None
                and global_['result'].path.header.frame_id == path.header.frame_id):
            tail = global_['result'].path.poses
            end = path.poses[-1].pose.position
            if tail:
                i = min(range(len(tail)), key=lambda i: math.hypot(
                    tail[i].pose.position.x-end.x, tail[i].pose.position.y-end.y))
                # If endpoints do not meet, do not invent a straight bridge.
                if math.hypot(tail[i].pose.position.x-end.x, tail[i].pose.position.y-end.y) <= .35:
                    path.poses.extend(copy.deepcopy(tail[i+1:]))
        path.header.stamp = self.get_clock().now().to_msg()
        self.display.publish(path)

    async def compute(self, handle, kind):
        self.generation[kind] += 1
        generation = self.generation[kind]
        goal, now = handle.request, self.get_clock().now().nanoseconds/1e9
        cached = self.cache[kind]
        if now < self.last_check[kind]:
            self.cache[kind] = cached = None
            self.last_check[kind] = -math.inf
            self._local_source, self._handover_since, self._handover_anchor = None, None, None
        if handle.is_cancel_requested:
            handle.canceled()
            return ComputePathToPose.Result()
        if cached is not None:
            if (kind == 'global' and not self._global_pending and not cached.get('fallback')
                    and self.goal_key(goal) == self.goal_key(cached['goal'])
                    and now-self.last_check[kind] < self.p('global_period_sec')):
                handle.succeed()
                return copy.deepcopy(cached['result'])
            valid = await self.validate(kind, cached['result'].path)
            now = self.get_clock().now().nanoseconds/1e9
            if kind == 'global':
                reuse = valid and not cached.get('fallback') and not self._global_pending and self.goal_key(goal) == self.goal_key(cached['goal'])
                if reuse:
                    self.last_check[kind] = now
            else:
                due = now-self.last_check[kind] >= self.p('local_period_sec')
                refresh = self.local_needs_refresh(cached, goal) if due else False
                reuse = valid and not refresh and not cached.get('fallback')
                if due and reuse:
                    self.last_check[kind] = now
            if reuse:
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
                # Preserve the original header too, so FollowPath sees an
                # unchanged path rather than continually resetting its plan.
                return copy.deepcopy(cached['result'])
            if not valid:
                self.get_logger().info(f'{kind}: cached path blocked/unverified; replan ahead of schedule',
                                       throttle_duration_sec=1.)
        client = self._planner_clients[kind]
        if not await self.wait_ready(client.server_is_ready, 5.):
            handle.abort()
            return ComputePathToPose.Result()
        backend = await client.send_goal_async(goal)
        if not backend.accepted:
            handle.abort()
            return ComputePathToPose.Result()
        self.count[kind] += 1
        with self._scheduler_lock:
            self._cancellations[id(handle)] = dict(handle=handle, backend=backend, sent=False)
        try:
            response = await backend.get_result_async()
        finally:
            with self._scheduler_lock:
                self._cancellations.pop(id(handle), None)
        candidate_ok = response.status == 4 and bool(response.result.path.poses)
        if candidate_ok and kind == 'local':
            # Commit only after collision validation; do not clear front buffer
            # first, and never blend across an unchecked obstacle shortcut.
            try:
                tf = self.buffer.lookup_transform(response.result.path.header.frame_id, self.robot_frame, Time())
                p = tf.transform.translation
                sampled, deviation = prune_and_sample(response.result.path, p.x, p.y)
                candidate_ok = deviation <= self.p('max_path_deviation_m') and await self.validate('local', sampled)
                if candidate_ok:
                    response.result.path = sampled
            except TransformException:
                candidate_ok = False
        if handle.is_cancel_requested:
            handle.canceled()
        elif generation != self.generation[kind]:
            # A superseded request must not replace a newer cached route.
            handle.abort()
        elif candidate_ok:
            self.last_check[kind] = self.get_clock().now().nanoseconds/1e9
            self.cache[kind] = dict(goal=copy.deepcopy(goal), result=copy.deepcopy(response.result))
            if kind == 'local':
                self._local_source = copy.deepcopy(response.result.path)
                self._handover_since, self._prefix_active = None, False
                self._handover_anchor = None
            else:
                self._global_pending = False
            handle.succeed()
        else:
            replacement = await self.fallback(kind, goal, cached)
            if handle.is_cancel_requested:
                handle.canceled()
                return ComputePathToPose.Result()
            if generation != self.generation[kind]:
                handle.abort()
                return ComputePathToPose.Result()
            if replacement is not None:
                self._global_pending = True
                self.last_check['global'] = -math.inf
                if kind == 'local':
                    self.cache['local'] = dict(goal=copy.deepcopy(goal),
                        result=copy.deepcopy(replacement), fallback=True)
                    self._prefix_active = True
                elif cached is not None:
                    cached['fallback'] = True
                handle.succeed()
                return replacement
            self.cache[kind] = None
            if kind == 'local':
                self.last_check['global'] = -math.inf
            handle.abort()
        return response.result


def main():
    rclpy.init()
    node = StablePlanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
