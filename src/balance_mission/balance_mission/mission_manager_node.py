#!/usr/bin/env python3
"""Step 2 mission manager (known map, autonomous).

Flow
----
IDLE -> WAIT_SYSTEM -> SEARCH -> SAMPLING -> RETURN -> DONE

* WAIT_SYSTEM : publish AMCL initial pose, wait for Nav2 ACTIVE + TF + map,
                record the start pose in the map frame.
* SEARCH      : build coverage waypoints from the saved map, visit them with
                Nav2 and spin on the spot at each one while the detector looks
                for the target.  No teleop, no manual goals.
* SAMPLING    : target confirmed -> cancel motion, stand still, collect
                target (x, y) samples in the map frame, report a robust mean.
* RETURN      : navigate back to the start pose and stop.

Start it with:  ros2 service call /mission/start std_srvs/srv/Trigger
(or launch with autostart:=true).
"""

import enum
import math
import os
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from balance_mission.search_planner import (
    add_headings,
    estimate_position,
    order_waypoints,
    plan_search_waypoints,
    yaw_to_quaternion,
)


class State(enum.Enum):
    IDLE = 0
    WAIT_SYSTEM = 1
    SEARCH = 2
    SAMPLING = 3
    RETURN = 4
    DONE = 5
    FAILED = 6


LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionManager(Node):

    def __init__(self) -> None:
        super().__init__('mission_manager')

        # ---------------- parameters ----------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('autostart', False)
        self.declare_parameter('system_timeout_s', 90.0)
        self.declare_parameter('max_mission_time_s', 0.0)  # 0 = disabled

        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('wait_for_amcl_s', 8.0)

        self.declare_parameter('search_spacing', 0.9)
        self.declare_parameter('search_clearance', 0.22)
        self.declare_parameter('max_search_loops', 2)
        self.declare_parameter('loop_spacing_factor', 0.65)
        self.declare_parameter('spin_at_waypoints', True)
        self.declare_parameter('spin_angle', 6.2832)
        self.declare_parameter('spin_time_allowance_s', 25.0)
        self.declare_parameter('max_waypoint_failures', 6)

        self.declare_parameter('settle_time_s', 0.8)
        self.declare_parameter('sample_window_s', 1.5)
        self.declare_parameter('sample_timeout_s', 5.0)
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('outlier_m', 0.15)

        self.declare_parameter('return_max_retries', 3)
        self.declare_parameter('return_match_start_yaw', False)
        self.declare_parameter(
            'report_file', '~/ros2_target_report.yaml')

        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('spin_action', '/spin')
        self.declare_parameter('confirmed_topic', '/mission/target_confirmed')
        self.declare_parameter(
            'target_map_topic', '/mission/target_map_position')

        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.robot_frame = p('robot_frame').value
        self.system_timeout = float(p('system_timeout_s').value)
        self.max_mission_time = float(p('max_mission_time_s').value)
        self.publish_initial_pose = bool(p('publish_initial_pose').value)
        self.initial_pose = [float(v) for v in p('initial_pose').value]
        self.wait_for_amcl = float(p('wait_for_amcl_s').value)
        self.search_spacing = float(p('search_spacing').value)
        self.search_clearance = float(p('search_clearance').value)
        self.max_search_loops = int(p('max_search_loops').value)
        self.loop_spacing_factor = float(p('loop_spacing_factor').value)
        self.spin_at_waypoints = bool(p('spin_at_waypoints').value)
        self.spin_angle = float(p('spin_angle').value)
        self.spin_time_allowance = float(p('spin_time_allowance_s').value)
        self.max_wp_failures = int(p('max_waypoint_failures').value)
        self.settle_time = float(p('settle_time_s').value)
        self.sample_window = float(p('sample_window_s').value)
        self.sample_timeout = float(p('sample_timeout_s').value)
        self.min_samples = int(p('min_samples').value)
        self.outlier_m = float(p('outlier_m').value)
        self.return_max_retries = int(p('return_max_retries').value)
        self.return_match_yaw = bool(p('return_match_start_yaw').value)
        self.report_file = os.path.expanduser(p('report_file').value)

        # ---------------- state ----------------
        self.state = State.IDLE
        self.t_start: Optional[float] = None
        self.t_wait_start: Optional[float] = None
        self.start_pose: Optional[PoseStamped] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.amcl_stamp: Optional[float] = None
        self.init_pose_sent = 0
        self.last_init_pose_time = 0.0

        self.waypoints: List[Tuple[float, float, float]] = []
        self.wp_index = 0
        self.loops_done = 0
        self.wp_failures = 0
        self.goal_token = 0
        self.active_handle = None
        self.search_phase = 'none'  # none | nav | spin

        self.samples: Deque[Tuple[float, float, float]] = deque(maxlen=80)
        self.t_confirm: Optional[float] = None
        self.target_xy: Optional[Tuple[float, float]] = None
        self.return_retries = 0
        self.tick_count = 0
        self.nav_active = False
        self.bt_future = None

        # ---------------- ROS interfaces ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self, NavigateToPose, p('navigate_action').value)
        self.spin_client = ActionClient(
            self, Spin, p('spin_action').value)

        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, LATCHED)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_callback, LATCHED)
        self.create_subscription(
            Bool, p('confirmed_topic').value, self.confirmed_callback, 10)
        self.create_subscription(
            PointStamped, p('target_map_topic').value,
            self.sample_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.state_pub = self.create_publisher(
            String, '/mission/state', LATCHED)
        self.report_pub = self.create_publisher(
            PointStamped, '/mission/target_report', LATCHED)
        self.marker_pub = self.create_publisher(
            Marker, '/mission/target_marker', LATCHED)
        self.wp_pub = self.create_publisher(
            MarkerArray, '/mission/search_waypoints', LATCHED)

        self.bt_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')

        self.clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')

        self.create_service(Trigger, '/mission/start', self.start_service)
        self.create_service(Trigger, '/mission/abort', self.abort_service)

        self.create_timer(0.1, self.tick)

        self.publish_state()
        self.get_logger().info(
            'Mission manager ready. Call /mission/start to begin.')

        if bool(p('autostart').value):
            self.begin_mission()

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def elapsed(self) -> float:
        return 0.0 if self.t_start is None else time.monotonic() - self.t_start

    def set_state(self, new_state: State) -> None:
        if new_state != self.state:
            self.get_logger().info(
                f'STATE {self.state.name} -> {new_state.name} '
                f'(t={self.elapsed():.1f}s)')
        self.state = new_state
        self.publish_state()

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def lookup_robot_pose(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def amcl_callback(self, _msg: PoseWithCovarianceStamped) -> None:
        self.amcl_stamp = time.monotonic()

    def sample_callback(self, msg: PointStamped) -> None:
        self.samples.append(
            (time.monotonic(), msg.point.x, msg.point.y))

    def confirmed_callback(self, msg: Bool) -> None:
        if not msg.data or self.state != State.SEARCH:
            return
        self.get_logger().info(
            'TARGET CONFIRMED: stopping the search and holding still.')
        self.t_confirm = time.monotonic()
        self.cancel_active_goal()
        self.stop_robot()
        self.set_state(State.SAMPLING)

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def start_service(self, _req, res):
        if self.state not in (State.IDLE, State.FAILED, State.DONE):
            res.success = False
            res.message = f'Mission already running ({self.state.name})'
            return res
        self.begin_mission()
        res.success = True
        res.message = 'Mission started'
        return res

    def abort_service(self, _req, res):
        self.cancel_active_goal()
        self.stop_robot()
        self.set_state(State.FAILED)
        res.success = True
        res.message = 'Mission aborted'
        return res

    # ------------------------------------------------------------------
    # mission start
    # ------------------------------------------------------------------
    def begin_mission(self) -> None:
        self.t_start = time.monotonic()
        self.t_wait_start = self.t_start
        self.start_pose = None
        self.waypoints = []
        self.wp_index = 0
        self.loops_done = 0
        self.wp_failures = 0
        self.samples.clear()
        self.target_xy = None
        self.init_pose_sent = 0
        self.return_retries = 0
        self.set_state(State.WAIT_SYSTEM)

    def send_initial_pose(self) -> None:
        x, y, yaw = self.initial_pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03
        self.init_pub.publish(msg)
        self.init_pose_sent += 1
        self.last_init_pose_time = time.monotonic()
        self.get_logger().info(
            f'Published AMCL initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}).')

    def poll_nav_lifecycle(self) -> None:
        """Nav2 advertises its action before it is ACTIVE; wait for ACTIVE."""
        if not self.bt_state_client.service_is_ready():
            return
        if self.bt_future is not None and not self.bt_future.done():
            return
        self.bt_future = self.bt_state_client.call_async(GetState.Request())
        self.bt_future.add_done_callback(self.bt_state_done)

    def bt_state_done(self, future) -> None:
        try:
            state_id = future.result().current_state.id
            self.nav_active = state_id == LifecycleState.PRIMARY_STATE_ACTIVE
        except Exception:  # service vanished: treat as not active
            self.nav_active = False

    def tick_wait_system(self) -> None:
        now = time.monotonic()
        waited = now - (self.t_wait_start or now)

        if waited > self.system_timeout:
            self.get_logger().error(
                'System not ready (Nav2 / TF / map). Mission FAILED.')
            self.set_state(State.FAILED)
            return

        if (self.publish_initial_pose and self.init_pose_sent < 3
                and now - self.last_init_pose_time > 1.0
                and self.init_pub.get_subscription_count() > 0):
            self.send_initial_pose()

        self.poll_nav_lifecycle()
        if not self.nav_client.server_is_ready() or not self.nav_active:
            return
        if self.map_msg is None:
            return
        transform = self.lookup_robot_pose()
        if transform is None:
            return

        # Give AMCL a moment to accept the initial pose.
        if self.publish_initial_pose and waited < self.wait_for_amcl:
            if self.init_pose_sent < 1 or self.amcl_stamp is None:
                return

        self.record_start_pose(transform)
        if not self.build_waypoints(first_loop=True):
            self.get_logger().error(
                'Could not build any search waypoints from the map.')
            self.set_state(State.FAILED)
            return

        self.set_state(State.SEARCH)
        self.send_next_waypoint()

    def record_start_pose(self, transform) -> None:
        t = transform.transform
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = t.translation.x
        pose.pose.position.y = t.translation.y
        pose.pose.orientation = t.rotation
        self.start_pose = pose
        self.get_logger().info(
            f'Start pose recorded in {self.map_frame}: '
            f'x={t.translation.x:.2f}, y={t.translation.y:.2f}')

    # ------------------------------------------------------------------
    # search
    # ------------------------------------------------------------------
    def build_waypoints(self, first_loop: bool) -> bool:
        if self.map_msg is None:
            return False

        transform = self.lookup_robot_pose()
        if transform is not None:
            here = (transform.transform.translation.x,
                    transform.transform.translation.y)
        else:
            here = (self.start_pose.pose.position.x,
                    self.start_pose.pose.position.y)

        spacing = self.search_spacing
        if not first_loop:
            spacing *= self.loop_spacing_factor

        info = self.map_msg.info
        points = plan_search_waypoints(
            self.map_msg.data, info.width, info.height, info.resolution,
            (info.origin.position.x, info.origin.position.y),
            here, spacing=spacing, clearance=self.search_clearance,
        )
        if not points:
            return False

        route = order_waypoints(points, here)
        self.waypoints = add_headings(route, here)
        self.wp_index = 0
        self.get_logger().info(
            f'Search plan: {len(self.waypoints)} waypoints '
            f'(spacing {spacing:.2f} m, pass {self.loops_done + 1}/'
            f'{self.max_search_loops}).')
        self.publish_waypoint_markers()
        return True

    def publish_waypoint_markers(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for i, (x, y, _yaw) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = 'search_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.5, 1.0, 0.9
            array.markers.append(m)
        self.wp_pub.publish(array)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_next_waypoint(self) -> None:
        if self.state != State.SEARCH:
            return

        if self.wp_index >= len(self.waypoints):
            self.loops_done += 1
            if self.loops_done >= self.max_search_loops:
                self.get_logger().warning(
                    'Search finished without confirming the target. '
                    'Returning to start.')
                self.begin_return()
                return
            if not self.build_waypoints(first_loop=False):
                self.begin_return()
                return

        if self.wp_failures >= self.max_wp_failures:
            self.get_logger().error(
                'Too many failed waypoints. Returning to start.')
            self.begin_return()
            return

        x, y, yaw = self.waypoints[self.wp_index]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)

        self.goal_token += 1
        token = self.goal_token
        self.search_phase = 'nav'
        self.get_logger().info(
            f'Waypoint {self.wp_index + 1}/{len(self.waypoints)}: '
            f'({x:.2f}, {y:.2f})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, t=token: self.goal_response(f, t, 'nav'))

    def goal_response(self, future, token: int, kind: str) -> None:
        handle = future.result()
        if token != self.goal_token:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warning(f'{kind} goal rejected.')
            self.on_goal_finished(kind, False)
            return
        self.active_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, t=token: self.goal_result(f, t, kind))

    def goal_result(self, future, token: int, kind: str) -> None:
        if token != self.goal_token:
            return
        self.active_handle = None
        status = future.result().status
        self.on_goal_finished(kind, status == GoalStatus.STATUS_SUCCEEDED)

    def on_goal_finished(self, kind: str, ok: bool) -> None:
        if self.state == State.RETURN and kind == 'return':
            self.on_return_finished(ok)
            return
        if self.state != State.SEARCH:
            return

        if kind == 'nav':
            if not ok:
                self.wp_failures += 1
                self.get_logger().warning(
                    f'Waypoint {self.wp_index + 1} not reached; skipping.')
                self.wp_index += 1
                self.send_next_waypoint()
                return
            if self.spin_at_waypoints and self.spin_client.server_is_ready():
                self.send_spin()
                return
            self.wp_index += 1
            self.send_next_waypoint()
        elif kind == 'spin':
            if not ok:
                self.get_logger().warning(
                    'Scan spin did not finish; moving on.')
            self.wp_index += 1
            self.send_next_waypoint()

    def send_spin(self) -> None:
        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_angle)
        allowance = DurationMsg()
        allowance.sec = int(self.spin_time_allowance)
        goal.time_allowance = allowance

        self.goal_token += 1
        token = self.goal_token
        self.search_phase = 'spin'
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, t=token: self.goal_response(f, t, 'spin'))

    def cancel_active_goal(self) -> None:
        self.goal_token += 1  # ignore any late results of the old goal
        handle, self.active_handle = self.active_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        self.search_phase = 'none'

    # ------------------------------------------------------------------
    # target sampling and reporting
    # ------------------------------------------------------------------
    def tick_sampling(self) -> None:
        self.stop_robot()
        now = time.monotonic()
        since = now - (self.t_confirm or now)
        if since < self.settle_time:
            return

        fresh = [(x, y) for t, x, y in self.samples
                 if t >= (self.t_confirm or 0.0) + self.settle_time]

        window_done = since >= self.settle_time + self.sample_window
        if len(fresh) >= self.min_samples and window_done:
            self.finish_sampling(fresh, 'stationary')
            return

        if since >= self.sample_timeout:
            recent = [(x, y) for t, x, y in self.samples
                      if t >= (self.t_confirm or 0.0) - 3.0]
            if fresh:
                self.finish_sampling(fresh, 'stationary(few)')
            elif recent:
                self.finish_sampling(recent, 'pre-stop fallback')
            else:
                self.get_logger().error(
                    'Target confirmed but no map-frame samples arrived. '
                    'Check target_localization_node / target_map_node / TF.')
                self.begin_return()

    def finish_sampling(self, points, source: str) -> None:
        estimate = estimate_position(points, self.outlier_m)
        if estimate is None:
            self.begin_return()
            return
        x, y, used = estimate
        self.target_xy = (x, y)
        t_found = self.elapsed()

        self.get_logger().info(
            f'TARGET LOCATION (map frame): x = {x:.3f} m, y = {y:.3f} m '
            f'[{used}/{len(points)} samples, {source}, '
            f't = {t_found:.1f} s]')

        report = PointStamped()
        report.header.frame_id = self.map_frame
        report.header.stamp = self.get_clock().now().to_msg()
        report.point.x = x
        report.point.y = y
        self.report_pub.publish(report)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.08
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            1.0, 0.2, 0.2, 0.95)
        self.marker_pub.publish(marker)

        try:
            os.makedirs(os.path.dirname(self.report_file) or '.',
                        exist_ok=True)
            with open(self.report_file, 'w', encoding='utf-8') as handle:
                yaml.safe_dump({
                    'frame': self.map_frame,
                    'target_x_m': round(x, 4),
                    'target_y_m': round(y, 4),
                    'samples_used': used,
                    'samples_total': len(points),
                    'time_to_report_s': round(t_found, 2),
                }, handle)
        except OSError as error:
            self.get_logger().warning(f'Could not write report file: {error}')

        self.begin_return()

    # ------------------------------------------------------------------
    # return to start
    # ------------------------------------------------------------------
    def begin_return(self) -> None:
        if self.start_pose is None:
            self.get_logger().error('No start pose recorded; cannot return.')
            self.set_state(State.FAILED)
            return
        self.cancel_active_goal()
        self.set_state(State.RETURN)
        self.send_return_goal()

    def send_return_goal(self) -> None:
        if not self.nav_client.server_is_ready():
            self.get_logger().error('Nav2 not available for the return.')
            self.on_return_finished(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.start_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        if not self.return_match_yaw:
            transform = self.lookup_robot_pose()
            if transform is not None:
                dx = (self.start_pose.pose.position.x
                      - transform.transform.translation.x)
                dy = (self.start_pose.pose.position.y
                      - transform.transform.translation.y)
                if math.hypot(dx, dy) > 0.2:
                    qx, qy, qz, qw = yaw_to_quaternion(math.atan2(dy, dx))
                    goal.pose.pose.orientation.x = qx
                    goal.pose.pose.orientation.y = qy
                    goal.pose.pose.orientation.z = qz
                    goal.pose.pose.orientation.w = qw

        self.goal_token += 1
        token = self.goal_token
        self.get_logger().info('Returning to the start position.')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, t=token: self.goal_response(f, t, 'return'))

    def on_return_finished(self, ok: bool) -> None:
        if ok:
            self.stop_robot()
            self.set_state(State.DONE)
            xy = ('not reported' if self.target_xy is None else
                  f'({self.target_xy[0]:.3f}, {self.target_xy[1]:.3f}) m')
            self.get_logger().info(
                f'MISSION COMPLETE in {self.elapsed():.1f} s. '
                f'Target at {xy}. Robot is back at the start.')
            return

        self.return_retries += 1
        if self.return_retries > self.return_max_retries:
            self.get_logger().error('Return failed repeatedly. FAILED.')
            self.set_state(State.FAILED)
            return

        self.get_logger().warning(
            f'Return attempt failed; clearing costmaps and retrying '
            f'({self.return_retries}/{self.return_max_retries}).')
        for client in (self.clear_global, self.clear_local):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

        timer_holder = {}

        def retry() -> None:
            timer_holder['t'].cancel()
            if self.state == State.RETURN:
                self.send_return_goal()

        timer_holder['t'] = self.create_timer(1.5, retry)

    # ------------------------------------------------------------------
    # main tick
    # ------------------------------------------------------------------
    def tick(self) -> None:
        self.tick_count += 1
        if self.tick_count % 20 == 0:
            self.publish_state()

        if self.state == State.WAIT_SYSTEM:
            self.tick_wait_system()
        elif self.state == State.SAMPLING:
            self.tick_sampling()
        elif self.state == State.SEARCH:
            if (self.max_mission_time > 0.0
                    and self.elapsed() > self.max_mission_time):
                self.get_logger().warning(
                    'Mission time budget used up. Returning to start.')
                self.begin_return()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
