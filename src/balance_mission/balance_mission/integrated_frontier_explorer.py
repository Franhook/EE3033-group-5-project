#!/usr/bin/env python3
"""Frontier exploration integrated with the existing CombinedTargetNode.

Publishes /mission/state=SEARCH (transient-local) so the detector's existing
confirmation gate is enabled. Stops exploration when /mission/target_confirmed
becomes true, and asks MappingSupervisor to save the current map.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class IntegratedFrontierExplorer(Node):
    def __init__(self):
        super().__init__('integrated_frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('update_period', 2.0)
        self.declare_parameter('minimum_frontier_cells', 5)
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('goal_blacklist_radius', 0.45)
        self.declare_parameter('no_frontier_cycles', 5)
        self.declare_parameter('save_map_on_finish', True)

        p = lambda name: self.get_parameter(name).value
        self.map_frame = str(p('map_frame'))
        self.robot_frame = str(p('robot_frame'))
        self.min_frontier = int(p('minimum_frontier_cells'))
        self.free_threshold = int(p('free_threshold'))
        self.blacklist_radius = float(p('goal_blacklist_radius'))
        self.no_frontier_limit = int(p('no_frontier_cycles'))
        self.save_map_on_finish = bool(p('save_map_on_finish'))

        self.map_msg = None
        self.goal_active = False
        self.goal_handle = None
        self.current_goal_xy = None
        self.failed_goals = []
        self.no_frontier_count = 0
        self.finished = False
        self.target_found = False
        self.last_target_label = 'unknown'
        self.last_target_map = None

        # Match CombinedTargetNode's transient-local mission-state subscription.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(
            String, '/mission/state', state_qos)
        self.status_pub = self.create_publisher(
            String, '/exploration/status_text', 10)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            OccupancyGrid, str(p('map_topic')), self.map_callback, 1)
        self.create_subscription(
            Bool, '/mission/target_confirmed', self.target_callback, 10)
        self.create_subscription(
            String, '/detector/matched_reference', self.label_callback, 10)
        from geometry_msgs.msg import PointStamped
        self.create_subscription(
            PointStamped, '/mission/target_map_position',
            self.target_map_callback, 10)

        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.save_client = self.create_client(
            Trigger, '/mapping/save_map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(float(p('update_period')), self.tick)
        self.get_logger().info(
            'Integrated frontier explorer started. Detector confirmation '
            'gate will be enabled by latched /mission/state=SEARCH.')

    def publish_state(self, value):
        msg = String()
        msg.data = value
        self.state_pub.publish(msg)

    def map_callback(self, msg):
        self.map_msg = msg

    def label_callback(self, msg):
        if msg.data:
            self.last_target_label = msg.data

    def target_map_callback(self, msg):
        self.last_target_map = (
            msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)

    def target_callback(self, msg):
        if not msg.data or self.target_found:
            return
        self.target_found = True
        self.finished = True
        self.publish_state('TARGET_FOUND')
        self.get_logger().info(
            f'TARGET CONFIRMED: {self.last_target_label}; '
            f'map position={self.last_target_map}')
        self.cancel_current_goal()
        self.publish_stop()
        self.publish_status('TARGET_FOUND')
        self.request_map_save()

    def publish_status(self, value):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def publish_stop(self):
        # Send a short burst after cancelling the Nav2 goal.
        stop = Twist()
        for _ in range(3):
            self.stop_pub.publish(stop)

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'Could not cancel Nav2 goal: {exc}')
        self.goal_active = False

    def request_map_save(self):
        if not self.save_map_on_finish:
            return
        if not self.save_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(
                '/mapping/save_map unavailable; map was not auto-saved.')
            return
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(self.save_result_callback)

    def save_result_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Map save completed: {response.message}')
            else:
                self.get_logger().error(
                    f'Map save failed: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'Map save service error: {exc}')

    @staticmethod
    def quaternion_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def grid_to_world(self, gx, gy):
        info = self.map_msg.info
        res = info.resolution
        lx = (gx + 0.5) * res
        ly = (gy + 0.5) * res
        yaw = self.quaternion_yaw(info.origin.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        return (
            info.origin.position.x + c * lx - s * ly,
            info.origin.position.y + s * lx + c * ly,
        )

    def is_free(self, x, y):
        info = self.map_msg.info
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return False
        value = self.map_msg.data[y * info.width + x]
        return 0 <= value < self.free_threshold

    def is_unknown(self, x, y):
        info = self.map_msg.info
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return False
        return self.map_msg.data[y * info.width + x] == -1

    def is_frontier(self, x, y):
        if not self.is_free(x, y):
            return False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if (dx or dy) and self.is_unknown(x + dx, y + dy):
                    return True
        return False

    def find_frontier_clusters(self):
        info = self.map_msg.info
        visited = set()
        clusters = []

        for y in range(1, info.height - 1):
            for x in range(1, info.width - 1):
                if (x, y) in visited or not self.is_frontier(x, y):
                    continue
                queue = deque([(x, y)])
                visited.add((x, y))
                cluster = []
                while queue:
                    cx, cy = queue.popleft()
                    cluster.append((cx, cy))
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            nx, ny = cx + dx, cy + dy
                            if (nx, ny) in visited:
                                continue
                            if self.is_frontier(nx, ny):
                                visited.add((nx, ny))
                                queue.append((nx, ny))
                if len(cluster) >= self.min_frontier:
                    clusters.append(cluster)
        return clusters

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception as exc:
            self.get_logger().debug(f'Waiting for robot TF: {exc}')
            return None

    def is_blacklisted(self, xy):
        return any(
            math.hypot(xy[0] - old[0], xy[1] - old[1])
            < self.blacklist_radius
            for old in self.failed_goals
        )

    def choose_goal(self, clusters, robot_xy):
        candidates = []
        for cluster in clusters:
            cx = sum(cell[0] for cell in cluster) / len(cluster)
            cy = sum(cell[1] for cell in cluster) / len(cluster)
            # Use an actual known-free frontier cell nearest the centroid.
            cell = min(
                cluster,
                key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            wx, wy = self.grid_to_world(*cell)
            if self.is_blacklisted((wx, wy)):
                continue
            distance = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            # Prefer large frontiers, but avoid choosing very distant ones.
            score = distance / math.sqrt(len(cluster))
            candidates.append((score, wx, wy))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1], candidates[0][2]

    def send_goal(self, x, y, robot_xy):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = math.atan2(y - robot_xy[1], x - robot_xy[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        self.goal_active = True
        self.current_goal_xy = (x, y)
        self.get_logger().info(
            f'Navigating to frontier x={x:.2f}, y={y:.2f}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warning('Nav2 rejected frontier goal.')
                self.mark_current_failed()
                self.goal_active = False
                return
            self.goal_handle = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        except Exception as exc:
            self.get_logger().error(f'Goal response failed: {exc}')
            self.mark_current_failed()
            self.goal_active = False

    def mark_current_failed(self):
        if self.current_goal_xy is not None:
            self.failed_goals.append(self.current_goal_xy)

    def goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Frontier reached; selecting next area.')
            else:
                self.get_logger().warning(
                    f'Frontier navigation ended with status {result.status}.')
                self.mark_current_failed()
        except Exception as exc:
            self.get_logger().error(f'Goal result failed: {exc}')
            self.mark_current_failed()
        self.goal_active = False
        self.goal_handle = None
        self.current_goal_xy = None

    def tick(self):
        # Re-publish SEARCH so a detector that starts later receives the
        # transient-local state. Do not leave SEARCH active after completion.
        if not self.finished:
            self.publish_state('SEARCH')
        if self.finished:
            return
        if self.map_msg is None:
            self.publish_status('WAITING_FOR_MAP')
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.publish_status('WAITING_FOR_NAV2')
            return
        robot_xy = self.robot_xy()
        if robot_xy is None:
            self.publish_status('WAITING_FOR_TF')
            return
        if self.goal_active:
            return

        clusters = self.find_frontier_clusters()
        goal = self.choose_goal(clusters, robot_xy) if clusters else None
        if goal is None:
            self.no_frontier_count += 1
            self.publish_status('CHECKING_FOR_FRONTIERS')
            if self.no_frontier_count >= self.no_frontier_limit:
                self.finished = True
                self.publish_state('MAPPING_COMPLETE')
                self.publish_status('MAPPING_COMPLETE')
                self.get_logger().info(
                    'No unblacklisted frontier goals remain; saving map.')
                self.request_map_save()
            return

        self.no_frontier_count = 0
        self.publish_status('EXPLORING')
        self.send_goal(goal[0], goal[1], robot_xy)


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_current_goal()
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
