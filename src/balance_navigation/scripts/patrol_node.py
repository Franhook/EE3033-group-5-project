#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import random
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from action_msgs.msg import GoalStatus

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('strategy', 'random')
        self.strategy = self.get_parameter('strategy').value

        self.get_logger().info(f"Patrol Node Started! Strategy: {self.strategy}")
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to the costmap
        self.costmap = None
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        self.waiting_for_result = False
        self.timer = self.create_timer(2.0, self.patrol_step)
        self.mixed_step = 0
        
        # Record the active navigation Goal Handle for explicit cancellation
        self.current_nav_goal_handle = None
        # Maximum tolerated recovery count; immediately choose a new target after this limit
        self.max_tolerated_recoveries = 2

    def costmap_callback(self, msg):
        self.costmap = msg

    # Validation logic - check whether this coordinate is safe
    def is_safe_goal(self, x, y):
        if self.costmap is None:
            return True # If the map has not arrived, allow the request through and let Nav2 reject it

        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        width = self.costmap.info.width
        height = self.costmap.info.height

        # Convert to grid indices
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        # Bounds check
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False 

        index = grid_y * width + grid_x
        cost = self.costmap.data[index]

        # Cost definition: 0=Free, 1~252=inflation area (increasingly dangerous), 253=inscribed obstacle, 254=lethal collision, -1/255=unknown
        if 0 <= cost <= 30:
            return True
        return False

    def patrol_step(self):
        if self.waiting_for_result:
            return

        if self.strategy == 'spin':
            self.execute_spin()
        elif self.strategy == 'mixed':
            # Alternate between spinning and random displacement
            if self.mixed_step % 2 == 0:
                self.execute_spin()
            else:
                self.execute_random(forward_only=False)
            self.mixed_step += 1
        elif self.strategy == 'wander':
            # Wandering strategy with a preference for forward exploration
            self.execute_random(forward_only=True)
        else:
            # Pure random walk
            self.execute_random(forward_only=False)

    def execute_spin(self):
        if not self.spin_client.wait_for_server(timeout_sec=1.0):
            return
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = 2.0 
        self.get_logger().info('Dispatching Spin patrol goal...')
        self.waiting_for_result = True
        
        send_goal_future = self.spin_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.spin_response_callback)

    def execute_random(self, forward_only=False):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            return
            
        try:
            # Get the robot's current real-time pose
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            
            # Convert the quaternion to the current yaw angle (Yaw)
            q = t.transform.rotation
            curr_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except TransformException:
            return

        # Try at most 50 times to find a completely safe point
        target_x, target_y = 0.0, 0.0
        safe_point_found = False

        for _ in range(50):
            if forward_only:
                # Wander Wander mode: sample only within a ±60-degree (about 1.04-radian) sector in front of the robot to avoid frequent turnarounds
                angle = random.uniform(curr_yaw - 1.04, curr_yaw + 1.04)
            else:
                # Random Random mode: sample uniformly over all 360 degrees
                angle = random.uniform(0, 2 * math.pi)
                
            r = random.uniform(0.5, 1.5)
            test_x = curr_x + r * math.cos(angle)
            test_y = curr_y + r * math.sin(angle)
            
            if self.is_safe_goal(test_x, test_y):
                target_x = test_x
                target_y = test_y
                safe_point_found = True
                break

        if not safe_point_found:
            self.get_logger().warn('Could not find a safe patrol goal in map! Retrying next cycle...')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        
        # Randomly generate the final orientation at the destination
        target_yaw = random.uniform(0, 2 * math.pi)
        
        # Convert yaw to a quaternion (z, w)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.get_logger().info(f'Dispatching Safe goal: X={target_x:.2f}, Y={target_y:.2f}, Yaw={target_yaw*180/math.pi:.0f}deg')
        self.waiting_for_result = True
        self.current_nav_goal_handle = None
        # When sending a goal, attach a feedback callback for real-time status monitoring
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_response_callback)

    # Navigation-callback logic 
    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by server.')
            self.waiting_for_result = False
            return
            
        # Record the accepted Goal Handle so the Feedback callback can cancel it proactively
        self.current_nav_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """ Monitor real-time Nav2 feedback and implement fail-fast logic """
        feedback = feedback_msg.feedback
        recoveries = feedback.number_of_recoveries
        
        # If recovery behavior is triggered and reaches the tolerated limit
        if recoveries >= self.max_tolerated_recoveries:
            self.get_logger().warn(f'Nav2 triggered recovery {recoveries} times! Canceling goal to Fail Fast.')
            # If the handle is still valid, immediately send a cancellation request
            if self.current_nav_goal_handle is not None:
                self.current_nav_goal_handle.cancel_goal_async()
                # Clear it to prevent multiple cancellation requests when feedback arrives too quickly
                self.current_nav_goal_handle = None 

    # Spin-action callback logic
    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.waiting_for_result = False
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    # Generic result callback
    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Patrol goal reached successfully. Ready for next.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 ABORTED the goal. Generating a new goal.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was actively CANCELED (Fast fail triggered). Generating a new goal.')
        else:
            self.get_logger().info(f'Goal finished with unknown status: {status}')
            
        # Release the lock so the timer can generate a new target on the next second
        self.current_nav_goal_handle = None
        self.waiting_for_result = False


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()