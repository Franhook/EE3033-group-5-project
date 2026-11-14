import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus, GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class AutonomousMissionNode(Node):

    def __init__(self):
        super().__init__('autonomous_mission_node')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter(
            'target_confirmed_topic',
            '/mission/target_confirmed'
        )
        self.declare_parameter(
            'navigate_to_pose_action',
            '/navigate_to_pose'
        )
        self.declare_parameter('explore_cancel_service',
                               '/explore/_action/cancel_goal')

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        target_topic = self.get_parameter(
            'target_confirmed_topic'
        ).value

        action_name = self.get_parameter(
            'navigate_to_pose_action'
        ).value

        cancel_service_name = self.get_parameter(
            'explore_cancel_service'
        ).value

        # Mission state
        self.start_pose = None
        self.target_confirmed = False
        self.return_started = False
        self.return_finished = False
        self.exploration_cancel_requested = False

        # Obtain the robot pose in the MAP coordinate frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Nav2 navigation action client.
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            action_name
        )

        # Service used to cancel active exploration goals.
        self.cancel_client = self.create_client(
            CancelGoal,
            cancel_service_name
        )

        # Listen for target confirmation.
        self.target_sub = self.create_subscription(
            Bool,
            target_topic,
            self.target_callback,
            10
        )

        # Periodically try to record the starting pose.
        self.start_timer = self.create_timer(
            0.5,
            self.record_start_pose
        )

        self.get_logger().info(
            'Mission controller started. '
            'Waiting to record the starting pose in map coordinates.'
        )

    def record_start_pose(self):
        if self.start_pose is not None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

        except TransformException:
            # TF may not be ready immediately after startup.
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        values = [
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ]

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'Invalid TF values. Waiting for a valid starting pose.'
            )
            return

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = self.map_frame
        self.start_pose.header.stamp = self.get_clock().now().to_msg()

        self.start_pose.pose.position.x = translation.x
        self.start_pose.pose.position.y = translation.y
        self.start_pose.pose.position.z = translation.z

        self.start_pose.pose.orientation.x = rotation.x
        self.start_pose.pose.orientation.y = rotation.y
        self.start_pose.pose.orientation.z = rotation.z
        self.start_pose.pose.orientation.w = rotation.w

        self.start_timer.cancel()

        self.get_logger().info(
            f'Starting pose recorded in {self.map_frame}: '
            f'x={translation.x:.2f}, y={translation.y:.2f}'
        )

    def target_callback(self, msg):
        if not msg.data:
            return

        if self.target_confirmed:
            return

        self.target_confirmed = True

        self.get_logger().info(
            'Target confirmed. Preparing to stop exploration '
            'and return to the starting position.'
        )

        if self.start_pose is None:
            self.get_logger().error(
                'Cannot return safely: starting pose was not recorded.'
            )
            return

        self.cancel_exploration()

    def cancel_exploration(self):
        if self.exploration_cancel_requested:
            return

        self.exploration_cancel_requested = True

        if not self.cancel_client.service_is_ready():
            self.get_logger().error(
                'Exploration cancellation service is unavailable. '
                'Not sending the return goal while exploration '
                'may still be active.'
            )
            return

        request = CancelGoal.Request()

        # A zero UUID requests cancellation of all goals
        # handled by this action server.
        request.goal_info = GoalInfo()

        future = self.cancel_client.call_async(request)
        future.add_done_callback(self.cancel_result_callback)

    def cancel_result_callback(self, future):
        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Could not cancel exploration: {error}'
            )
            return

        self.get_logger().info(
            f'Exploration cancellation response: '
            f'{response.return_code}'
        )

        # Continue only if the cancellation request was accepted.
        if response.return_code == CancelGoal.Response.ERROR_NONE:
            self.return_to_start()
        else:
            self.get_logger().error(
                'Exploration cancellation was not accepted. '
                'Return navigation has not been started.'
            )

    def return_to_start(self):
        if self.return_started:
            return

        self.return_started = True

        if not self.nav_client.server_is_ready():
            self.get_logger().error(
                'Nav2 NavigateToPose action is unavailable. '
                'Cannot start the return journey.'
            )
            self.return_started = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.start_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            'Sending Nav2 a goal to return to the starting position.'
        )

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Failed to send return goal: {error}'
            )
            self.return_started = False
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Nav2 rejected the return goal.'
            )
            self.return_started = False
            return

        self.get_logger().info(
            'Return goal accepted. Waiting for arrival.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.return_result_callback
        )

    def return_result_callback(self, future):
        try:
            result = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Failed to receive return result: {error}'
            )
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.return_finished = True

            self.get_logger().info(
                'MISSION COMPLETE: robot returned to its starting position.'
            )

        else:
            self.get_logger().error(
                f'Return navigation failed. Nav2 status: {result.status}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()