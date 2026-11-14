#!/usr/bin/env python3
"""YOLO + confirmation + camera geometry + TF mapping in one ROS 2 node.

"""

import math
import time
import traceback

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from balance_interface.msg import TargetInfo
from balance_object_detect.yolo_detector import YoloDetector


class CombinedTargetNode(Node):
    def __init__(self):
        # Keep detector node name so existing params and ROS CLI commands work.
        super().__init__('object_detector_node')

        # Detector settings taken from current YOLO-only implementation.
        self.declare_parameter('yolo_model', 'yolo26n.pt')
        self.declare_parameter('conf_thres', 0.30)
        self.declare_parameter('yolo_allowed_labels',
                               ['apple', 'bottle', 'banana', 'orange', 'cup', 'stop sign'])
        self.declare_parameter('yolo_confirm_frames', 2)
        self.declare_parameter('lost_timeout', 1.0)
        self.declare_parameter('use_compressed_image', True)
        self.declare_parameter('rotate_image_180', True)
        # Confirmation settings from balance_mission.
        self.declare_parameter('required_detections', 2)
        self.declare_parameter('max_gap_seconds', 0.8)
        self.declare_parameter('max_center_shift_px', 100.0)
        self.declare_parameter('gate_on_mission_state', False)

        # Keep the latest localisation calibration unchanged for comparisons.
        self.declare_parameter('fx', 303.58978001)
        self.declare_parameter('fy', 302.97064273)
        self.declare_parameter('cx', 325.24190807)
        self.declare_parameter('cy', 220.28007631)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_x', -0.035)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_height', 0.09)
        self.declare_parameter('camera_pitch_deg', -5.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')

        p = lambda name: self.get_parameter(name).value
        self.allowed_labels = [str(x).lower() for x in p('yolo_allowed_labels')]
        self.yolo_confirm_frames = max(1, int(p('yolo_confirm_frames')))
        self.lost_timeout = float(p('lost_timeout'))
        self.use_compressed_image = bool(p('use_compressed_image'))
        self.rotate_image_180 = bool(p('rotate_image_180'))
        self.required = max(1, int(p('required_detections')))
        self.max_gap = float(p('max_gap_seconds'))
        self.max_shift = float(p('max_center_shift_px'))
        self.gate = bool(p('gate_on_mission_state'))
        self.fx, self.fy = float(p('fx')), float(p('fy'))
        self.cx, self.cy = float(p('cx')), float(p('cy'))
        self.image_width = int(p('image_width'))
        self.image_height = int(p('image_height'))
        self.camera_x = float(p('camera_x'))
        self.camera_y = float(p('camera_y'))
        self.camera_height = float(p('camera_height'))
        self.camera_pitch = math.radians(float(p('camera_pitch_deg')))
        self.base_frame = str(p('base_frame'))
        self.map_frame = str(p('map_frame'))

        self.bridge = CvBridge()
        self.yolo = YoloDetector(model_name=str(p('yolo_model')),
                                 conf_thres=float(p('conf_thres')))

        self.candidate_label = None
        self.candidate_box = None
        self.candidate_count = 0
        self.locked_label = None
        self.last_seen = 0.0
        self._reset_confirmation()
        self.mission_state = 'UNKNOWN'

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        if self.use_compressed_image:
            self.create_subscription(CompressedImage, '/image_raw/compressed',
                                     self.image_callback, camera_qos)
            self.pub_result = self.create_publisher(
                CompressedImage, '/detector/result_image/compressed', camera_qos)
            camera_topic = '/image_raw/compressed'
        else:
            self.create_subscription(Image, '/image_raw',
                                     self.image_callback, camera_qos)
            self.pub_result = self.create_publisher(
                Image, '/detector/result_image', camera_qos)
            camera_topic = '/image_raw'

        self.pub_status = self.create_publisher(String, '/detector/status', 10)
        self.pub_reference = self.create_publisher(
            String, '/detector/matched_reference', 10)
        self.pub_info = self.create_publisher(
            TargetInfo, '/detector/target_info', camera_qos)
        self.pub_observed = self.create_publisher(
            TargetInfo, '/detector/observed_target', 10)
        self.pub_confirmed = self.create_publisher(
            Bool, '/mission/target_confirmed', 10)
        self.pub_relative = self.create_publisher(
            PointStamped, '/mission/target_relative_position', 10)
        self.pub_map = self.create_publisher(
            PointStamped, '/mission/target_map_position', 10)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, '/mission/state',
                                 self.state_callback, latched)
        self.create_service(Trigger, '/mission/reset_confirmation',
                            self.reset_confirmation_service)
        self.create_service(Empty, '/detector/reset_target',
                            self.reset_target_service)
        self.create_timer(0.5, self.repeat_confirmation)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            'Combined target node ready; camera=%s, mission gate=%s, classes=%s'
            % (camera_topic, self.gate, self.allowed_labels))

    def _reset_confirmation(self):
        self.confirm_count = 0
        self.confirm_last_time = None
        self.confirm_last_center = None
        self.confirmed = False

    def reset_detector(self):
        self.candidate_label = None
        self.candidate_box = None
        self.candidate_count = 0
        self.locked_label = None
        self.last_seen = 0.0

    def reset_confirmation_service(self, _request, response):
        self._reset_confirmation()
        response.success = True
        response.message = 'Confirmation reset'
        return response

    def reset_target_service(self, _request, response):
        self.reset_detector()
        self._reset_confirmation()
        self.get_logger().info('YOLO target and confirmation state reset.')
        return response

    def state_callback(self, msg):
        previous = self.mission_state
        self.mission_state = msg.data
        if msg.data == 'SEARCH' and previous != 'SEARCH':
            self._reset_confirmation()
            self.get_logger().info('Search started: confirmation re-armed.')

    def publish_confirmed(self):
        result = Bool()
        result.data = True
        self.pub_confirmed.publish(result)

    def repeat_confirmation(self):
        if self.confirmed and self.mission_state == 'SEARCH':
            self.publish_confirmed()

    @staticmethod
    def calculate_iou(box1, box2):
        a1, b1, a2, b2 = box1
        c1, d1, c2, d2 = box2
        intersection = (max(0, min(a2, c2) - max(a1, c1))
                        * max(0, min(b2, d2) - max(b1, d1)))
        area1 = max(0, a2-a1) * max(0, b2-b1)
        area2 = max(0, c2-c1) * max(0, d2-d1)
        total = area1 + area2 - intersection
        return intersection / total if total > 0 else 0.0

    def confirm_observation(self, target):
        """Same confirmation policy as the latest standalone mission node."""
        if self.gate and self.mission_state != 'SEARCH':
            self.confirm_count = 0
            self.confirm_last_time = None
            self.confirm_last_center = None
            return
        now = time.monotonic()
        center = (target.center_x, target.center_y)
        consecutive = (
            self.confirm_last_time is not None
            and now - self.confirm_last_time <= self.max_gap
            and math.dist(center, self.confirm_last_center) <= self.max_shift
        )
        self.confirm_count = self.confirm_count + 1 if consecutive else 1
        self.confirm_last_time = now
        self.confirm_last_center = center
        if self.confirm_count >= self.required and not self.confirmed:
            self.confirmed = True
            self.publish_confirmed()
            self.get_logger().info(
                f'TARGET CONFIRMED after {self.confirm_count} observations.')

    def localize(self, target):
        """Preserve the existing geometric projection and frame conventions."""
        if not target.detected or target.width <= 0 or target.height <= 0:
            return
        u = float(target.center_x)
        v = float(target.center_y) + float(target.height)/2.0
        if not (0 <= u < self.image_width
                and self.cy < v < self.image_height - 5):
            self.get_logger().warning(
                'Target ground-contact pixel outside usable image region',
                throttle_duration_sec=2.0)
            return
        alpha = math.atan2(v - self.cy, self.fy)
        ground_angle = self.camera_pitch + alpha
        if ground_angle <= math.radians(2):
            self.get_logger().warning(
                'Ground ray too close to horizon for reliable localization',
                throttle_duration_sec=2.0)
            return
        distance = self.camera_height / math.tan(ground_angle)
        if not math.isfinite(distance) or distance <= 0:
            return
        optical_depth = (distance * math.cos(self.camera_pitch)
                         + self.camera_height * math.sin(self.camera_pitch))
        right_offset = ((u - self.cx)/self.fx) * optical_depth
        robot_x = distance + self.camera_x
        robot_y = -right_offset + self.camera_y
        if not all(map(math.isfinite, (robot_x, robot_y))):
            return

        relative = PointStamped()
        # Same timing as latest localisation node: processing-time stamp.
        # Source TargetInfo carries no image capture timestamp.
        relative.header.stamp = self.get_clock().now().to_msg()
        relative.header.frame_id = self.base_frame
        relative.point.x = float(robot_x)
        relative.point.y = float(robot_y)
        relative.point.z = 0.0
        self.pub_relative.publish(relative)
        self.map_transform(relative)

    def map_transform(self, relative):
        """Try stamped TF first, then latest TF (latest target_map_node)."""
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, relative.header.frame_id,
                    Time.from_msg(relative.header.stamp))
            except TransformException:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, relative.header.frame_id, Time())
            result = do_transform_point(relative, transform)
            result.header.frame_id = self.map_frame
            self.pub_map.publish(result)
            self.get_logger().info(
                f'Target MAP position: x={result.point.x:.3f} m, '
                f'y={result.point.y:.3f} m', throttle_duration_sec=1.0)
        except TransformException as exc:
            self.get_logger().warning(
                f'Map transform unavailable: {exc}',
                throttle_duration_sec=2.0)

    def image_callback(self, msg):
        start = time.time()
        try:
            if self.use_compressed_image:
                frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                     cv2.IMREAD_COLOR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if frame is None:
                return
                
            if self.rotate_image_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            height, width = frame.shape[:2]
            detections = self.yolo.detect(frame, classes=None)
            candidates = [d for d in detections
                          if str(d['label']).lower() in self.allowed_labels]
            if self.locked_label is not None:
                candidates = [d for d in candidates
                              if str(d['label']).lower() == self.locked_label]
            best = max(candidates, key=lambda d: float(d['conf'])) if candidates else None

            info = TargetInfo()
            status = String()
            status.data = 'IDLE'
            if best is not None:
                label = str(best['label']).lower()
                x1, y1, x2, y2 = [int(x) for x in best['box']]
                same = (self.candidate_label == label
                        and self.candidate_box is not None
                        and self.calculate_iou(best['box'], self.candidate_box) >= 0.20)
                self.candidate_count = self.candidate_count+1 if same else 1
                self.candidate_label = label
                self.candidate_box = best['box']
                self.last_seen = start
                if self.candidate_count >= self.yolo_confirm_frames:
                    self.locked_label = label
                    status.data = 'TRACKING'
                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    info.detected = True
                    info.similarity = float(best['conf'])
                    info.center_x = cx
                    info.center_y = cy
                    info.width = x2-x1
                    info.height = y2-y1
                    info.error_x = (cx-width/2)/(width/2)
                    info.error_y = (cy-height/2)/(height/2)
                    info.delay_time = float(time.time()-start)
                    self.pub_observed.publish(info)
                    label_msg = String()
                    label_msg.data = label
                    self.pub_reference.publish(label_msg)
                    self.confirm_observation(info)
                    self.localize(info)
                else:
                    status.data = 'VERIFYING'
                if self.pub_result.get_subscription_count() > 0:
                    color = (0,255,0) if status.data == 'TRACKING' else (0,255,255)
                    cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                    cv2.putText(frame, f'{label} {float(best["conf"]):.2f}',
                                (x1, max(20,y1-8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color, 2)
            else:
                if self.locked_label is not None and start-self.last_seen < self.lost_timeout:
                    status.data = 'LOST'
                else:
                    self.reset_detector()
                    status.data = 'IDLE'
                # Preserve confirmation count until the next real observation;
                # the max_gap_seconds rule prevents stale counts from merging.

            self.pub_info.publish(info)
            self.pub_status.publish(status)
            if self.pub_result.get_subscription_count() > 0:
                cv2.putText(frame, f'YOLO ONLY: {status.data}', (15,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                if self.use_compressed_image:
                    annotated = self.bridge.cv2_to_compressed_imgmsg(
                        frame, dst_format='jpg')
                else:
                    annotated = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                annotated.header = msg.header
                self.pub_result.publish(annotated)
        except Exception as exc:
            self.get_logger().error(
                f'Combined target processing failed: {exc}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTargetNode()
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
