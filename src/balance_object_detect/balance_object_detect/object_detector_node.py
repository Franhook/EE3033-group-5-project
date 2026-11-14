import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# Import interfaces 
from balance_interface.msg import TargetInfo
from balance_interface.srv import SetTarget
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# Import detector modules
from balance_object_detect.yolo_detector import YoloDetector
from balance_object_detect.reid_detector import ReIDDetector
from balance_object_detect.generic_detector import GenericDetector

import cv2
import numpy as np
import time

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        # --- 1. Parameter declarations ---
        self.declare_parameter('yolo_model', 'yolo26n.pt')
        self.declare_parameter('reid_model', 'osnet_x0_25')
        self.declare_parameter('generic_model', 'mobilenet_v3_small')
        self.declare_parameter('conf_thres', 0.3)
        self.declare_parameter('person_match_thres', 0.50)
        self.declare_parameter('object_match_thres', 0.60)
        self.declare_parameter('lost_timeout', 1.0)
        self.declare_parameter('track_method', 'kalman')
        self.declare_parameter('expand_ratio', 0.00)
        self.declare_parameter('feature_update_rate', 0.00)
        self.declare_parameter('enable_point_undistortion', True)
        self.declare_parameter('reset_timeout', 3.0)
        self.declare_parameter('use_compressed_image', False)
        self.declare_parameter('learning_thres', 0.80)
        self.declare_parameter('max_bank_size', 3)
        self.declare_parameter('negative_ttl', 10.0)

        self.conf_thres = self.get_parameter('conf_thres').value
        self.person_match_thres = self.get_parameter('person_match_thres').value
        self.object_match_thres = self.get_parameter('object_match_thres').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.track_method = self.get_parameter('track_method').value
        self.expand_ratio = self.get_parameter('expand_ratio').value
        self.feature_update_rate = self.get_parameter('feature_update_rate').value
        self.enable_point_undistortion = self.get_parameter('enable_point_undistortion').value
        self.reset_timeout = self.get_parameter('reset_timeout').value
        self.use_compressed_image = self.get_parameter('use_compressed_image').value
        self.learning_thres = self.get_parameter('learning_thres').value
        self.max_bank_size = self.get_parameter('max_bank_size').value
        self.negative_ttl = self.get_parameter('negative_ttl').value
        # Used to store the camera intrinsic matrix (K) and distortion coefficients (D)
        self.camera_matrix = None
        self.dist_coeffs = None

        # --- 2. QoS QoS configuration ---
        # Video streams and high-frequency control data: Best Effort (do not retransmit lost packets, preserving real-time performance))
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 3. Initialize the three main models ---
        self.bridge = CvBridge()
        self.get_logger().info("Initializing Models...")
        
        # A. YOLO (Responsible for finding targets)
        self.yolo = YoloDetector(
            model_name=self.get_parameter('yolo_model').value,
            conf_thres=self.conf_thres
        )
        
        # B. ReID (Responsible for person features)
        try:
            self.reid_extractor = ReIDDetector(model_name=self.get_parameter('reid_model').value)
        except:
            self.get_logger().warn("ReID Init Failed. Person tracking might not work.")
            self.reid_extractor = None

        # C. Generic (Responsible for generic-object features)
        generic_name = self.get_parameter('generic_model').value
        self.generic_extractor = GenericDetector(model_name=generic_name)
        
        # --- 4. State variables ---
        self.target_feature = None      # Feature vector of the locked target
        self.target_class_id = None     # Locked target class ID (for example, 0=person, 47=apple))
        self.target_class_name = ""     # Class name
        self.tracking_mode = "IDLE"     # IDLE, PERSON, OBJECT
        self.initial_target_feature = None  # Original feature
        self.has_reset_feature = False      # Flag indicating whether a reset has already occurred
        self.last_seen_time = 0.0 # Timestamp when the target was last seen
        self.last_valid_msg = None # Last valid message
        # Dictionary used to store the blacklist： {'feat': feature_array, 'last_seen': float}
        self.negative_bank = []
        # Cache the most recent IDLE result for responding to mouse clicks
        self.latest_idle_detections = []
        self.latest_idle_image = None
        # First-observation flag
        self.is_first_match = False
        # Global frame timestamp
        self.last_frame_time = time.time()

        # --- 5. Communication interfaces ---
        # Subscribe to images (raw or compressed))
        if self.use_compressed_image:
            self.sub_image = self.create_subscription(
                CompressedImage, '/image_raw/compressed', self.image_callback, qos_profile=qos_sensor)
            self.get_logger().info("Subscribed to COMPRESSED image stream (/image_raw/compressed).")
        else:
            self.sub_image = self.create_subscription(
                Image, '/image_raw', self.image_callback, qos_profile=qos_sensor)
            self.get_logger().info("Subscribed to RAW image stream (/image_raw).")
            
        # Publish the result image (Best Effort)
        if self.use_compressed_image:
            self.pub_result = self.create_publisher(
                CompressedImage, '/detector/result_image/compressed', qos_profile=qos_sensor)
        else:
            self.pub_result = self.create_publisher(
                Image, '/detector/result_image', qos_profile=qos_sensor)
            
        # Publish target information (Best Effort)
        self.pub_info = self.create_publisher(
            TargetInfo, '/detector/target_info', qos_profile=qos_sensor)
        
        # Target-setting service server (Reliable)
        self.srv_set_target = self.create_service(
            SetTarget, '/detector/set_target', self.set_target_callback)
        
        # Register the service that resets the target
        self.srv_reset_target = self.create_service(Empty, '/detector/reset_target', self.reset_target_callback)
        
        # Subscribe to camera intrinsics for single-point distortion correction
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 1)
        
        # Recognition-state-machine publisher
        self.pub_status = self.create_publisher(String, '/detector/status', 10)

        # Subscribe to mouse-click coordinates for target selection
        self.sub_click = self.create_subscription(Point, '/detector/click_point', self.click_callback, 10)
        
        # --- 6. Initialize the Kalman filter ---
        if self.track_method == "kalman":
            self.get_logger().info("Initializing Kalman Filter Tracker...")
            # Four-dimensional state: [center x, center y, velocity vx, velocity vy]vy]
            # Two-dimensional observation: [center x, center y]y]
            self.kf = cv2.KalmanFilter(4, 2)
            # Measurement matrix (only positions x and y are observable) x, y)
            self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], np.float32)
            # State-transition matrix (physical motion: x = x + vx*dt); assume dt=1 for a simplified model
            self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
            # Process noise: keep position changes small (1e-3) while allowing velocity to change abruptly(5e-2)
            self.kf.processNoiseCov = np.array([[1e-3, 0, 0, 0],
                                                [0, 1e-3, 0, 0],
                                                [0, 0, 5e-2, 0],
                                                [0, 0, 0, 5e-2]], np.float32)
            # Measurement-noise covariance
            self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
            # Posterior error-covariance matrix (initial value expressing confidence in the initial state))
            self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 1.0
            self.kf_initialized = False # Flag indicating whether the first coordinate has been supplied

        self.get_logger().info("Object Detector Node Ready. Waiting for target...")

    def calculate_iou(self, box1, box2):
        """Calculate intersection over union for two bounding boxes (Intersection over Union)"""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2

        # Calculate intersection coordinates
        inter_x1 = max(x1_1, x1_2)
        inter_y1 = max(y1_1, y1_2)
        inter_x2 = min(x2_1, x2_2)
        inter_y2 = min(y2_1, y2_2)

        # Calculate intersection area
        inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
        
        # Calculate the area of each box
        box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)

        # Prevent division by zero 0
        denominator = float(box1_area + box2_area - inter_area)
        if denominator == 0:
            return 0.0
            
        return inter_area / denominator

    def _initialize_target(self, roi, class_id, class_name):
        """ Entry point for feature extraction and state initialization """
        if class_id == 0: 
            self.tracking_mode = "PERSON"
            if self.reid_extractor:
                feat = self.reid_extractor.extract_feature(roi)
            else:
                self.get_logger().error("ReID model not loaded!")
                return False, "ReID model not loaded."
        else:
            self.tracking_mode = "OBJECT"
            feat = self.generic_extractor.extract_feature(roi)

        if feat is not None:
            self.target_feature = feat
            self.target_class_id = class_id
            self.target_class_name = class_name
            self.initial_target_feature = feat.copy()
            self.has_reset_feature = False
            self.negative_bank.clear()
            self.last_seen_time = time.time()
            self.last_valid_msg = None
            self.is_first_match = True
            return True, f"Locked on {class_name} (ID:{class_id})."
        else:
            return False, "Feature extraction failed."

    def camera_info_callback(self, msg):
        """ Get the camera intrinsics and distortion coefficients; cache them after the first retrieval """
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Intrinsics loaded for Point Undistortion.")

    def reset_target_callback(self, request, response):
        """ Service callback: reset the currently locked target and return to IDLE mode """
        self.tracking_mode = "IDLE"
        self.target_feature = None
        self.target_class_id = None
        self.target_class_name = ""
        self.initial_target_feature = None
        self.negative_bank.clear()
        self.latest_idle_detections = []
        self.is_first_match = False
        self.get_logger().info("Target reset. Returning to IDLE mode.")
        return response

    def click_callback(self, msg):
        """ Subscription callback: respond to a GUI click and try to lock the clicked target """
        if self.tracking_mode != "IDLE":
            self.get_logger().warn("Click ignored: Detector is already tracking.")
            return
            
        if not self.latest_idle_detections or self.latest_idle_image is None:
            return

        cx, cy = msg.x, msg.y
        best_det = None
        min_area = float('inf')

        for det in self.latest_idle_detections:
            x1, y1, x2, y2 = det['box']
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                area = (x2 - x1) * (y2 - y1)
                if area < min_area:
                    min_area = area
                    best_det = det
        
        if best_det is None:
            return
        
        x1, y1, x2, y2 = best_det['box']
        cv_img = self.latest_idle_image
        h, w = cv_img.shape[:2]
        pad_w = int((x2 - x1) * self.expand_ratio)
        pad_h = int((y2 - y1) * self.expand_ratio)
        roi = cv_img[max(0, y1 - pad_h):min(h, y2 + pad_h), max(0, x1 - pad_w):min(w, x2 + pad_w)]
        
        success, msg_txt = self._initialize_target(roi, best_det['cls'], best_det['label'])
        
        if success:
            self.get_logger().info(f"GUI Clicked! {msg_txt}")
            status_msg = String()
            status_msg.data = "TRACKING"
            self.pub_status.publish(status_msg)


    def set_target_callback(self, request, response):
        """
        Service callback: receive a screenshot, determine what it contains, and extract its features
        """
    def set_target_callback(self, request, response):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(request.target_image, "bgr8")
            detections = self.yolo.detect(cv_img, classes=None)
            
            if not detections:
                response.success = False
                response.message = "No object detected."
                return response
            
            target_obj = detections[0] 
            x1, y1, x2, y2 = target_obj['box']
            img_h, img_w = cv_img.shape[:2]
            pad_w = int((x2 - x1) * self.expand_ratio)
            pad_h = int((y2 - y1) * self.expand_ratio)
            roi = cv_img[max(0, y1 - pad_h):min(img_h, y2 + pad_h), max(0, x1 - pad_w):min(img_w, x2 + pad_w)]

            # 【Refactoring: directly call the unified target-profile creation function】
            success, msg_txt = self._initialize_target(roi, target_obj['cls'], target_obj['label'])
            response.success = success
            response.message = msg_txt
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def image_callback(self, msg):
        """ Video-stream processing loop """
        # Record the exact time this image frame was received
        start_process_time = time.time()
        current_time = time.time()
        # Calculate the actual elapsed time since the previous frame (dt)
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        # 1. If currently in IDLE mode
        if self.tracking_mode == "IDLE":
            # Continuously send an IDLE heartbeat to the GUI
            status_msg = String()
            status_msg.data = "IDLE"
            self.pub_status.publish(status_msg)

            if self.pub_result.get_subscription_count() > 0:
                if self.use_compressed_image:
                    cv_img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                else:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Keep a copy of the original image
                self.latest_idle_image = cv_img.copy()
                # Run whole-image target detection without restricting classes
                raw_detections = self.yolo.detect(cv_img, classes=None)
                
                # Use custom NMS to remove duplicate detections
                detections = []
                for det in raw_detections:
                    is_duplicate = False
                    for f_det in detections:
                        if self.calculate_iou(det['box'], f_det['box']) > 0.60:
                            is_duplicate = True
                            break
                    if not is_duplicate:
                        detections.append(det)
                    
                self.latest_idle_detections = detections # Cache the preliminary candidate coordinates
                # Draw all objects that can be selected
                for det in detections:
                    bx1, by1, bx2, by2 = det['box']
                    label = det['label']
                    conf = det['conf']
                    cv2.rectangle(cv_img, (bx1, by1), (bx2, by2), (0, 0, 255), 2)
                    cv2.putText(cv_img, f"{label} {conf:.2f}", (bx1, by1-8), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # GUI Status indicator in the upper-left corner
                cv2.putText(cv_img, "State: IDLE (Select Target)", (15, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                if self.use_compressed_image:
                    self.pub_result.publish(self.bridge.cv2_to_compressed_imgmsg(cv_img, dst_format='jpg'))
                else:
                    self.pub_result.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
            return

        try:
            # Decode
            if self.use_compressed_image:
                # Decompress the JPEG byte array into an OpenCV image
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = cv_img.shape[:2]

            # Calculate the time difference in advance
            time_diff = current_time - self.last_seen_time

            # TTLDynamic TTL forgetting mechanism
            if self.negative_bank:
                # If the target has already been lost (leaving BUFFERING and entering SEARCHING), pause blacklist aging
                if time_diff > self.lost_timeout:
                    for n in self.negative_bank:
                        n['last_seen'] += dt # Move last_seen forward to offset the increase in current_time
                
                # Perform the normal TTL expiration process
                original_len = len(self.negative_bank)
                self.negative_bank = [n for n in self.negative_bank if (current_time - n['last_seen']) <= self.negative_ttl]
                if len(self.negative_bank) < original_len:
                    self.get_logger().info("TTL Expired: Removed old negative samples from memory bank.")

            # Predict in advance to maintain the KF state
            pred_state = None
            if self.track_method == "kalman" and self.kf_initialized:
                pred_state = self.kf.predict()

            # --- A. Target-specific detection ---
            raw_detections = self.yolo.detect(cv_img, classes=[self.target_class_id])

            # Remove duplicate detections
            detections = []
            for det in raw_detections:
                is_duplicate = False
                for f_det in detections:
                    if self.calculate_iou(det['box'], f_det['box']) > 0.60:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    detections.append(det)
            
            best_match = None
            max_score = -30.0
            best_raw_sim = 0.0
            
            # --- B. Iterate through and match candidates ---
            active_thres = self.person_match_thres if self.tracking_mode == "PERSON" else self.object_match_thres
            for det in detections:
                x1, y1, x2, y2 = det['box']
                if (x2-x1) < 5 or (y2-y1) < 5: continue

                # Crops from real-time detections also need proportional expansion
                pad_w = int((x2 - x1) * self.expand_ratio)
                pad_h = int((y2 - y1) * self.expand_ratio)
                nx1 = max(0, x1 - pad_w)
                ny1 = max(0, y1 - pad_h)
                nx2 = min(w, x2 + pad_w)
                ny2 = min(h, y2 + pad_h)
                roi = cv_img[ny1:ny2, nx1:nx2]
                
                curr_feat = None
                if self.tracking_mode == "PERSON":
                    curr_feat = self.reid_extractor.extract_feature(roi)
                else:
                    curr_feat = self.generic_extractor.extract_feature(roi)
                
                # Calculate positive-sample similarity
                pos_sim = 0.0
                if curr_feat is not None:
                    pos_sim = float(np.dot(self.target_feature, curr_feat))
                # Calculate the maximum negative-sample similarity
                neg_sim = 0.0
                if self.max_bank_size > 0 and self.negative_bank and curr_feat is not None:
                    neg_sims = [float(np.dot(n['feat'], curr_feat)) for n in self.negative_bank]
                    neg_sim = max(neg_sims)

                # Score using the difference
                score = pos_sim - neg_sim if self.negative_bank else pos_sim

                det['sim'] = pos_sim     # Keep the original similarity only for UI display and blacklist-admission decisions
                det['score'] = score     # Combined score used for candidate selection and locking
                det['feat'] = curr_feat
                
                if score > max_score:
                    max_score = score
                    best_raw_sim = pos_sim
                    best_match = det

            # --- C. Decision logic ---
            target_msg = TargetInfo()
            status_msg = String()
            # State flag used by subsequent drawing logic
            is_locked = False
            is_buffered = False

            if time_diff > self.lost_timeout or self.is_first_match:
                can_lock = (best_raw_sim >= self.learning_thres)
            else:
                can_lock = (best_raw_sim >= active_thres)

            if best_match and can_lock and max_score > 0:
                self.is_first_match = False
                is_locked = True
                self.last_seen_time = current_time
                self.has_reset_feature = False # Restore the lock and clear the reset flag
                status_msg.data = "TRACKING"

            # --- Negative-sample admission and feature updating ---
            if self.max_bank_size > 0:
                for det in detections:
                    # If the target was successfully locked, skip the true target
                    if is_locked and det is best_match:
                        continue
                    
                    if det.get('feat') is not None and det.get('sim', 0.0) > active_thres:
                        # If the current box's absolute similarity to the target is at least that of the selected box, it is a true target that was incorrectly rejected
                        if is_locked and det.get('sim', 0.0) >= best_raw_sim:
                            continue
                        curr_feat = det['feat']
                        
                        # Find the most similar negative sample
                        max_n_sim = -1.0
                        best_n_idx = -1
                        for i, n in enumerate(self.negative_bank):
                            n_sim = float(np.dot(n['feat'], curr_feat))
                            if n_sim > max_n_sim:
                                max_n_sim = n_sim
                                best_n_idx = i
                                
                        if max_n_sim > self.learning_thres + 0.10:
                            # 【Sample update: if a new sample is too similar to an existing negative sample, update the old sample using feature fusion）
                            self.negative_bank[best_n_idx]['last_seen'] = current_time 
                            blended_neg = 0.8 * self.negative_bank[best_n_idx]['feat'] + 0.2 * curr_feat
                            self.negative_bank[best_n_idx]['feat'] = blended_neg / np.linalg.norm(blended_neg)
                        else:
                            # 【New-sample addition: if the new sample is dissimilar to every existing negative sample and exceeds active_thres, add it to the negative-sample database
                            if is_locked:
                                self.negative_bank.append({'feat': curr_feat.copy(), 'last_seen': current_time})
                                
                # Maintain the queue length
                while len(self.negative_bank) > self.max_bank_size:
                    self.negative_bank.pop(0)

            if is_locked:
                # CASE A: Target locked successfully; track normally
                bx1, by1, bx2, by2 = best_match['box']
                cx, cy = (bx1+bx2)//2, (by1+by2)//2

                # --- Dynamic target-feature update ---
                if (best_match['sim'] >= self.learning_thres) and self.feature_update_rate > 0.0:
                    best_feat = best_match['feat']
                    # Fuse the previous memory with the new image
                    blended = (1.0 - self.feature_update_rate) * self.target_feature + self.feature_update_rate * best_feat
                    # Apply L2 normalization again (mandatory for cosine similarity))
                    self.target_feature = blended / np.linalg.norm(blended)

                # --- Single-point coordinate distortion correction ---
                real_cx, real_cy = float(cx), float(cy)
                if self.enable_point_undistortion and self.camera_matrix is not None and self.dist_coeffs is not None:
                    # cv2.undistortPoints requires a three-dimensional array (N, 1, 2)
                    pts = np.array([[[cx, cy]]], dtype=np.float32)
                    # Passing P=camera_matrix converts the output directly back to pixel coordinates instead of physically normalized coordinates
                    undistorted = cv2.undistortPoints(
                        pts, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)
                    real_cx, real_cy = undistorted[0][0]

                # --- Kalman-update logic ---
                if self.track_method == "kalman":
                    meas = np.array([[real_cx], [real_cy]], dtype=np.float32)
                    if not self.kf_initialized:
                        # On the first target observation, forcibly initialize the state
                        self.kf.statePost = np.array([[real_cx], [real_cy], [0.0], [0.0]], dtype=np.float32)
                        self.kf_initialized = True
                    else:
                        # Correct the internal KF state
                        self.kf.correct(meas)
                    # Predict smoothed coordinates
                    real_cx, real_cy = int(self.kf.statePost[0]), int(self.kf.statePost[1])
                    
                # Assemble the message
                target_msg.detected = True
                target_msg.similarity = float(max_score)
                target_msg.center_x = int(real_cx)
                target_msg.center_y = int(real_cy)
                target_msg.width = int(bx2 - bx1)
                target_msg.height = int(by2 - by1)
                target_msg.error_x = (real_cx - w/2) / (w/2)
                target_msg.error_y = (real_cy - h/2) / (h/2)
                target_msg.delay_time = float(time.time() - start_process_time)
                # Record the most recent width and height for blind-zone prediction drawing
                self.last_width = target_msg.width
                self.last_height = target_msg.height
                self.last_valid_msg = target_msg
                
            else:
                if self.last_valid_msg is not None and time_diff < self.lost_timeout:
                    # CASE B: Target temporarily lost: use KF prediction to maintain it and enter the buffering state
                    is_buffered = True
                    status_msg.data = "BUFFERING"
                    target_msg = TargetInfo() # Create a new object to avoid modifying old memory

                    if self.track_method == "kalman" and self.kf_initialized:
                        #  Kalman prediction-only logic 
                        self.kf.statePost = pred_state 
                        cx, cy = int(pred_state[0]), int(pred_state[1])
                        
                        target_msg.detected = True
                        target_msg.similarity = self.last_valid_msg.similarity # Reuse the previous similarity value
                        target_msg.center_x = cx
                        target_msg.center_y = cy
                        target_msg.width = self.last_width
                        target_msg.height = self.last_height
                        target_msg.error_x = (cx - w/2) / (w/2)
                        target_msg.error_y = (cy - h/2) / (h/2)
                        target_msg.delay_time = float(time.time() - start_process_time)
                    else:
                        # Retain the previous-frame data
                        target_msg = self.last_valid_msg
                else:
                    # CASE C: Target completely lost
                    target_msg.detected = False
                    if self.track_method == "kalman":
                        self.kf_initialized = False

                    if time_diff > self.reset_timeout:
                        status_msg.data = "SEARCHING" # Ask the patrol node to take control
                        # Template-reset logic
                        if not self.has_reset_feature and self.initial_target_feature is not None:
                            self.target_feature = self.initial_target_feature.copy()
                            self.has_reset_feature = True
                            self.get_logger().info("Target lost for too long. Template reset to INITIAL to prevent drift.")
                    else:
                        status_msg.data = "LOST" # The target is lost, but remain waiting; patrol is not needed yet

            # --- D. Publish control information and state-machine status ---
            self.pub_info.publish(target_msg)
            self.pub_status.publish(status_msg)

            # --- E. Draw and publish images (only when subscribers exist)) ---
            # Check the subscriber count; skip subsequent drawing when there are no subscribers to save CPUCPU
            if self.pub_result.get_subscription_count() > 0:
                
                # Draw all boxes (red: not locked))
                for det in detections:
                    if is_locked and det is best_match:
                        continue
                    bx1, by1, bx2, by2 = det['box']
                    sim_val = det.get('sim', 0.0)
                    score_val = det.get('score', 0.0)
                    
                    cv2.rectangle(cv_img, (bx1, by1), (bx2, by2), (0, 0, 255), 2)
                    # Upper-left outside the box: original similarity (sim)
                    cv2.putText(cv_img, f"Sim:{sim_val:.2f}", (bx1, by1-8), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    # Upper-left inside the box: combined score (score)
                    cv2.putText(cv_img, f"Scr:{score_val:.2f}", (bx1+5, by1+20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Draw the locked box (green) or buffering box (yellow))
                if is_locked:
                    bx1, by1, bx2, by2 = best_match['box']
                    sim_val = best_match.get('sim', 0.0)
                    score_val = best_match.get('score', 0.0)
                    
                    cv2.rectangle(cv_img, (bx1, by1), (bx2, by2), (0, 255, 0), 3)
                    cv2.putText(cv_img, f"Sim:{sim_val:.2f}", (bx1, by1-8), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(cv_img, f"Scr:{score_val:.2f}", (bx1+5, by1+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                           
                elif is_buffered:
                    msg = target_msg
                    cx, cy = msg.center_x, msg.center_y
                    bw, bh = msg.width, msg.height
                    x1, y1 = cx - bw//2, cy - bh//2
                    x2, y2 = cx + bw//2, cy + bh//2
                    
                    cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(cv_img, f"PREDICTED", (x1, y1-8), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Upper left: status and timer
                state_str = status_msg.data
                time_lost_str = f"Lost: {time_diff:.1f}s" if state_str != "TRACKING" else "Lost: 0.0s"
                ui_color = (0, 255, 0) if state_str == "TRACKING" else ((0, 255, 255) if state_str == "BUFFERING" else (0, 0, 255))
                
                cv2.putText(cv_img, f"State: {state_str}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ui_color, 2)
                cv2.putText(cv_img, time_lost_str, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ui_color, 2)

                # Upper right: target class name
                if self.target_class_name:
                    text_size = cv2.getTextSize(f"Target: {self.target_class_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                    text_x = w - text_size[0] - 15
                    cv2.putText(cv_img, f"Target: {self.target_class_name}", (text_x, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                if self.use_compressed_image:
                    self.pub_result.publish(self.bridge.cv2_to_compressed_imgmsg(cv_img, dst_format='jpg'))
                else:
                    self.pub_result.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Loop Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
