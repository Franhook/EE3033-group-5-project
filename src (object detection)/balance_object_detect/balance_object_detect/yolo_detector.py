import cv2
import torch
import os
import requests
import numpy as np

# Try to import YOLO and handle environment dependencies
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False
    print("Error: 'ultralytics' library not found. Please install it.")


class YoloDetector:
    def __init__(self, model_name='yolo11n.pt', device=None, conf_thres=0.5):
        """
        Initialize the YOLO detector
        :param model_name: Model name or path
        :param device: 'cpu' or 'cuda'
        :param conf_thres: Confidence threshold
        """
        self.conf_thres = conf_thres
        self.device = device if device else (
            'cuda' if torch.cuda.is_available() else 'cpu'
        )

        # Automatic download and loading logic
        self.model = self._load_model(model_name)
        print(f"[YoloDetector] Model loaded on {self.device}")

    # -------------------- Edited code --------------------

    def _load_model(self, model_name):
        """Load YOLO weights from the workspace weights directory."""

        # Locate the ROS 2 workspace from this source file.
        workspace_dir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                '..',
                '..',
                '..'
            )
        )

        # Build the path to the model weights.
        target_dir = os.path.join(workspace_dir, 'weights')

        if not model_name.endswith('.pt'):
            model_name += '.pt'

        model_path = os.path.join(target_dir, model_name)

        # Check that the YOLO library is available.
        if not HAS_YOLO:
            raise ImportError(
                "The ultralytics library is not installed."
            )

        # Check that the model weights exist.
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"YOLO model weights not found: {model_path}"
            )

        # Load the model.
        print(f"[YoloDetector] Loading weights from: {model_path}")
        return YOLO(model_path)

    # -------------------- Existing detection code --------------------

    def detect(self, cv_image, classes=None):
        """
        Run detection
        :param cv_image: OpenCV BGR image
        :param classes: (List[int]) Specify class IDs to detect;
                        for example, [0] is person and [47] is apple.
                        None detects all classes.
        :return: list of dicts:
                 [{'box': [x1,y1,x2,y2], 'conf': float, 'cls': int}, ...]
        """
        if self.model is None:
            return []

        # verbose=False disables console output
        results = self.model(
            cv_image,
            classes=classes,
            conf=self.conf_thres,
            device=self.device,
            verbose=False
        )

        detections = []

        for r in results:
            boxes = r.boxes

            for box in boxes:
                # Get integer coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])

                detections.append({
                    'box': [x1, y1, x2, y2],
                    'conf': conf,
                    'cls': cls_id,
                    'label': self.model.names[cls_id]
                })

        return detections