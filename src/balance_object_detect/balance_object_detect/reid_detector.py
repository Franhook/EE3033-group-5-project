import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image
import numpy as np
import cv2

# Try to import torchreid torchreid
try:
    import torchreid
    HAS_TORCHREID = True
except ImportError:
    HAS_TORCHREID = False
    print("Warning: torchreid not found. ReID features will fail.")

class ReIDDetector:
    def __init__(self, model_name='osnet_x0_25', device=None):
        """
        Initialize the feature extractor (ReID) (ReID)
        """
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"[ReIDDetector] Initializing {model_name} on {self.device}...")

        if not HAS_TORCHREID:
            raise ImportError("torchreid library is missing.")

        try:
            # Build the model using torchreid
            # pretrained=True pretrained=True automatically downloads the weights
            self.model = torchreid.models.build_model(
                name=model_name,
                num_classes=1000,  # No effect in inference mode
                loss='softmax',
                pretrained=True
            )
            self.model.to(self.device)
            self.model.eval()
        except Exception as e:
            print(f"[ReIDDetector] Model load failed: {e}")
            raise e
        
        print("[ReIDDetector] Initialization Complete.")

    def preprocess_image(self, cv_img, target_size=(256, 128)):
        # ... (Code logic is identical to GenericDetector; note the width/height order) ...
        # Note: target_size uses the format (Height, Width)
        h, w = cv_img.shape[:2]
        target_h, target_w = target_size
        scale = min(target_w / w, target_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(cv_img, (new_w, new_h))
        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2

        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        return canvas

    def extract_feature(self, cv2_image_crop):
        """
        Input: cropped target image (OpenCV BGR) (OpenCV BGR)
        Output: normalized feature vector (NumPy array) (Numpy array)
        """
        if cv2_image_crop is None or cv2_image_crop.size == 0:
            return None

        # OSNet requires 256(H) x 128(W) 256(H) x 128(W)
        image = cv2.cvtColor(self.preprocess_image(cv2_image_crop, target_size=(256, 128)), cv2.COLOR_BGR2RGB)
        # OpenCV (BGR) -> PIL (RGB)
        pil_img = Image.fromarray(image)
        
        # Preprocess [1, C, H, W]
        transform_pipeline = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        input_tensor = transform_pipeline(pil_img).unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad():
            features = self.model(input_tensor)
            
        # Normalize features (required for cosine similarity))
        features = features.cpu()
        norm = features.norm(p=2, dim=1, keepdim=True)
        features = features.div(norm.expand_as(features))
        
        # Convert to a one-dimensional NumPy array
        return features.numpy().flatten()

    @staticmethod
    def compute_similarity(feat1, feat2):
        """
        Calculate cosine similarity between two normalized vectors
        Range: -1.0 to 1.0 (closer to 1.0 means more similar))
        """
        if feat1 is None or feat2 is None:
            return 0.0
        return np.dot(feat1, feat2)
