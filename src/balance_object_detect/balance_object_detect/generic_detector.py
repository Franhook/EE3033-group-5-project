import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import numpy as np
import cv2

class GenericDetector:
    def __init__(self, model_name='mobilenet_v3_small', device=None):
        """
        Initialize the generic-object feature extractor
        :param model_name: 'mobilenet_v3_small' or 'mobilenet_v3_large'
        """
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"[GenericDetector] Initializing {model_name} on {self.device}...")
        
        # 1. Load a variant by name (all use the default ImageNet weights))
        if 'large' in model_name.lower():
            self.backbone = models.mobilenet_v3_large(weights=models.MobileNet_V3_Large_Weights.DEFAULT)
        else:
            # Default to small small
            self.backbone = models.mobilenet_v3_small(weights=models.MobileNet_V3_Small_Weights.DEFAULT)
        
        # 2. Remove the final classification layer (Classifier) and retain only the feature extractor
        # MobileNetV3 architecture: input -> features -> avgpool -> classifier
        # After avgpool, flatten the output to obtain semantic features
        self.model = nn.Sequential(
            self.backbone.features,
            self.backbone.avgpool,
            nn.Flatten() 
        )
        
        self.model.to(self.device)
        self.model.eval()

        print("[GenericDetector] Initialization Complete.")

    def preprocess_image(self, cv_img, target_size=(224, 224)):
        """
        LetterboxLetterbox preprocessing: preserve the aspect ratio and pad missing areas with black
        :param target_size: (H, W)
        """
        h, w = cv_img.shape[:2]
        target_h, target_w = target_size
        
        scale = min(target_w / w, target_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        resized = cv2.resize(cv_img, (new_w, new_h))
        
        # Create a black canvas
        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        
        # Calculate the centered position
        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2
        
        # Paste the image
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        return canvas

    def extract_feature(self, cv2_image_crop):
        """Input: OpenCV image (BGR); output: normalized feature vector"""
        if cv2_image_crop is None or cv2_image_crop.size == 0:
            return None

        # --- Optimization: use Letterbox instead of brute-force Resize Resize ---
        # MobileNet MobileNet input is usually 224x224 224x224
        processed_img = self.preprocess_image(cv2_image_crop, target_size=(224, 224))

        # Convert BGR -> RGB BGR -> RGB
        image = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(image)
        
        # Note: transforms no longer needs Resize here; only ToTensor and Normalize are required Normalize
        # Temporarily build a transform used only for conversion transform
        transform_pipeline = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
        input_tensor = transform_pipeline(pil_img).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            features = self.model(input_tensor)
            
        # Normalize (required for cosine similarity))
        features = features.cpu()
        norm = features.norm(p=2, dim=1, keepdim=True)
        features = features.div(norm.expand_as(features))
        
        return features.numpy().flatten()
