#! /usr/bin/python3
import os
from pathlib import Path

import numpy as np
from ultralytics import YOLO


class ConeDetector:
    def __init__(self, device: str, detection_height_threshold: int):
        yolo_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "yolov8.pt"
        keypoint_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "keypoint_detector.pt"

        self.device = device

        # YOLOv8 settings
        self.yolo_model = YOLO(yolo_model_path)
        self.cone_image_margin = 5
        
        
        # self.keypoint_model = 

    def find_cones(self, image: np.ndarray):
        """
        Runs the cone position estimation pipeline on an image

        Args:
            image: The input image
        Returns:
            An Nx4 array of cones: category, X, Y, Z
        """
        yolo_detections = self.yolo_model.predict(image, device="0")[0]
        print(np.sum(list(yolo_detections.speed.values())))

        cones = np.empty((0, 4))
        for bbox in yolo_detections.boxes:
            category = bbox.cls.int().detach().cpu().item()
            cone_confidence = bbox.conf.detach().cpu().item()
            bbox_kpts = [round(x) for x in bbox.xyxy[0].detach().cpu().numpy()]
            bbox_kpts[0] = max(0, bbox_kpts[0] - self.cone_image_margin)
            bbox_kpts[1] = max(0, bbox_kpts[1] - self.cone_image_margin)

            print(category)
        print("---")

        return np.array([
            [1, 5, 0.1, -0.3],
        ])