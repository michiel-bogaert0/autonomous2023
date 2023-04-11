#! /usr/bin/python3
import os
from pathlib import Path
import numpy as np
import numpy.typing as npt
import torch
from ultralytics import YOLO


class ConeDetector:
    def __init__(self, device: str, detection_height_threshold: int):
        yolo_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "yolov8.pt"
        keypoint_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "keypoint_detector.pt"

        self.device = device

        # YOLOv8 settings
        self.yolo_model = YOLO(yolo_model_path)
        self.keypoint_model = KeypointDetector(keypoint_model_path, detection_height_threshold)

        # Camera settings
        self.focal_length = 8
        self.camera_matrix = np.load(Path(os.getenv("BINARY_LOCATION")) / "pnp" / "camera_calibration_baumer.npz")["camera_matrix"]
        self.sensor_height = 5.76
        self.image_height = 1200

    def find_cones(self, image: npt.ArrayLike):
        """
        Runs the cone position estimation pipeline on an image

        Args:
            image: The input image
        Returns:
            An Nx4 array of cones: category, X, Y, Z
        """
        yolo_detections = self.yolo_model.predict(image, device=self.device)[0]
        print(np.sum(list(yolo_detections.speed.values())))

        image_tensor = yolo_detections.orig_img
        categories, heights, bottoms = self.keypoint_model.predict(image_tensor, yolo_detections.boxes)
        
        cones = self.height_to_pos(categories, heights, bottoms)
            
        return cones
    
    def height_to_pos(self, categories: torch.Tensor, heights: torch.Tensor, bottoms: torch.Tensor) -> npt.ArrayLike:
        """Converts a tensor of cone heights and bottom keypoints to an array of locations
        
        Returns:
            a Nx4 array of category, X, Y, Z
        """
        N = len(categories)
        gt_height = torch.full(N, 0.305)
        gt_height[categories == 2] = 0.5

        gt_ring_height = torch.full(N, 0.0275)
        gt_ring_height[categories == 2] = 0.032

        gt_ring_radius = torch.full(N, 0.153 / 2)
        gt_ring_radius[categories == 2] = 0.196 / 2

        depths = (gt_height - gt_ring_height) * self.focal_length / (self.sensor_height * heights / self.image_height)

        return torch.vstack(
            [
                depths,
                -depths * (bottoms[:, 0] - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
                - gt_ring_radius,
                -depths * (bottoms[:, 1] - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
                - gt_ring_height,
            ]
        ).T
