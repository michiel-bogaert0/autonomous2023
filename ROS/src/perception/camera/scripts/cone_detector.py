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

        self.device = device

        # YOLOv8 settings
        self.yolo_model = YOLO(yolo_model_path)

    def find_cones(self, image: npt.ArrayLike):
        """
        Runs the cone position estimation pipeline on an image

        Args:
            image: The input image
        Returns:
            An Ultralytics detection object
        """
        yolo_detections = self.yolo_model.predict(image, device=self.device)[0]

        return yolo_detections
