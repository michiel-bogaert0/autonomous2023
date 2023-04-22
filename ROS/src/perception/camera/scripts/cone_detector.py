#! /usr/bin/python3
import os
from pathlib import Path

import numpy as np
import numpy.typing as npt
import torch
import torchvision.transforms as T
from ultralytics import YOLO


class ConeDetector:
    def __init__(
        self, yolo_model_path: Path, device: str, detection_height_threshold: int
    ):
        self.device = device

        # YOLOv8 settings
        self.yolo_model = YOLO(yolo_model_path)
        self.original_image_size = (1200, 1920)
        self.image_size = (416, 640)
        self.bbox_margin = 5
        self.dummy_input = torch.rand(1, 3, *self.image_size).to(self.device)
        self.detection_height_threshold = detection_height_threshold

        self.resizer = T.Resize(
            (self.image_size[0], self.image_size[1]), antialias=False
        )

        # Warm-up (YOLO does this automatically, so just run one image through)
        print("YOLO warm-up")
        self.yolo_model.predict(self.dummy_input, device=self.device)

    def find_cones(self, image: torch.Tensor, conf: float = 0.25) -> npt.ArrayLike:
        """
        Runs the cone position estimation pipeline on an image

        Args:
            image: The input image (not normalised) 3xHxW
        Returns:
            An array of Nx6: xyxy, conf, cat
        """
        yolo_img = self.resizer(image).unsqueeze(0)
        yolo_detections = self.yolo_model.predict(
            yolo_img, conf=conf, device=self.device
        )[0]

        # Post-process detections
        bb = yolo_detections.boxes.data.cpu().numpy()

        # Rescale the bboxes
        bb[:, :4:2] *= self.original_image_size[1] / self.image_size[1]
        bb[:, 1:4:2] *= self.original_image_size[0] / self.image_size[0]
        bb[:, 0] -= self.bbox_margin
        bb[:, 1] -= self.bbox_margin
        bb[bb[:, 0] < 0, 0] = 0
        bb[bb[:, 1] < 0, 1] = 0
        bb[:, :4] = np.rint(bb[:, :4])

        # Filter out invalid bboxes
        heights, widths = bb[:, 2] - bb[:, 0], bb[:, 3] - bb[:, 1]
        valid_bboxes = np.logical_and(
            heights >= self.detection_height_threshold,
            widths >= self.detection_height_threshold / 4,
        )
        valid_bboxes = np.logical_and(
            bb[:, -1] != 4,
            valid_bboxes,
        )
        bb = bb[valid_bboxes]

        return bb
