#! /usr/bin/python3
import os
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import torch
from geometry_msgs.msg import Point
from .IntBoundingBox import IntBoundingBox
from keypoint_detector.module import RektNetModule
from ugr_msgs.msg import BoundingBox, ConeKeypoint


class ConeKeypointDetector:
    def __init__(self, device: str):
        model_file = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "keypoints.ckpt"
        self.img_size = (60, 80)

        self.device = torch.device(device)

        self.model = RektNetModule.load_from_checkpoint(str(model_file))
        self.model.to(self.device)
        self.model.eval()

    def infer(self, bounding_box_batch: torch.FloatTensor) -> np.ndarray:
        """
        Run inference on a batch of bounding boxes
        Args:
            bounding_box_batch: The bounding box batch

        Returns:
        The inferred keypoints of every cone in the batch
        """
        hms, keypoints = self.model(bounding_box_batch)

        return keypoints.detach().cpu().numpy()

    def create_batch(
        self, image: np.ndarray, bbs: list("ROSBoundingBox")
    ) -> torch.FloatTensor:
        """
        Converts a ConeDetection into a batch Tensor for inference
        Args:
            image: The input image
            bbs: The detected bounding boxes

        Returns:
        The batched Tensor
        """
        int_bbs = [IntBoundingBox.from_img(bb, image) for bb in bbs]
        crops = [image[bb.top : bb.bottom, bb.left : bb.right] for bb in int_bbs]
        pytorch_tensors = [
            self.pytorch_cv_prepare(crop, self.img_size) for crop in crops
        ]

        return torch.stack(pytorch_tensors)

    def to_ros_keypoints(self, keypoints: np.ndarray) -> list("Point"):
        """
        Converts an array of keypoints to a ROS-compatible list
        Args:
            keypoints: Array of keypoint detections

        Returns:
        List of Points corresponding to the keypoints of a cone
        """
        new_keypoints = []
        for x, y in keypoints:
            new_keypoints.append(
                Point(x=x / self.img_size[0], y=y / self.img_size[1], z=0.0)
            )
        return new_keypoints

    def detect_keypoints(
        self, image: np.ndarray, bbs: list("ROSBoundingBox")
    ) -> list("ConeKeypoint"):
        """
        Given a cone detection update message, return the detected keypoints of every cone
        Args:
            image: The input image
            bbs: The detected bounding boxes

        Returns:
        The inferred keypoints of every cone in the detection message as a ConeKeypoint list
        """
        batch = self.create_batch(image, bbs).to(self.device)  # B x 7 x 2
        keypoint_collection = self.infer(batch)

        cone_keypoints = []
        for bb, keypoints in zip(bbs, keypoint_collection):
            n_keypoints = self.to_ros_keypoints(keypoints)
            cone_keypoints.append(ConeKeypoint(bb_info=bb, keypoints=n_keypoints))

        return cone_keypoints

    def pytorch_cv_prepare(self, img: np.ndarray, img_size: Tuple[int, int]):
        img = cv2.resize(img, img_size)
        img = np.moveaxis(img, -1, 0)
        img = img.astype(np.float32)
        img /= 255
        return torch.FloatTensor(img)
