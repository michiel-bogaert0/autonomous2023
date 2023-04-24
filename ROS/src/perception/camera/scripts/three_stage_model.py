import cv2
import numpy as np
import numpy.typing as npt
import torch
import time

from cone_detector import ConeDetector
from keypoint_detector import KeypointDetector

CAT_TO_COLOUR = {
    0: (255, 0, 0),
    1: (0, 250, 250),
    2: (0, 100, 220),
    3: (0, 130, 220),
}


class ThreeStageModel():
    def __init__(
        self,
        yolo_model_path,
        keypoint_model_path,
        camera_matrix,
        height_to_pos,
        detection_height_threshold=15,
        detection_max_distance=15,
        device="cuda:0",
    ):
        self.height_to_pos = height_to_pos
        self.detection_height_threshold = detection_height_threshold
        self.detection_max_distance = detection_max_distance
        self.device = device

        self.cone_detector = ConeDetector(
            yolo_model_path, self.device, detection_height_threshold
        )

        # Create the keypoint detector
        self.keypoint_detector = KeypointDetector(
            keypoint_model_path, self.detection_height_threshold, self.device
        )

        # Camera settings
        self.camera_matrix = camera_matrix

        # Timing info
        self.starter, self.ender = torch.cuda.Event(
            enable_timing=True
        ), torch.cuda.Event(enable_timing=True)

    def predict(self, original_image: npt.ArrayLike):
        
        latencies = []

        # The image should be 3xHxW and on the GPU
        start = time.perf_counter()
        image = torch.from_numpy(original_image).to(self.device).permute(2, 0, 1)
        latencies.append(1000*(time.perf_counter() - start))

        # Nx6 array of cones: xyxy, conf, cat
        start = time.perf_counter()
        bboxes = self.cone_detector.find_cones(image)
        latencies.append(1000*(time.perf_counter() - start))

        # Predict keypoints        
        start = time.perf_counter()
        valid_cones, heights, bottoms = self.keypoint_detector.predict(image, bboxes)
        latencies.append(1000*(time.perf_counter() - start))
        
        # Find cone locations
        start = time.perf_counter()
        categories = bboxes[valid_cones, -1]
        confidences = bboxes[valid_cones, -2]
        cones = self.height_to_pos(categories, heights, bottoms)
        latencies.append(1000*(time.perf_counter() - start))

        return cones, confidences, latencies

    def visualise(self, image, bboxes, bottoms):
        for bbox in bboxes:
            bbox_int = np.rint(bbox).astype(int)
            image = cv2.rectangle(
                image, bbox_int[:2], bbox_int[2:4], CAT_TO_COLOUR[int(bbox_int[-1])], 2
            )

        for bottom in bottoms:
            bottom_int = np.rint(bottom).astype(int)
            image = cv2.circle(image, bottom_int, 3, (255, 255, 255), -1)
            image = cv2.circle(image, bottom_int, 2, (0, 0, 0), -1)
            image = cv2.circle(image, bottom_int, 1, (255, 255, 255), -1)

        return image
