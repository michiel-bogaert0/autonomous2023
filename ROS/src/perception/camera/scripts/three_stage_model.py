import time

import cv2
import numpy as np
import numpy.typing as npt
import torch
from cone_detector import ConeDetector
from keypoint_detector import KeypointDetector
from ugr_msgs.msg import BoundingBox, BoundingBoxesStamped

CAT_TO_COLOUR = {
    0: (255, 0, 0),
    1: (0, 250, 250),
    2: (0, 100, 220),
    3: (0, 130, 220),
}
FSOCO_TO_NORMAL_CAT = np.array([1, 0, 3, 2])


class ThreeStageModel:
    def __init__(
        self,
        yolo_model_path,
        keypoint_model_path,
        height_to_pos,
        pub_bounding_boxes,
        camera_matrix,
        detection_height_threshold=33,
        device="cuda:0",
        visualise=False,
    ):
        self.height_to_pos = height_to_pos
        self.pub_bounding_boxes = pub_bounding_boxes
        self.detection_height_threshold = detection_height_threshold
        self.device = device
        self.use_vis = visualise

        self.cone_detector = ConeDetector(
            yolo_model_path, self.device, detection_height_threshold
        )

        # Create the keypoint detector
        self.keypoint_detector = KeypointDetector(keypoint_model_path, self.device)

        # Camera settings
        self.camera_matrix = camera_matrix

    def predict(self, original_image: npt.ArrayLike, header):
        """Given an image, pass it through the global keypoint detector, match the keypoints, and return the cones that were found.

        Args:
            original_image: the numpy image object of (H, W, 3)
            header: the original ROS header of the image

        Returns:
            - an array of Nx5 containing (belief, category, x, y, z)
            - the pipeline latencies containing triplets of (pre-processing, bbox inference, kpt inference, post-processing)
            - (optional) a visualisation image
        """
        # Don't sync GPU and CPU for timing
        # This is less acurate, but does not slow down the code as much
        latencies = []

        # The image should be 3xHxW, normalised, and on the GPU
        start = time.perf_counter()
        image = torch.from_numpy(original_image).to(self.device).permute(2, 0, 1) / 255
        latencies.append(1000 * (time.perf_counter() - start))

        # Nx6 array of cones: xyxy, conf, cat
        start = time.perf_counter()
        bboxes = self.cone_detector.find_cones(image)
        latencies.append(1000 * (time.perf_counter() - start))

        bbox_msg = BoundingBoxesStamped()
        bbox_msg.header = header

        # type, score, left, top, width, height
        bbox_msg.bounding_boxes = [
            BoundingBox(
                int(row[5]),
                row[4],
                row[0],
                row[1],
                row[2] - row[0],
                row[3] - row[1],
            )
            for row in bboxes
        ]
        self.pub_bounding_boxes.publish(bbox_msg)

        # Predict keypoints
        start = time.perf_counter()
        valid_cones, heights, bottoms = self.keypoint_detector.predict(image, bboxes)
        latencies.append(1000 * (time.perf_counter() - start))

        # Find cone locations
        start = time.perf_counter()
        categories = bboxes[valid_cones, -1]
        cones = self.height_to_pos(categories, heights, bottoms)

        # Add the beliefs back
        cones = np.hstack(
            (
                np.reshape(bboxes[valid_cones, 4], (len(bboxes[valid_cones, 4]), 1)),
                cones,
            )
        )

        latencies.append(1000 * (time.perf_counter() - start))

        vis_img = None
        if self.use_vis:
            vis_img = np.copy(original_image)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
            vis_img = self.visualise(vis_img, bboxes, bottoms)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB)

        return cones, latencies, vis_img

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
