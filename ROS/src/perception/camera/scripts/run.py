#! /usr/bin/python3

import os
import sys
import time
from pathlib import Path
from typing import Tuple

import neoapi
import numpy as np
import rospy
import torch
from cone_detection.cone_detection import ConeDetector
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from keypoint_detection.keypoint_detection import ConeKeypointDetector
from node_fixture.node_fixture import create_diagnostic_message
from pnp.cone_pnp import ConePnp
from sensor_msgs.msg import Image
from tools.tools import np_to_ros_image, ros_img_to_np
from ugr_msgs.msg import (
    BoundingBox,
    ConeKeypoints,
    ObservationWithCovarianceArrayStamped,
)


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception")
        self.pub_keypoints = rospy.Publisher(
            "/output/cone_keypoints", ConeKeypoints, queue_size=10
        )
        self.pub_pnp = rospy.Publisher(
            "/output/update",
            ObservationWithCovarianceArrayStamped,
            queue_size=10,
        )

        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        self.rate = rospy.get_param("~rate", 10)

        self.tensorrt = rospy.get_param("~tensorrt", True)

        # Cone detection
        self.device = (
            "cuda:0"
            if torch.cuda.is_available() and rospy.get_param("~cuda", True)
            else "cpu"
        )
        self.cone_detector = ConeDetector(self.device, self.tensorrt)
        # The minimum height of a cone detection in px for it to be run through the keypoint detector
        self.detection_height_threshold = rospy.get_param(
            "~detection_height_threshold", 30
        )

        # Keypoint detection
        self.keypoint_detector = ConeKeypointDetector(self.device)
        self.publish_keypoints = rospy.get_param("~publish_keypoints", False)

        cone_dev = self.cone_detector.yolo_model.device
        keyp_dev = self.keypoint_detector.device

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="perception camera node",
                message=f"CUDA devices used: cone={cone_dev} - keypoint={keyp_dev}",
            )
        )

        # PNP
        # See documentation for more information about these settings!
        max_distance = rospy.get_param("~max_distance", 20.0)
        scale = rospy.get_param("~scale", 0.001)
        cones_location = rospy.get_param("~cones_location", "cones.npz")
        camcal_location = rospy.get_param(
            "~camcal_location", "camera_calibration_baumer.npz"
        )

        cone_models = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / cones_location
        )
        camera_cal_archive = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camcal_location
        )

        camera_matrix = camera_cal_archive["camera_matrix"]
        distortion_matrix = camera_cal_archive["distortion_matrix"]

        self.pnp = ConePnp(
            cone_models=cone_models,
            scale=scale,
            max_distance=max_distance,
            camera_matrix=camera_matrix,
            distortion_matrix=distortion_matrix,
        )
        self.sub_images = rospy.Subscriber(
            "/input/image", Image, self.run_perception_pipeline
        )
        rospy.spin()

    def run_perception_pipeline(self, ros_image: Image) -> None:
        """
        Given an image, run through the entire perception pipeline and publish to ROS
        Args:
            image: The input image as numpy array
            ros_image: The input image as ROS message
        """

        timings = []

        image = ros_img_to_np(image=ros_image)

        start = time.perf_counter()
        bbs = self.cone_detector.detect_cones(image)
        end = time.perf_counter()
        timings.append(end - start)

        keypoints = []

        # Don't count detections that aren't tall enough
        h, w, c = image.shape
        detection_height_threshold = self.detection_height_threshold / h

        # Filter bbs by height and if they are taller than they are wide
        bbs = [
            bb
            for bb in bbs
            if bb.height > bb.width and bb.height > detection_height_threshold
        ]

        if len(bbs) != 0:
            # There were bounding boxes detected
            start = time.perf_counter()
            keypoints = self.keypoint_detector.detect_keypoints(image, bbs)
            end = time.perf_counter()
            timings.append(end - start)

            # Create a keypoints message
            cone_keypoints_msg = ConeKeypoints()
            cone_keypoints_msg.cone_keypoints = keypoints
            cone_keypoints_msg.header.stamp = ros_image.header.stamp
            cone_keypoints_msg.header.frame_id = ros_image.header.frame_id

            if self.publish_keypoints:
                self.pub_keypoints.publish(cone_keypoints_msg)

            # Run PNP
            start = time.perf_counter()
            self.run_pnp_pipeline(cone_keypoints_msg, (w, h))
            end = time.perf_counter()
            timings.append(end - start)

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[Perception] Camera node",
                message=f"Timings {' - '.join([str(int(x*1000)) for x in timings])} ms",
            )
        )

    def run_pnp_pipeline(self, msg: ConeKeypoints, img_size: Tuple[int, int]) -> None:
        """
        Given a keypoints ROS message, run through the PNP pipeline and publish to ROS
        Args:
            msg: the input keypoints message
            img_size: the size of the original image (W, H)
        """

        update_msg = self.pnp.generate_perception_update(msg, img_size=img_size)
        self.pub_pnp.publish(update_msg)


if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
