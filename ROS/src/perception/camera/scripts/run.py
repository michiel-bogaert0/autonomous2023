#! /usr/bin/python3

import os
from pathlib import Path
from typing import List

import numpy as np
import numpy.typing as npt
import rospy
import torch
from cone_detector import ConeDetector
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point
from keypoint_detector import KeypointDetector
from node_fixture.node_fixture import create_diagnostic_message
from sensor_msgs.msg import Image
from ugr_msgs.msg import (Observation, ObservationWithCovariance,
                          ObservationWithCovarianceArrayStamped)


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception")

        # ROS parameters
        self.sub_image_input = rospy.Subscriber(
            "/input/image", Image, self.run_perception_pipeline
        )
        self.pub_cone_locations = rospy.Publisher(
            "/output/update",
            ObservationWithCovarianceArrayStamped,
            queue_size=10,
        )

        self.visualise = rospy.get_param("~vis", False)
        if self.visualise:
            self.pub_image_annotated = rospy.Publisher(
                "/output/image_annotated",
                Image,
                queue_size=10,
            )

        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Cone detection parameters
        self.rate = rospy.get_param("~rate", 10)
        self.tensorrt = rospy.get_param("~tensorrt", True)
        self.device = (
            "cuda:0"
            if torch.cuda.is_available() and rospy.get_param("~cuda", True)
            else "cpu"
        )
        # The minimum height of a cone detection in px for it to be run through the keypoint detector
        self.detection_height_threshold = rospy.get_param(
            "~detection_height_threshold", 25
        )

        self.cone_detector = None
        self.cone_detector = ConeDetector(self.device, self.detection_height_threshold)

        keypoint_model_path = (
            Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "keypoint_detector.pt"
        )
        self.keypoint_detector = KeypointDetector(
            keypoint_model_path, self.detection_height_threshold, self.device
        )

        # Camera settings
        self.focal_length = 8
        self.camera_matrix = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / "camera_calibration_baumer.npz"
        )["camera_matrix"]
        self.sensor_height = 5.76
        self.image_height = 1200

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[PERC] Camera",
                message=f"CUDA device used: {self.device}",
            )
        )

        rospy.spin()

    def run_perception_pipeline(self, ros_image: Image) -> None:
        """
        Given an image, run through the entire perception pipeline and publish to ROS

        Args:
            image: The input image as numpy array
            ros_image: The input image as ROS message
        """

        # Wait for the cone detector to initialise
        if self.cone_detector is None or self.keypoint_detector is None:
            return

        img = self.ros_img_to_np(ros_image)

        # Nx4 array of cones: category, X, Y, Z
        yolo_detections = self.cone_detector.find_cones(img)

        image_tensor = torch.tensor(img).to(self.device)
        categories, heights, bottoms = self.keypoint_detector.predict(
            image_tensor, yolo_detections.boxes
        )

        cones = self.height_to_pos(categories, heights, bottoms)

        msg = self.create_observation_msg(cones, ros_image.header)

        self.pub_cone_locations.publish(msg)

    def height_to_pos(
        self, categories: torch.Tensor, heights: torch.Tensor, bottoms: torch.Tensor
    ) -> npt.ArrayLike:
        """Converts a tensor of cone heights and bottom keypoints to an array of locations

        Returns:
            a Nx4 array of category, X, Y, Z
        """
        N = len(categories)
        gt_height = np.full(N, 0.305)
        gt_height[categories == 2] = 0.5

        gt_ring_height = np.full(N, 0.0275)
        gt_ring_height[categories == 2] = 0.032

        gt_ring_radius = np.full(N, 0.153 / 2)
        gt_ring_radius[categories == 2] = 0.196 / 2

        depths = (
            (gt_height - gt_ring_height)
            * self.focal_length
            / (self.sensor_height * heights / self.image_height)
        )

        return np.vstack(
            [
                categories,
                depths,
                -depths
                * (bottoms[:, 0] - self.camera_matrix[0, 2])
                / self.camera_matrix[0, 0]
                - gt_ring_radius,
                -depths
                * (bottoms[:, 1] - self.camera_matrix[1, 2])
                / self.camera_matrix[1, 1]
                - gt_ring_height,
            ]
        ).T

    def create_observation_msg(
        self, cones: npt.ArrayLike, header
    ) -> ObservationWithCovarianceArrayStamped:
        """Given an array of cone locations and a message header, create an observation message

        Args:
            cones: Nx4 array of cone locations: category, X, Y, Z
            header: the header of the original image message

        Returns:
            An ObservationWithCovarianceArrayStamped message containing the detected cones
        """

        cone_positions: List[ObservationWithCovariance] = []

        for cone in cones:
            cone_positions.append(
                ObservationWithCovariance(
                    observation=Observation(
                        observation_class=int(cone[0]),
                        location=Point(*cone[1:]),
                    ),
                    covariance=[0.7, 0, 0, 0, 0.3, 0, 0, 0, 0.1],  # TODO: tweak
                )
            )

        msg = ObservationWithCovarianceArrayStamped()
        msg.observations = cone_positions
        msg.header = header

        return msg

    def ros_img_to_np(self, image: Image) -> np.ndarray:
        """Converts a ros image into an numpy array
        Args:
            image: ros image
        returns:
            numpy array containing the image data
        """
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1
        )

        return img


if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
