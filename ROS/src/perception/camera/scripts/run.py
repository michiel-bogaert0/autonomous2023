#! /usr/bin/python3

import os
import time
from pathlib import Path
from typing import List

import numpy as np
import numpy.typing as npt
import rospy
import torch
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point
from node_fixture.node_fixture import create_diagnostic_message
from sensor_msgs.msg import Image
from three_stage_model import ThreeStageModel
from two_stage_model import TwoStageModel
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
            raise NotImplementedError("Visualisation is not yet implemented, but code is ready")
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
        self.device = (
            "cuda:0"
            if torch.cuda.is_available() and rospy.get_param("~cuda", True)
            else "cpu"
        )

        # Camera settings
        self.focal_length = 8  # in mm
        self.camera_matrix = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / "camera_calibration_baumer.npz"
        )["camera_matrix"]
        self.sensor_height = 5.76  # in mm
        self.image_size = (1200, 1920)  # HxW

        # Model parameters

        # Maximum distance for keypoint matching (in pixels)
        self.matching_threshold_px = rospy.get_param(
            "~matching_threshold_px", 150
        )
        # The minimum height of a cone detection in px for it to be run through the keypoint detector
        self.detection_height_threshold = rospy.get_param(
            "~detection_height_threshold", 15
        )
        # Cones further than this are dropped during detection (in meters)
        self.detection_max_distance = rospy.get_param(
            "~detection_max_distance", 15
        )

        self.use_two_stage = rospy.get_param(
            "~use_two_stage", False
        )
        self.keypoint_detector_model = rospy.get_param(
            "~keypoint_detector_model", "mobilenetv3_threestage.pt"
        )

        if self.use_two_stage and "threestage" in self.keypoint_detector_model:
            rospy.logerr("You chose a three-stage keypoint detector but want to run in two-stage mode. Maybe check this:/")

        if not self.use_two_stage and "twostage" in self.keypoint_detector_model:
            rospy.logerr("You chose a two-stage keypoint detector but want to run in three-stage mode. Maybe check this:/")

        yolo_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / "yolov8.pt"
        keypoint_model_path = Path(os.getenv("BINARY_LOCATION")) / "nn_models" / self.keypoint_detector_model
        self.pipeline = None

        if self.use_two_stage:
            raise NotImplementedError("TwoStage is not yet implemented, but code is ready")
            self.pipeline = TwoStageModel(
                keypoint_model_path,
                self.height_to_pos,
                image_size=self.image_size,
                matching_threshold_px=self.matching_threshold_px,
                detection_height_threshold=self.detection_height_threshold,
                detection_max_distance=self.detection_max_distance,
                device=self.device,
            )
        else:
            self.pipeline = ThreeStageModel(
                yolo_model_path,
                keypoint_model_path,
                height_to_pos=self.height_to_pos,
                camera_matrix=self.camera_matrix,
                detection_height_threshold=self.detection_height_threshold,
                detection_max_distance=self.detection_max_distance,
                device=self.device,
            )


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
        # Wait for the pipeline to initialise
        if self.pipeline is None:
            return

        image = self.ros_img_to_np(ros_image)

        # The image should be an RGB Opencv array of HxWx3
        cones, cone_confidences, latencies = self.pipeline.predict(image)

        msg = self.create_observation_msg(cones, ros_image.header)

        timing_msg = f"[{np.sum(latencies):.0f}]"
        for t in latencies:
            timing_msg += f" {t:.1f}"
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[PERC] Pipeline latency",
                message= f"{timing_msg} ms",
            )
        )

        self.pub_cone_locations.publish(msg)

    def height_to_pos(
        self, categories: npt.ArrayLike, heights: npt.ArrayLike, bottoms: npt.ArrayLike
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
            / (self.sensor_height * heights / self.image_size[0])
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
