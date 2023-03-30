#! /usr/bin/python3

import rospy
from ugr_msgs.msg import (
    ObservationWithCovarianceArrayStamped,
    ObservationWithCovariance,
    Observation,
)
from geometry_msgs.msg import Point
from typing import List
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Image
from node_fixture.node_fixture import create_diagnostic_message
import torch
from cone_detector import ConeDetector
import numpy as np
import numpy.typing as npt


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

        # Setup node and spin
        self.cone_detector = ConeDetector(
            self.device, self.tensorrt, self.detection_height_threshold
        )

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="perception camera node",
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

        img = self.ros_img_to_np(ros_image)

        # Nx4 array of cones: category, X, Y, Z
        cones = self.cone_detector.process_image(img)

        msg = self.create_observation_msg(cones, ros_image.header)

        self.pub_cone_locations.publish(msg)

    def create_observation_msg(
        cones: npt.ArrayLike, header
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
