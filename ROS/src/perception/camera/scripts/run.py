#! /usr/bin/python3

import rospy
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Image
from node_fixture.node_fixture import create_diagnostic_message
import torch
from cone_detector import ConeDetector

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
            self.device,
            self.tensorrt,
            self.detection_height_threshold
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

        return


if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
