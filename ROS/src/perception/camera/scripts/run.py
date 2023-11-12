#! /usr/bin/python3

import os
import sys
from pathlib import Path
from typing import List

import numpy as np
import numpy.typing as npt
import rospy
import torch
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point
from node_fixture.fixture import create_diagnostic_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from three_stage_model import ThreeStageModel
from two_stage_model import TwoStageModel
from ugr_msgs.msg import (
    BoundingBoxesStamped,
    Observation,
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception")

        # ROS parameters

        # Node I/O
        self.sub_image_input = rospy.Subscriber("/input/image", Image, self.get_image)
        self.pub_cone_locations = rospy.Publisher(
            "/output/update",
            ObservationWithCovarianceArrayStamped,
            queue_size=10,
        )

        # Visualisation and diagnostics
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

        # Hardware parameters
        self.rate = rospy.get_param("~rate", 10)
        self.device = (
            "cuda:0"
            if torch.cuda.is_available() and rospy.get_param("~cuda", True)
            else "cpu"
        )
        # TensorRT is only used for cone detection in the three-stage model
        self.tensorrt = rospy.get_param("~tensorrt", True)
        if self.tensorrt and self.device == "cpu":
            rospy.logerr("TensorRT does not work on CPU:/")

        # Camera settings (based on Baumer/Ricoh combination)
        self.focal_length = 8  # in mm
        self.camera_matrix = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / "camera_calibration_baumer.npz"
        )["camera_matrix"]
        self.sensor_height = 5.76  # in mm
        self.image_size = (1200, 1920)  # HxW

        # Model parameters

        # Maximum distance for keypoint pair matching (in pixels)
        self.matching_threshold_px = rospy.get_param("~matching_threshold_px", 150)
        # The minimum height of a cone detection in px for it to be run through the keypoint detector
        # 33px corresponds to a small cone at 15m distance
        self.detection_height_threshold = rospy.get_param(
            "~detection_height_threshold", 33
        )

        # The keypoint detector used also decides whether to use a 2/3-stage model
        # Best option: unet_threestage.pt               -> more accurate and faster, but uses two neural nets
        # Second best option: mobilenetv3_twostage.pt   -> slower but only one network
        self.keypoint_detector_model = rospy.get_param(
            "~keypoint_detector_model", "unet_threestage.pt"
        )
        self.use_two_stage = "twostage" in self.keypoint_detector_model

        yolo_model_path = (
            Path(os.getenv("BINARY_LOCATION"))
            / "nn_models"
            / f"yolov8s.{'engine' if self.tensorrt else 'pt'}"
        )
        keypoint_model_path = (
            Path(os.getenv("BINARY_LOCATION"))
            / "nn_models"
            / self.keypoint_detector_model
        )
        # Initialise the pipeline as None, since it takes some time to start-up
        self.pipeline = None
        self.ros_image = None

        if self.use_two_stage:
            self.pipeline = TwoStageModel(
                keypoint_model_path,
                self.height_to_pos,
                image_size=(1184, 1920),
                matching_threshold_px=self.matching_threshold_px,
                detection_height_threshold=self.detection_height_threshold,
                device=self.device,
                visualise=self.visualise,
            )
        else:
            # We only publish bboxes in 3-stage mode
            self.pub_bounding_boxes = rospy.Publisher(
                "/output/bounding_boxes",
                BoundingBoxesStamped,
                queue_size=10,
            )
            self.pipeline = ThreeStageModel(
                yolo_model_path,
                keypoint_model_path,
                height_to_pos=self.height_to_pos,
                pub_bounding_boxes=self.pub_bounding_boxes,
                camera_matrix=self.camera_matrix,
                detection_height_threshold=self.detection_height_threshold,
                device=self.device,
                visualise=self.visualise,
            )

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[PERC] Camera",
                message=f"CUDA device used: {self.device}",
            )
        )

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.run_perception_pipeline()
            r.sleep()

    def get_image(self, ros_image: Image):
        self.ros_image = ros_image

    def run_perception_pipeline(self) -> None:
        """
        Given an image, run through the entire perception pipeline and publish to ROS

        Args:
            image: The input image as numpy array
            ros_image: The input image as ROS message
        """

        # If no image, do nothing
        ros_image = self.ros_image
        if ros_image is None:
            return
        self.ros_image = None

        # Wait for the pipeline to initialise
        if self.pipeline is None:
            return

        image = self.ros_img_to_np(ros_image)

        # The image should be an RGB Opencv array of HxWx3
        if self.use_two_stage:
            cones, latencies, vis_img = self.pipeline.predict(image)
        else:
            cones, latencies, vis_img = self.pipeline.predict(image, ros_image.header)

        # Publish the image
        msg = self.create_observation_msg(cones, ros_image.header)
        self.pub_cone_locations.publish(msg)

        # Publish disgnostics and visualisation
        timing_msg = f"[{np.sum(latencies):.0f}]"
        for t in latencies:
            timing_msg += f" {t:.1f}"
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[PERC] Pipeline latency",
                message=f"{timing_msg} ms",
            )
        )

        if self.visualise:
            self.pub_image_annotated.publish(
                self.np_to_ros_img(vis_img, ros_image.header.frame_id)
            )

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
            observation_class = int(cone[0])
            if (
                not rospy.get_param("~use_orange_cones", False)
                and observation_class == 2
            ):
                continue
            cone_positions.append(
                ObservationWithCovariance(
                    observation=Observation(
                        belief=1.0,
                        observation_class=int(cone[0]),
                        location=Point(*cone[1:]),
                    ),
                    covariance=[0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1],  # TODO: tweak
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

    def np_to_ros_img(self, arr: np.ndarray, frame) -> Image:
        """Creates a ROS image type based on a Numpy array
        Args:
            arr: numpy array in RGB format (H, W, 3), datatype uint8
        Returns:
            ROS Image with appropriate header and data
        """

        ros_img = Image(encoding="rgb8")
        ros_img.height, ros_img.width, _ = arr.shape
        contig = arr  # np.ascontiguousarray(arr)
        ros_img.data = contig.tobytes()
        ros_img.step = contig.strides[0]
        ros_img.is_bigendian = (
            arr.dtype.byteorder == ">"
            or arr.dtype.byteorder == "="
            and sys.byteorder == "big"
        )

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame
        ros_img.header = header

        return ros_img


if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
