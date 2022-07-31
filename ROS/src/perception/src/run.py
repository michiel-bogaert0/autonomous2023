#! /usr/bin/python3

import os
import sys
import time
from pathlib import Path
from typing import Tuple

from cone_detection.cone_detection import ConeDetector
from pnp.cone_pnp import ConePnp
from keypoint_detection.keypoint_detection import ConeKeypointDetector
import neoapi
import numpy as np
import rospy
import torch
from sensor_msgs.msg import Image
from ugr_msgs.msg import BoundingBox, ConeKeypoints, PerceptionUpdate


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception_jetson")
        self.pub_raw = rospy.Publisher("/perception/raw_image", Image, queue_size=10)
        self.pub_keypoints = rospy.Publisher(
            "/perception/processed/cone_keypoints", ConeKeypoints, queue_size=10
        )
        self.pub_pnp = rospy.Publisher(
            "/perception/processed/raw_perception_update", PerceptionUpdate, queue_size=10
        )

        self.rate = rospy.get_param("~rate", 10)

        self.tensorrt = rospy.get_param("~tensorrt",True)

        # Cone detection
        self.device = (
            "cuda:0"
            if torch.cuda.is_available() and rospy.get_param("~cuda", True)
            else "cpu"
        )
        self.cone_detector = ConeDetector(self.device,self.tensorrt)
        # The minimum height of a cone detection in px for it to be run through the keypoint detector
        self.detection_height_threshold = rospy.get_param(
            "~detection_height_threshold", 30
        )

        # Keypoint detection
        self.keypoint_detector = ConeKeypointDetector(self.device)
        self.publish_keypoints = rospy.get_param("~publish_keypoints", False)

        cone_dev = self.cone_detector.yolo_model.device
        keyp_dev = self.keypoint_detector.model.device
        rospy.logwarn(f"CUDA devices used: cone={cone_dev} - keypoint={keyp_dev}")

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

        self.setup_camera()

        self.generate_camera_data()

    def setup_camera(self) -> None:
        """Sets up the Baumer camera with the right settings
        Returns:
            Sets the self.camera object
        """

        # Init camera
        camera = neoapi.Cam()
        camera.Connect("700006709126")

        # Get the configuration saved on the camera itself
        selector = camera.GetFeature("UserSetSelector")
        selector.value = "UserSet1"

        user = camera.GetFeature("UserSetLoad")
        user.Execute()

        # Use continuous mode
        camera.f.TriggerMode.value = (
            neoapi.TriggerMode_Off
        )  # set camera to trigger mode, the camera starts streaming
        camera.f.AcquisitionFrameRateEnable = (
            True  # enable the frame rate control (optional)
        )
        camera.f.AcquisitionFrameRate = 24  # set the frame rate to 24 fps (optional)

        if camera.f.PixelFormat.GetEnumValueList().IsReadable("BGR8"):
            camera.f.PixelFormat.SetString("BGR8")

        # Limit the height of the output image
        height = 900
        camera.f.Height = height
        camera.f.OffsetY = 1200 - height

        self.camera = camera

    def np_to_ros_image(self, arr: np.ndarray) -> Image:
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

        ros_img.header.stamp = rospy.Time.now()
        ros_img.header.frame_id = "ugr/car_base_link/sensors/cam0"

        return ros_img

    def generate_camera_data(self):
        """This node publishes the frames from the camera at a fixed rate"""
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            # Grab frame
            image = self.camera.GetImage()

            if not image.IsEmpty():
                image = image.Convert("RGB8").GetNPArray()
                ros_img = self.np_to_ros_image(image)
                self.run_perception_pipeline(image, ros_img)
                self.pub_raw.publish(ros_img)

            rate.sleep()

    def run_perception_pipeline(self, image: np.ndarray, ros_image: Image) -> None:
        """
        Given an image, run through the entire perception pipeline and publish to ROS
        Args:
            image: The input image as numpy array
            ros_image: The input image as ROS message
        """
        timings = []

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

        rospy.loginfo(f"Timings {' - '.join([str(x) for x in timings])} s")

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