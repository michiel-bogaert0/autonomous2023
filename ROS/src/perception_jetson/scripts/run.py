#! /usr/bin/python3.8

import time
import numpy as np
import neoapi
import sys

import cone_detection
import torch
import rospy
from sensor_msgs.msg import Image


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception_jetson")
        self.pub = rospy.Publisher("/perception/raw_image", Image, queue_size=10)

        self.rate = rospy.get_param("~rate", 10)

        # Cone detection
        self.device = "cuda" if torch.cuda.is_available() and rospy.get_param("~cuda", True) else "cpu"
        self.cone_detector = cones.ConeDetector(self.device)

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
        ros_img.data = contig.tostring()
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
                image = image.Convert("RGB8")
                ros_img = self.np_to_ros_image(image.GetNPArray())
                self.run_perception_pipeline(image)
                self.pub.publish(ros_img)

            rate.sleep()

    def run_perception_pipeline(self, image: np.ndarray) -> None:
        """
        Given an image, run through the entire perception pipeline and publish to ROS

        Args:
            image: The input image
        """

        tic = time.perf_counter()
        bbs = self.cone_detector.detect_cones(image)
        toc = time.perf_counter()

        keypoints = []

        # Don't count detections that aren't tall enough
        h, w, c = image.shape
        detection_height_threshold = self.detection_height_threshold / h

        # Filter bbs by height and if they are taller than they are wide
        bbs = [bb for bb in bbs if bb.height > bb.width and bb.height > detection_height_threshold]

        if len(bbs) != 0:
            # There were bounding boxes detected
            keypoints = self.keypoint_detector.detect_keypoints(image, bbs)
            keypoints_time = time.time()

            self.cone_timing.append(bbs_time - start_time)
            self.keypoint_timing.append(keypoints_time - bbs_time)
            cone_av = sum(self.cone_timing) / len(self.cone_timing)
            keypoint_av = sum(self.keypoint_timing) / len(self.keypoint_timing)
            rospy.loginfo(f"{cone_av:.3f} - {keypoint_av:.3f}")

            cone_keypoints_msg = ConeKeypoints()
            cone_keypoints_msg.cone_keypoints = keypoints
            cone_keypoints_msg.img = msg
            cone_keypoints_msg.header.stamp = msg.header.stamp
            cone_keypoints_msg.header.frame_id = msg.header.frame_id
            self.publish(
                "/processed/cone_keypoints",
                cone_keypoints_msg,
            )

        if self.vis:
            self.visualisation_callback(image, bbs, keypoints)

if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
