#! /usr/bin/python3.8

import os
from pathlib import Path

import cv2 as cv
import neoapi
import numpy as np
import rospy
import yaml
from publisher_abstract.publisher import PublishNode
from sensor_msgs.msg import CameraInfo


class CameraNode(PublishNode):
    def __init__(self):
        super().__init__("camera")

        # Whether or not to undistort the images or use them raw, as is
        # Unless you know what you're doing, don't enable use_raw
        self.use_raw = rospy.get_param("~use_raw", False)

        camsettings_location = rospy.get_param("~camsettings", "camera_settings.yaml")
        with open(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camsettings_location, "r"
        ) as stream:
            self.camera_settings = yaml.safe_load(stream)

        self.setup_camera()

    def get_camera_info(self):
        msg = CameraInfo()
        msg.height = self.camera_settings["height"]
        msg.width = self.camera_settings["width"]

        if self.use_raw:
            msg.D = self.distortion_matrix.flatten().tolist()
            msg.K = self.camera_matrix.flatten().tolist()
            msg.R = np.eye(3).flatten().tolist()
            msg.P = np.hstack((self.camera_matrix, np.zeros((3, 1)))).flatten().tolist()

        else:
            msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.K = self.optimal_camera_matrix.flatten().tolist()
            msg.R = np.eye(3).flatten().tolist()
            msg.P = (
                np.hstack((self.optimal_camera_matrix, np.zeros((3, 1))))
                .flatten()
                .tolist()
            )
        return msg

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

        self.camera = camera

        # Calibration parameters
        camcal_location = rospy.get_param(
            "~camcal_location", "camera_calibration_baumer.npz"
        )
        camera_cal_archive = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camcal_location
        )
        self.camera_matrix = camera_cal_archive["camera_matrix"]
        self.distortion_matrix = camera_cal_archive["distortion_matrix"]
        self.optimal_camera_matrix, _ = cv.getOptimalNewCameraMatrix(
            self.camera_matrix,
            np.zeros(4),
            (self.camera_settings["width"], self.camera_settings["height"]),
            1,
        )

        self.maps = cv.initUndistortRectifyMap(
            self.camera_matrix,
            self.distortion_matrix,
            None,
            self.optimal_camera_matrix,
            (self.camera_settings["width"], self.camera_settings["height"]),
            cv.CV_32FC1,
        )

    def process_data(self):
        """
        reads the frames from the camera

        returns:
            Ros Image to be published
        """
        image = self.camera.GetImage()

        if not image.IsEmpty():
            img = image.GetNPArray()

            if not self.use_raw:
                img = cv.remap(
                    img,
                    *self.maps,
                    cv.INTER_LINEAR,
                )

            # Convert image from BGR to RGB
            img = img[..., ::-1]
            ros_img = self.np_to_ros_image(img)

            return ros_img


node = CameraNode()
node.publish_image_data()
