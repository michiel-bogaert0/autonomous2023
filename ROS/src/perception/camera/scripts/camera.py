#! /usr/bin/python3.8

import os
from pathlib import Path

import cv2 as cv
import neoapi
import numpy as np
import rospy
from publisher_abstract.publisher import PublishNode


class CameraNode(PublishNode):
    def __init__(self):
        super().__init__("camera")
        self.setup_camera()

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

        camcal_location = rospy.get_param(
            "~camcal_location", "camera_calibration_baumer.npz"
        )
        camera_cal_archive = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camcal_location
        )
        self.camera_matrix = camera_cal_archive["camera_matrix"]
        self.distortion_matrix = camera_cal_archive["distortion_matrix"]
        self.optimal_camera_matrix, _ = cv.getOptimalNewCameraMatrix(
            self.camera_matrix, np.zeros(4), (1920, 1200), 1
        )

    def process_data(self):
        """
        reads the frames from the mp4 file

        returns:
            Ros Image to be published
        """

        image = self.camera.GetImage()

        if not image.IsEmpty():
            img = image.GetNPArray()

            img = cv.undistort(
                img,
                self.camera_matrix,
                self.distortion_matrix,
                None,
                self.optimal_camera_matrix,
            )

            # Convert image from BGR to RGB
            img = img[..., ::-1]
            ros_img = self.np_to_ros_image(img)

            return ros_img


node = CameraNode()
node.publish_image_data()
