#! /usr/bin/python3.8

import os
from pathlib import Path

import cv2 as cv
import yaml
import neoapi
import numpy as np
import rospy
from publisher_abstract.publisher import PublishNode
from sensor_msgs.msg import CameraInfo
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture.node_fixture import create_diagnostic_message


class CameraNode(PublishNode):
    def __init__(self):
        super().__init__("camera")

        # Whether or not to undistort the images or use them raw, as is
        # Unless you know what you're doing, don't enable use_raw
        self.use_raw = rospy.get_param("~use_raw", False)

        # Auto-brightness parameters
        self.target_brightness = rospy.get_param("~target_brightness", 70)
        self.brightness_deadzone = rospy.get_param("~brightness_deadzone", 3)
        self.max_exposure = rospy.get_param("~max_exposure", 300_000)
        self.min_gain = rospy.get_param("~min_gain", 1)
        self.max_gain = rospy.get_param("~max_gain", 10)
        self.stepsize_gain = rospy.get_param("~stepsize_gain", 0.1)
        self.stepsize_exposure = rospy.get_param("~stepsize_exposure", 0.1)
        self.camera_control_interval = rospy.get_param("~camera_control_interval", 50)

        self.exposure = 100_000
        self.gain = 1
        self.last_control = self.camera_control_interval

        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        camsettings_location = rospy.get_param("~camsettings", "camera_settings.yaml")
        with open(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camsettings_location, "r"
        ) as stream:
            self.camera_settings = yaml.safe_load(stream)

        self.setup_camera()

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.WARN,
                name="[HW] Camera driver",
                message=f"Brightness - Exp/Gain: INIT - {self.exposure}/{self.gain:.2f}",
            )
        )

    def get_camera_info(self):
        msg = CameraInfo()
        msg.height = self.camera_settings["height"]
        msg.width = self.camera_settings["width"]

        if self.use_raw:
            msg.D = self.distortion_matrix.flatten().tolist()
            msg.K = self.camera_matrix.flatten().tolist()
            msg.P = np.hstack((self.camera_matrix, np.zeros(3, 1))).flatten().list()

        else:
            msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.K = self.optimal_camera_matrix.flatten().tolist()
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

        # Use continuous mode
        camera.f.TriggerMode.value = (
            neoapi.TriggerMode_Off
        )  # set camera to trigger mode, the camera starts streaming
        if camera.f.PixelFormat.GetEnumValueList().IsReadable("BGR8"):
            camera.f.PixelFormat.SetString("BGR8")

        # Setting all the needed parameters to manually change exposure and gain
        camera.f.Height = self.camera_settings["height"]
        camera.f.Width = self.camera_settings["width"]
        camera.f.ExposureMode.Set(neoapi.ExposureMode_Timed)
        camera.f.ExposureAuto.Set(neoapi.ExposureAuto_Off)
        camera.f.BalanceWhiteAuto.Set(neoapi.BalanceWhiteAuto_Off)
        camera.f.GainSelector.Set(neoapi.GainSelector_All)
        camera.f.GainAuto.Set(neoapi.GainAuto_Off)
        camera.f.SequencerConfigurationMode.Set(neoapi.SequencerConfigurationMode_Off)
        camera.f.ColorTransformationAuto.Set(neoapi.ColorTransformationAuto_Off)
        camera.f.SequencerMode.Set(neoapi.SequencerMode_Off)
        camera.f.SequencerMode.Set(neoapi.SequencerMode_Off)
        camera.f.SequencerMode.Set(neoapi.SequencerMode_Off)

        # Some default values
        camera.f.ExposureTime.Set(self.exposure)
        camera.f.Gain.Set(self.gain)

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

    def update_camera_params(self, img):
        # Get a measure of the perceived image brightness
        # Formula: 0.2126*R + 0.7152*G + 0.0722*B
        # Remember that the image is still BGR here
        means = np.mean(img, axis=(0, 1))
        perceived_brightness = 0.2126 * means[2] + 0.7152 * means[1] + 0.0722 * means[0]

        # Tweak the camera parameters
        err = perceived_brightness - self.target_brightness
        if err > self.brightness_deadzone:
            # The image is too bright
            if self.gain == self.min_gain:
                self.exposure -= self.stepsize_exposure * self.exposure
            else:
                self.gain -= self.stepsize_gain * self.gain
                self.gain = max(self.gain, self.min_gain)
        elif -err > self.brightness_deadzone:
            # The image is too dim
            if self.gain == self.max_gain:
                self.exposure += self.stepsize_exposure * self.exposure
            else:
                self.gain += self.stepsize_gain * self.gain
                self.gain = min(self.gain, self.max_gain)

        self.camera.f.ExposureTime.Set(self.exposure)
        self.camera.f.Gain.Set(self.gain)

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.WARN
                if abs(err) > self.brightness_deadzone
                else DiagnosticStatus.OK,
                name="[HW] Camera driver",
                message=f"Brightness - Exp/Gain: {perceived_brightness:.1f} - {self.exposure}/{self.gain:.2f}",
            )
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

            self.last_control -= 1
            if self.last_control < 0:
                self.last_control = self.camera_control_interval
                self.update_camera_params(img)

            if not self.use_raw:
                img = cv.remap(
                    img,
                    *self.maps,
                    cv.INTER_LINEAR,
                )

            # Convert image from BGR to RGB
            img = img[..., ::-1]
            ros_img = self.np_to_ros_image(img)

            # print("E: ", self.camera.f.ExposureTime.Get())
            # print("G: ", self.camera.f.Gain.Get())
            # print("B: ", img.mean())
            return ros_img


node = CameraNode()
node.publish_image_data()
