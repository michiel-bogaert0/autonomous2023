#! /usr/bin/python3.8

import cv2 as cv
import neoapi
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

        camera.f.ExposureAuto.Set(False)
        print(camera.f.ExposureAuto.Set(neoapi.ExposureMode_Timed))
        camera.f.ExposureTime.value = 2000000

        if camera.f.PixelFormat.GetEnumValueList().IsReadable("BGR8"):
            camera.f.PixelFormat.SetString("BGR8")

        self.camera = camera

    def process_data(self):
        """
        reads the frames from the mp4 file

        returns:
            Ros Image to be published
        """

        image = self.camera.GetImage()

        if not image.IsEmpty():
            img = image.GetNPArray()[..., ::-1]  # BGR to RGB

            ros_img = self.np_to_ros_image(img)

            return ros_img


node = CameraNode()
node.publish_image_data()
