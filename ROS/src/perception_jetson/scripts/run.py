#! /usr/bin/python3.8

import numpy as np
import neoapi
import sys
import rospy
from sensor_msgs.msg import Image


class PerceptionNode:
    def __init__(self):
        rospy.init_node("perception_jetson")
        self.pub = rospy.Publisher("/perception/raw_image", Image, queue_size=10)

        self.rate = rospy.get_param("~rate", 10)

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
                image.Convert("RGB8")
                ros_img = self.np_to_ros_image(image.GetNPArray())
                self.pub.publish(ros_img)

            rate.sleep()


if __name__ == "__main__":
    try:
        pn = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
