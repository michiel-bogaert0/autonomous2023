#! /usr/bin/python3
import sys
import cv2
import numpy as np
import rospy
import torch
from tools.tools import np_to_ros_image, ros_img_to_np
from sensor_msgs.msg import Image

class ImageSaver:
    def __init__(self):
        rospy.init_node("image_saver")
        self.sub_img = rospy.Subscriber(
            "/input/image", Image, self.save_image
        )
        rospy.spin()

    def save_image(self, ros_image: Image) -> None:
        """
        Save an image.

        Args:
            ros_image: The input image as ROS message
        """

        image = ros_img_to_np(image=ros_image)[:, :, ::-1]

        cv2.imwrite("/home/ugentracing/cone_9m.png", image)

        rospy.signal_shutdown("Image saved")


if __name__ == "__main__":
    try:
        img_saver = ImageSaver()
    except rospy.ROSInterruptException:
        pass
