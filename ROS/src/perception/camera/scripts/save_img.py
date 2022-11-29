#! /usr/bin/python3
import sys
import cv2
import numpy as np
import rospy
import torch
from tools.tools import np_to_ros_image, ros_img_to_np
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

class ImageSaver:
    def __init__(self):
        rospy.init_node("image_saver")
        self.sub_save = rospy.Subscriber(
            "/input/save", Empty, self.save_image
        )
        self.sub_img = rospy.Subscriber(
            "/input/image", Image, self.receive_image
        )
        self.last_image = None

        rospy.spin()


    def save_image(self, ros_image: Image) -> None:
        """
        Save an image to the device.

        Args:
            ros_image: The input image as ROS message
        """
        if self.last_image is None:
            rospy.logerr(f"No image to save.")
            return

        img = ros_img_to_np(image=self.last_image)[:, :, ::-1]

        path = f"/home/ugentracing/{self.last_image.header.stamp}.png"
        cv2.imwrite(path, img)
        rospy.loginfo(f"Save image to {path}")

    def receive_image(self, ros_image: Image) -> None:
        """
        Save the latest image internally for future writing.

        Args:
            ros_image: The input image as ROS message
        """

        self.last_image = ros_image


if __name__ == "__main__":
    try:
        img_saver = ImageSaver()
    except rospy.ROSInterruptException:
        pass
