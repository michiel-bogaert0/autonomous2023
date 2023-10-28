#! /usr/bin/python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class ImageSaver:
    def __init__(self):
        rospy.init_node("image_saver")
        self.sub_save = rospy.Subscriber("/input/save", Empty, self.save_image)
        self.sub_img = rospy.Subscriber("/input/image", Image, self.receive_image)
        self.last_image = None

        self.bridge = CvBridge()

        rospy.spin()

    def save_image(self, ros_image: Image) -> None:
        """
        Save an image to the device.

        Args:
            ros_image: The input image as ROS message
        """
        if self.last_image is None:
            rospy.logerr("No image to save.")
            return

        img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")

        path = f"/home/ugr/autonomous2023/calib_images/{self.last_image.header.stamp}.png".replace(
            "-", "_"
        )
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
