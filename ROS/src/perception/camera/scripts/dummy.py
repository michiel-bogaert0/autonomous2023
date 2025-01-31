#! /usr/bin/python3.8

import os
from pathlib import Path

import cv2 as cv
import rospy
from publisher_abstract.publisher import PublishNode
from sensor_msgs.msg import Image


class DummyCamNode(PublishNode):
    """
    generates dummy data based on an input path
    """

    def __init__(self):
        super().__init__("dummy")
        self.input = rospy.get_param("~file_path", "track_walk.mp4")
        self.path = (
            Path(os.getenv("BINARY_LOCATION"))
            / "dummy_data"
            / "visual_pipeline"
            / self.input
        )
        self.cap = cv.VideoCapture(str(self.path))

    def process_data(self) -> Image:
        """
        reads the frames from the mp4 file

        returns:
            Ros Image to be published
        """

        if not self.cap.isOpened():
            print(f"Error opening video stream or file: {self.path}")

        # Capture frame-by-frame
        ret, frame = self.cap.read()

        if ret:
            # Publish the image as a ROS image
            frame = frame[..., ::-1]  # BGR to RGB
            ros_img = self.np_to_ros_image(frame)

            return ros_img

        else:
            self.cap.set(cv.CAP_PROP_POS_FRAMES, 0)


node = DummyCamNode()
node.publish_image_data()
