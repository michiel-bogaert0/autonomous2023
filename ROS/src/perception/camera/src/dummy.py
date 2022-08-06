#! /usr/bin/python3.8

import rospy
from pathlib import Path
import cv2 as cv
import os
from sensor_msgs.msg import Image


from publisher_abstract.publisher import PublishNode


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
        ros_img.data = contig.tobytes()
        ros_img.step = contig.strides[0]
        ros_img.is_bigendian = (
            arr.dtype.byteorder == ">"
            or arr.dtype.byteorder == "="
            and sys.byteorder == "big"
        )

    def process_data(self) -> Image:
        """
        reads the frames from the mp4 file

        returns:
            Ros Image to be published
        """
        if not self.cap.isOpened():
            print(f"Error opening video stream or file: {path}")

        # Capture frame-by-frame
        ret, frame = self.cap.read()

        if ret:
            # Publish the image as a ROS image
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            ros_img = self.np_to_ros_image(frame)

            return ros_img

        else:
            self.cap.set(cv.CAP_PROP_POS_FRAMES, 0)


node = DummyCamNode()
node.publish_image_data()
