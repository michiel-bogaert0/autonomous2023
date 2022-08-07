from abc import ABC, abstractmethod
from typing import Any

import rospy
from node_fixture.node_fixture import ROSNode, AddSubscriber
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import numpy as np


"""
A perception publisher should have the following structure:
option 1: with a predefined rate and a publish function -> call the publish_image_data function 
option 2: subscribes to a topic that publishes raw data -> start the child rosnode (by calling node.start())
"""


class PublishNode(ROSNode, ABC):
    def __init__(self, name):
        super().__init__(name, False)
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.frame = (
            f"ugr/car_base_link/sensors/{rospy.get_param('~sensor_name','cam0')}"
        )

    @abstractmethod
    def process_data(self) -> Image:
        """
        any child of this class should have a process_data function

        returns:
            a ros image (header not required)
        """
        pass

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

    @AddSubscriber("raw/input")
    def publish_sub_data(self, data: Any):
        """
        simple wrapper function for uniformity reasons, used to connect to fsds sim
        """
        self.publish("/input/image", data)

    def publish_image_data(self):
        """
        main function that publishes raw image data,
        uses the process data implemented in the child node
        """
        if self.rate is not None:
            while not rospy.is_shutdown():
                data = self.process_data()

                if data is not None:
                    data.header = self.create_header()
                    self.publish("/input/image", data)
               
                   
                self.rate.sleep()

    def create_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame
        return header
