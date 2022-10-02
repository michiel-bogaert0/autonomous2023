#!/usr/bin/python3
import queue
from collections import deque
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, Tuple, Type, get_type_hints

import enum

import cv2
import numpy as np
import PyKDL
import roslib.message
import rospy
import rostopic
import torch
from cv_bridge import CvBridge
from fs_msgs.msg import Cone
from geometry_msgs.msg import TransformStamped
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE, get_tcpros_handler
from rospy.rostime import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_kdl import transform_to_kdl
from ugr_msgs.msg import BoundingBox, Map, ObservationWithCovariance, ObservationWithCovarianceArrayStamped, PerceptionUpdate


@dataclass
class IntBoundingBox:
    left: int
    top: int
    width: int
    height: int
    right: int
    bottom: int

    tl: Tuple[int, int] = (0, 0)
    br: Tuple[int, int] = (0, 0)

    def __post_init__(self):
        self.tl = (self.left, self.top)
        self.br = (self.right, self.bottom)

    @staticmethod
    def from_img(or_bb: BoundingBox, img: np.ndarray):
        h, w, c = img.shape
        bb_left = int(w * or_bb.left)
        bb_top = int(h * or_bb.top)
        bb_w = int(w * or_bb.width)
        bb_h = int(h * or_bb.height)
        bb_right = bb_left + bb_w
        bb_bottom = bb_top + bb_h

        return IntBoundingBox(
            left=bb_left,
            top=bb_top,
            width=bb_w,
            height=bb_h,
            right=bb_right,
            bottom=bb_bottom,
        )


class AddSubscriber:
    functions_to_subscribe = {}

    def __init__(self, topic, queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
        self.topic = topic
        self.queue_size = queue_size
        self.buff_size = buff_size

    def __call__(self, func):
        AddSubscriber.functions_to_subscribe[self.topic] = (
            func,
            self.queue_size,
            self.buff_size,
        )
        return func


class DataLatch:
    """
    This class is a useful tool for when you need to save data from topics or from anywhere.
    It supported both deques and just normal values.
    """

    def __init__(self) -> None:
        self.latch = {}
        self.is_deque = {}

    def create(self, topic, length):
        """
        Creates a new entry in the latch

        Args:
          topic: the name of the item in the latch
          length: if equal to 1, it just saves a single value, if bigger than 1, it uses a deque
        """
        if topic in self.latch:
            return

        if length > 1:
            self.is_deque[topic] = True
            self.latch[topic] = deque([], length)
        else:
            self.is_deque[topic] = False
            self.latch[topic] = None

    def set(self, topic, value):
        """
        Adds a value to the latch in topic

        Args:
          topic: the place in the latch to store the value
          value: the object to store. Can be pushed to a deque in some cases.
        """
        if topic not in self.latch:
            return

        if not self.is_deque[topic]:
            self.latch[topic] = value
        else:
            self.latch[topic].append(value)

    def get(self, topic):
        """
        Returns the value in the latch at a certain topic

        Args:
          topic: the topic to return data from

        Returns:
          The deque or a single value (single value could be None), depending on the latch type (see self.create)
          When topic not found, returns None
        """
        if topic in self.latch:
            return self.latch[topic]
        else:
            return None


class ROSNode:
    def __init__(self, name: str, already_add_subscriber: bool = True):
        self.name = name

        self.bridge = CvBridge()
        rospy.init_node(self.name, anonymous=True)

        if already_add_subscriber:
            self.add_subscribers()

        self.publishers = {}

    @staticmethod
    def do_transform_perception_update(
        perception_update: PerceptionUpdate, transform: TransformStamped
    ) -> PerceptionUpdate:
        """
        Custom transformation method to apply a transformation to a perception update
        Args:
          - perception_update: PerceptionUpdate = the message to transform
          - transform: TransformStamped = the transformation to apply

        Returns:
          A transformed PerceptionUpdate
        """
        kdl_transform = transform_to_kdl(transform)
        res = PerceptionUpdate()
        for cone in perception_update.cone_relative_positions:
            p = kdl_transform * PyKDL.Vector(
                cone.location.x, cone.location.y, cone.location.z
            )
            new_cone = Cone()
            new_cone.color = cone.color
            new_cone.location.x = p[0]
            new_cone.location.y = p[1]
            new_cone.location.z = p[2]
            res.cone_relative_positions.append(new_cone)

        res.header = transform.header
        return res

    @staticmethod
    def do_transform_observations(
        observations: ObservationWithCovarianceArrayStamped, transform: TransformStamped
    ) -> ObservationWithCovarianceArrayStamped:
        """
        Custom transformation method to apply a transformation to a the ObservationWithCovarianceArrayStamped message
        Args:
          - observations: ObservationWithCovarianceArrayStamped = the message to transform
          - transform: TransformStamped = the transformation to apply

        Returns:
          A transformed ObservationWithCovarianceArrayStamped message
        """
        kdl_transform = transform_to_kdl(transform)
        res = ObservationWithCovarianceArrayStamped()
        for obs in observations.observations:
            p = kdl_transform * PyKDL.Vector(
                obs.location.x, obs.location.y, obs.location.z
            )

            new_observation = ObservationWithCovariance()
            new_observation.location.x = p[0]
            new_observation.location.y = p[1]
            new_observation.location.z = p[2]
            new_observation.observation_class = obs.observation_class

            res.observations.append(new_observation)

        res.header = transform.header
        return res

    @staticmethod
    def do_transform_map(map: Map, transform) -> Map:
        """
        Custom transformation method to apply a transformation to a Map message
        Args:
          - map: Map = the message to transform
          - transform: TransformStamped = the transformation to apply

        Returns:
          A transformed Map
        """
        kdl_transform = transform_to_kdl(transform)
        res = Map()
        for cone in map.cone_positions:
            p = kdl_transform * PyKDL.Vector(
                cone.location.x, cone.location.y, cone.location.z
            )
            new_cone = Cone()
            new_cone.color = cone.color
            new_cone.location.x = p[0]
            new_cone.location.y = p[1]
            new_cone.location.z = p[2]
            res.cone_positions.append(new_cone)

        res.header = transform.header
        return res

    def add_subscribers(self):
        for topic, data in AddSubscriber.functions_to_subscribe.items():
            callback, queue_size, buff_size = data

            topic_msg_class = None
            try:
                type_hints = get_type_hints(callback)
                topic_msg_class = list(type_hints.values())[-1]
            except Exception as e:
                topic_msg_type, _, fn = rostopic.get_topic_type(
                    f"/{topic.strip('/')}", True
                )  #
                topic_msg_class = roslib.message.get_message_class(topic_msg_type)

            rospy.Subscriber(
                topic,
                topic_msg_class,
                partial(callback, self),
                queue_size=queue_size,
                buff_size=buff_size,
            )

    def to_int_bb(self, or_bb: BoundingBox, img: np.ndarray) -> IntBoundingBox:
        return IntBoundingBox.from_img(or_bb, img)

    def to_channel_in_front(self, img: np.ndarray):
        return np.moveaxis(img, -1, 0)

    def to_channel_in_back(self, img: np.ndarray):
        return np.moveaxis(img, 0, -1)

    @AddSubscriber(topic="state")
    def state_callback(self, data: String):
        self.state_update(str(data))

    def state_update(self, new_state: str):
        pass

    def ros_img_to_cv_img(self, img: Image):
        return self.bridge.imgmsg_to_cv2(img, "rgb8")

    def cv_img_to_ros_img(self, img: np.array):
        return self.bridge.cv2_to_imgmsg(img, "rgb8")

    def pytorch_ros_prepare(self, img: Image, img_size: Tuple[int, int]):
        # img_size is (w,h)
        cv_img = self.ros_img_to_cv_img(img)
        return self.pytorch_cv_prepare(cv_img, img_size)

    def pytorch_cv_prepare(self, img: np.ndarray, img_size: Tuple[int, int]):
        img = cv2.resize(img, img_size)
        img = self.to_channel_in_front(img)
        img = img.astype(np.float32)
        img /= 255
        return torch.FloatTensor(img)

    def get_publisher(self, topic: str, msg_type: Type):
        key = (topic, msg_type)
        if key not in self.publishers:
            publisher = rospy.Publisher(topic, msg_type, queue_size=10)
            self.publishers[(topic, msg_type)] = publisher

        publisher = self.publishers[(topic, msg_type)]

        return publisher

    def publish(self, topic: str, data: Any):
        publisher = self.get_publisher(topic, type(data))
        publisher.publish(data)

    def get_data_folder(self):
        return Path(__file__).parent.parent / "data"

    def start(self):
        rospy.spin()

    @staticmethod
    def convertROSStampToTimestamp(rosstamp: Time) -> int:
        """
        Converts a ROS stamp object to a float number
        Basically combines secs and nsecs properties together
        """
        return int(rosstamp.__str__()) / 10**9

class DiagnosticStatusEnum(enum.Enum):
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3

def create_diagnostic_message(
    level: DiagnosticStatusEnum,
    name: String,
    message: String,
) -> DiagnosticArray:
    """Creates a DiagnosticArray message with a level, name and message

    Args:
        As defined in the DiagnosticArray

    Returns:
        A DiagnosticArray message
    """
    diag_status = DiagnosticStatus()
    diag_status.level = level
    diag_status.name = name
    diag_status.message = message
    diag_array = DiagnosticArray()
    diag_array.status.append(diag_status)

    return diag_array
