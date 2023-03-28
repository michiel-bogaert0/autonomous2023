#!/usr/bin/python3
import enum
import queue
from collections import deque
from functools import partial
from pathlib import Path
from typing import Any, Tuple, Type, get_type_hints

import numpy as np
import PyKDL
import roslib.message
import rospy
import rostopic
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fs_msgs.msg import Cone
from geometry_msgs.msg import TransformStamped
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE, get_tcpros_handler
from rospy.rostime import Time
from std_msgs.msg import String
from tf2_kdl import transform_to_kdl
from ugr_msgs.msg import (Map, ObservationWithCovariance,
                          ObservationWithCovarianceArrayStamped)

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

        rospy.init_node(self.name, anonymous=True)

        if already_add_subscriber:
            self.add_subscribers()

        self.publishers = {}

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
                obs.observation.location.x, obs.observation.location.y, obs.observation.location.z
            )

            new_observation = ObservationWithCovariance()
            new_observation.observation.location.x = p[0]
            new_observation.observation.location.y = p[1]
            new_observation.observation.location.z = p[2]
            new_observation.observation.observation_class = (
                obs.observation.observation_class
            )
            new_observation.observation.belief = obs.observation.belief

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

##
## See https://ugentracing.sharepoint.com/sites/UGRWiki/SitePages/State-machine-conventions.aspx
##

# All scopes of state machines in autonomous
class StateMachineScopeEnum(str, enum.Enum):
    AUTONOMOUS = "autonomous"
    SLAM = "slam"

# States of SLAM state machine
class SLAMStatesEnum(str, enum.Enum):
    IDLE = "idle"
    EXPLORATION = "exploration"
    RACING = "racing"
    FINISHED = "finished"

# Autonomous missions
class AutonomousMission(str, enum.Enum):
    MANUAL = "manual"
    AUTOCROSS = "autocross"
    ACCELERATION = "acceleration"
    TRACKDRIVE = "trackdrive"
    EBSTEST = "emergencybraketest"
    SKIDPAD = "skidpad"
    
# States of global state machine
class AutonomousStatesEnum(str, enum.Enum):
    ASDRIVE = "asdrive"
    ASREADY = "asready"
    MANUAL = "manual"
    ASOFF = "asoff"
    ASEMERGENCY = "asemergency"
    ASFINISHING = "asfinishing"
    ASFINISHED = "asfinished"

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
