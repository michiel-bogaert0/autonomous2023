#!/usr/bin/env python3
# import numpy as np
import rospy
import tf2_ros as tf

# from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Boundaries, Path
from node_fixture.fixture import (  # DiagnosticStatus,; NodeManagingStatesEnum,; ROSNode,; create_diagnostic_message,
    DiagnosticArray,
    SLAMStatesEnum,
)
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Float64
from ugr_msgs.msg import State


class MinimumCurvature(ManagedNode):
    def __init__(self):
        rospy.init_node("minimum_curvature")
        super().__init__("minimum_curvature")
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.slam_state = SLAMStatesEnum.IDLE
        rospy.Subscriber("/state", State, self.handle_state_change)
        self.start_sender()
        rospy.spin()

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.car_width = rospy.get_param("~car_width", 1.0)

        # Publishers for the path and velocity

        self.path_pub = super().AddPublisher("/output/path", Path, queue_size=10)
        self.velocity_pub = super().AddPublisher(
            "/output/velocity", Float64, queue_size=10
        )

        # Diagnostic publisher
        self.diagnostic_pub = super().AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Subscribers for the boundaries
        self.boundaries_sub = super().AddSubscriber(
            "/input/boundaries", Boundaries, self.receive_boundaries
        )

    def doActivate(self):
        pass


node = MinimumCurvature()
