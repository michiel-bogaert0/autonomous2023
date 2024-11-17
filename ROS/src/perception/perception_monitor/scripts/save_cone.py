#!/usr/bin/env python3

import os

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import NavSatFix


class CovarianceNode:
    def __init__(self):
        """
        Subscribes to two Observation topics, sets up a publisher and initializes parameters and variables
        """
        rospy.init_node("gps_cone_saver", anonymous=True)

        # Set up subscribers and publisher to receive/send observations
        rospy.Subscriber(
            "/input/cones",
            DiagnosticStatus,
            self.register_cone,
        )
        rospy.Subscriber(
            "/input/gps_locations",
            NavSatFix,
            self.get_cone_location,
        )

        self.confirmation_publisher = rospy.Publisher(
            "/output/topic", DiagnosticStatus, queue_size=1
        )

        # Initialize variables
        self.cones = list()
        self.gps_location = NavSatFix()

    def get_cone_location(self, msg):
        """
        Get the location of the cones
        """
        self.gps_location = msg

    def register_cone(self, msg):
        """
        Add cone to the list of cones
        """
        color = msg.name
        self.cones.append((self.gps_location, color))  # CHANGE FORMAT?

    def save_cones(self):
        """
        Saves the cones to a yaml file
        """

        with open(f"{os.path.dirname(__file__)}/../") as f:
            yaml.dump(self.cones, f, default_flow_style=False)
        msg = DiagnosticStatus()
        msg.level = 0
        self.confirmation_publisher.publish()


node = CovarianceNode()
rospy.spin()
