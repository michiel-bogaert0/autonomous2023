#!/usr/bin/env python3

import os

# from nav_msgs.msg import Odometry (If Odometry)
from datetime import datetime

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
            "/input/save_cone_color",
            DiagnosticStatus,
            self.register_cone,
        )
        rospy.Subscriber(
            "/input/gps_locations",
            NavSatFix,
            self.get_cone_location,
        )
        # If Odometry
        # rospy.Subscriber(
        #     "/input/gps_locations_odom",
        #     Odometry,
        #     self.get_cone_location,
        # )

        self.confirmation_publisher = rospy.Publisher(
            "/output/topic", DiagnosticStatus, queue_size=1
        )

        # Initialize variables
        self.yaml_file = f"{os.path.dirname(__file__)}/../../../slam/slam_simulator/maps/gps_maps/{datetime.now().strftime('%d-%m-%Y')}(1).yaml"
        self.cones = list()
        self.gps_location = NavSatFix()  # Odometry() (If Odometry)

    def get_cone_location(self, msg):
        """
        Get the location of the cones
        """
        rospy.loginfo("Received GPS location!")
        self.gps_location = msg

    def register_cone(self, msg):
        """
        Add cone to the list of cones
        """
        color_class = int(msg.values[0].value)
        if color_class == -5:
            self.save_cones()
            return

        # If NavSatFix
        cone_dict = {
            "location": {
                "latitude": self.gps_location.latitude,
                "longitude": self.gps_location.longitude,
                "altitude": self.gps_location.altitude,
            },
            "color": color_class,
        }
        # If Odometry
        # cone_dict = {
        #     "location": {
        #         "x": self.gps_location.pose.pose.position.x,
        #         "y": self.gps_location.pose.pose.position.y,
        #         "z": self.gps_location.pose.pose.position.z,
        #     },
        #     "color": color_class,
        # }
        self.cones.append(cone_dict)
        confirmation_msg = DiagnosticStatus()
        confirmation_msg.level = 0
        confirmation_msg.message = (
            f"Registered cone with color {color_class} at {datetime.now()}"
        )
        self.confirmation_publisher.publish(confirmation_msg)

    def save_cones(self):
        """
        Saves the cones to a yaml file
        """

        rospy.loginfo("Saving cones to file...")
        with open(self.yaml_file, "w") as f:
            yaml.dump(self.cones, f, default_flow_style=False)
        rospy.loginfo(f"Saved cones to file {self.yaml_file}")
        confirmation_msg = DiagnosticStatus()
        confirmation_msg.level = 0
        confirmation_msg.message = f"Saved cones to file {self.yaml_file}"
        self.confirmation_publisher.publish(confirmation_msg)


node = CovarianceNode()
rospy.spin()
