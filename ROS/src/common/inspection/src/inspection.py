#!/usr/bin/env python3
import math

import rospy
from node_fixture.node_manager import NodeManager
from std_msgs.msg import Float64


class Inspection(NodeManager):
    def __init__(self) -> None:
        """
        Inspection node
        """
        super().__init__("inspection")
        rospy.loginfo("Inspection node started")

        self.spin()

    def doConfigure(self):
        """
        Configure the inspection node
        """
        rospy.loginfo("Configuring inspection node")
        self.STEERING_SPEED = rospy.get_param("~steering_speed", 0.05)
        self.VELOCITY = rospy.get_param("~velocity", 3.5)
        self.counter = 0

        self.steering_pub = rospy.Publisher("/output/steering", Float64)
        self.velocity_pub = rospy.Publisher("/output/velocity", Float64)

    def active(self):
        """
        Inspection node is active
        """
        steering_angle = math.pi / 2 * math.sin(self.counter * self.STEERING_SPEED)

        self.steering_pub.publish(steering_angle)
        self.velocity_pub.publish(self.VELOCITY)

        self.counter += 1

        rospy.loginfo(f"Steering angle: {steering_angle}, velocity: {self.VELOCITY}")


node = Inspection()
