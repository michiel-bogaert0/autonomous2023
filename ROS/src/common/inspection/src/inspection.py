#!/usr/bin/env python3
import math

import rospy
from node_fixture import AutonomousStatesEnum
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Float64, UInt16
from ugr_msgs.msg import State


class Inspection(ManagedNode):
    def __init__(self) -> None:
        """
        Inspection node
        """
        super().__init__("inspection", "active")
        rospy.loginfo("Inspection node started")

        self.spin()

    def doConfigure(self):
        """
        Configure the inspection node
        """
        rospy.loginfo("Configuring inspection node")
        self.STEERING_SPEED = rospy.get_param("~steering_speed", 0.05)
        self.VELOCITY = rospy.get_param("~velocity", 3.5)
        self.DURATION = rospy.get_param("~duration", 26)
        self.counter = 0

        self.steering_pub = rospy.Publisher("/output/steering", Float64)
        self.velocity0_pub = rospy.Publisher("/output/velocity0", Float64)
        self.velocity1_pub = rospy.Publisher("/output/velocity1", Float64)
        self.lap_complete_pub = rospy.Publisher(
            "/output/lapComplete", UInt16, queue_size=5, latch=True
        )

        self.state_as_sub = rospy.Subscriber(
            "/state/as", State, self.handle_state_change
        )

        self.start = False

    def handle_state_change(self, msg: State):
        if msg.cur_state == AutonomousStatesEnum.ASDRIVE:
            # Start inspection
            rospy.loginfo("Starting inspection")
            self.stop_time = rospy.Time.now() + rospy.Duration(self.DURATION)
            self.start = True

    def active(self):
        """
        Inspection node is active
        """
        if not self.start:
            return

        if rospy.Time.now() > self.stop_time:
            # Stop inspection
            self.velocity_pub.publish(0)
            self.lap_complete_pub.publish(1)
        else:
            # Perform inspection
            steering_angle = ((7 * math.pi) / 9) * math.sin(
                self.counter * self.STEERING_SPEED
            )

            self.steering_pub.publish(steering_angle)
            self.velocity_pub0.publish(self.VELOCITY)
            self.velocity_pub1.publish(self.VELOCITY)

            self.counter += 1
            rospy.loginfo(
                f"Steering angle: {steering_angle}, velocity: {self.VELOCITY}"
            )


node = Inspection()
