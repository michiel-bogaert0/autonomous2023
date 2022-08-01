#!/usr/bin/env python3

import numpy as np
import rospy
import can
import cantools
import struct
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistWithCovariance, Twist

class CanConverter:
    def __init__(self):
        rospy.init_node("can_node")
        self.can_pub = rospy.Publisher("/can_messages", String, queue_size=10)
        self.vel_left = rospy.Publisher(
            "/mechatronics/wheel_encoder/left",
            TwistWithCovarianceStamped,
            queue_size=10,
        )
        self.vel_right = rospy.Publisher(
            "/mechatronics/wheel_encoder/right",
            TwistWithCovarianceStamped,
            queue_size=10,
        )

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 8 * 2.54 / 100)  # in m
        self.axis0_frame = rospy.get_param("~axis0/frame", "ugr/car_axis0")
        self.axis1_frame = rospy.get_param("~axis1/frame", "ugr/car_axis1")

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 1000000),
        )

        self.odrive_db = cantools.database.load_file(rospy.get_param("~odrive_dbc", "../odrive.dbc"))

        try:
            self.listen_on_can()
        except rospy.ROSInterruptException:
            pass

    def listen_on_can(self) -> None:
        """Listens on the can interface of the Jetson and publishes the messages"""

        # iterate over received messages
        # this keeps looping forever
        for msg in self.bus:
            # Publish message on ROS

            # Check if the message is a ODrive command
            axis_id = msg.arbitration_id >> 5
            if axis_id == 1 or axis_id == 2:
                cmd_id = msg.arbitration_id & 0b11111

                if cmd_id == 9:

                    can_msg = self.odrive_db.decode_message(cmd_id, msg.data)

                    # Encoder estimate
                    twist_msg = TwistWithCovarianceStamped()
                    twist_msg.header.frame_id = (
                        self.axis0_frame if axis_id == 1 else self.axis1_frame
                    )
                    twist_msg.header.stamp = rospy.Time.from_sec(msg.timestamp)

                    twist_msg.twist = TwistWithCovariance()

                    # TODO Use actual covariance measurements (first we need data to estimate these)
                    twist_msg.twist.covariance = np.diag([0.1, 0.1, 0, 0, 0, 0]).reshape((1, 36)).tolist()[0]
                    twist_msg.twist.twist = Twist()

                    speed = can_msg["Vel_Estimate"]  # in rev/s
                    speed *= np.pi * self.wheel_diameter
                    twist_msg.twist.twist.linear.x = speed if axis_id == 1 else -speed # The left wheel is inverted

                    if axis_id == 1:
                        self.vel_right.publish(twist_msg)
                    elif axis_id == 2:
                        self.vel_left.publish(twist_msg)

                    continue

            # If the message was not recognised, just post it to the general topic
            self.can_pub.publish(
                f"{msg.timestamp} - [{msg.arbitration_id}] {msg.data.hex()}"
            )

            # Check for external shutdown
            if rospy.is_shutdown():
                return

    def send_on_can(self, msg: can.Message) -> None:
        """Sends a message msg over the can bus

        Args:
            msg: the message to send

        """
        try:
            self.bus.send(msg)
            rospy.loginfo(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            rospy.logerr("Message NOT sent")


if __name__ == "__main__":
    try:
        cp = CanConverter()
    except rospy.ROSInterruptException:
        pass
