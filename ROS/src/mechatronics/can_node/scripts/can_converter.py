#!/usr/bin/env python3
import struct

import can
import cantools
import numpy as np
import rospy
from can_msgs.msg import Frame
from imu import ImuConverter
from odrive import OdriveConverter


class CanConverter:
    def __init__(self):
        rospy.init_node("can_converter")
        self.can_pub = rospy.Publisher("/output/can", Frame, queue_size=10)

        # The first element is the front IMU, the second is the rear IMU
        self.IMU_IDS = [0xE2, 0xE3]

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        # Create the right converters
        self.odrive = OdriveConverter()
        self.imu = ImuConverter()

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
            can_msg = Frame()
            can_msg.header.stamp = rospy.Time.from_sec(msg.timestamp)
            can_msg.id = msg.arbitration_id
            can_msg.data = msg.data
            self.can_pub.publish(can_msg)

            # Check if the message is a ODrive command
            axis_id = msg.arbitration_id >> 5
            if axis_id == 1 or axis_id == 2:
                cmd_id = msg.arbitration_id & 0b11111

                if cmd_id == 9:
                    self.odrive.handle_odrive_vel_msg(msg, axis_id, cmd_id)
                    continue

            imu_id = msg.id & 0xFF
            if imu_id in self.IMU_IDS:
                self.imu.handle_imu_msg(msg, imu_id == self.IMU_IDS[0])

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
