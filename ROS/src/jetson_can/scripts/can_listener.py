#!/usr/bin/env python3

from math import pi
import rospy
import can
import struct
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

WHEEL_DIAMETER = 8 * 2.54 / 100 # in m

class CanPublisher:

    def __init__(self):
        rospy.init_node("jetson_can")
        self.can_pub = rospy.Publisher("/can_messages", String, queue_size=10)

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 1000000),
        )

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
            axis_id = (msg.arbitration_id >> 5) & 0b1
            if axis_id == 1 or axis_id == 2:
                cmd_id = msg.arbitration_id & 0b11111

                if cmd_id == 9:
                    # Encoder estimate
                    twist_msg = TwistStamped()
                    
                    speed = struct.unpack('f', msg.data[4:]) # in rev/s
                    speed *= pi * WHEEL_DIAMETER

                    twist_msg.twist.linear.x = speed

                    # TODO: use both wheels data?
            

            # If the message was not recognised, just post it to the general topic
            self.can_pub.publish(f"{msg.timestamp} - [{msg.arbitration_id}] {msg.data.hex()}")

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
        cp = CanPublisher()
    except rospy.ROSInterruptException:
        pass
