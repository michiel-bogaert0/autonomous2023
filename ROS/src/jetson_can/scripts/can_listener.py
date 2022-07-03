#!/usr/bin/env python3

import rospy
import can
import time
from std_msgs.msg import String

class CanPublisher:

    def __init__(self):
        rospy.init_node("jetson_can")
        pub = rospy.Publisher("/diagnostics", String, queue_size=10)

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 1000000),
        )

        while True:
            
            # Check for external shutdown
            if rospy.is_shutdown():
                return
                
            # Send back a test message
            self.send_on_can(can.Message(arbitration_id=0x41, data=[0, 0, 0, 0, 63, 0, 0, 0]))

            time.sleep(1)

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
            pub.publish(f"[{msg.arbitration_id}] {msg.data.hex()}")

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
