#!/usr/bin/env python3
import can
import rospy
from can_msgs.msg import Frame
from run import CanProcessor


class CanConverter:
    def __init__(self):
        rospy.init_node("can_converter")
        self.can_processor = CanProcessor()

        self.can_pub = rospy.Publisher("ugr/can", Frame, queue_size=10)
        self.can_ids = []
        self.specific_pubs = {}

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        try:
            self.listen_on_can()
        except rospy.ROSInterruptException:
            pass

    def listen_on_can(self) -> None:
        """Listens to CAN and publishes all incoming messages to a topic (still non-readable format)"""

        # iterate over received messages, keeps looping forever
        for msg in self.bus:
            # publish message on ROS
            can_msg = Frame()
            can_msg.header.stamp = rospy.Time.from_sec(msg.timestamp)
            can_msg.id = msg.arbitration_id
            can_msg.data = msg.data
            self.can_pub.publish(can_msg)

            # publish message on specific topic (without making duplicates of the same topic)
            if can_msg.id not in self.can_ids:
                self.specific_pubs[str(can_msg.id)] = rospy.Publisher(
                    f"ugr/can/{can_msg.id}", Frame, queue_size=10
                )
                self.can_ids.append(can_msg.id)

            pub = self.specific_pubs[str(can_msg.id)]
            pub.publish(can_msg)

            # Check for external shutdown
            if rospy.is_shutdown():
                return


if __name__ == "__main__":
    try:
        cp = CanConverter()
    except rospy.ROSInterruptException:
        pass
