#!/usr/bin/env python3

import rospy
import can
from std_msgs.msg import String


def listen_on_can() -> None:
    """Listens on the can0 interface of the Jetson and publishes the messages"""

    pub = rospy.Publisher("/diagnostics", String, queue_size=10)

    # create a bus instance
    bus = can.interface.Bus(
        bustype="socketcan",
        channel=rospy.get_param("~interface", "can0"),
        bitrate=rospy.get_param("~baudrate", 1000000),
    )

    # iterate over received messages
    # this keeps looping forever
    for msg in bus:
        pub.publish(f"[{msg.arbitration_id}] {msg.data.hex()}")

        # Check for external shutdown
        if rospy.is_shutdown():
            return


if __name__ == "__main__":
    rospy.init_node("jetson_can")

    try:
        listen_on_can()
    except rospy.ROSInterruptException:
        pass
