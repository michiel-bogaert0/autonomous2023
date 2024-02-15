#!/usr/bin/env python3
import cantools
import rospy
from can_msgs.msg import Frame
from std_msgs.msg import Float32, String


class CanProcessor:
    def __init__(self):
        self.db = cantools.database.load_file(
            "autonomous2023/ROS/src/can/dbc/hv500_can2_map_v23_EID_LINKS.dbc"
        )
        self.can_sub = rospy.Subscriber("/ugr/can", Frame, self.receive_can_frame)

        rospy.spin()

    def receive_can_frame(self, msg: Frame) -> None:
        """Receives CAN frames, converts them to readable format and publishes them to a topic"""

        # decode the message
        decoded_msg = self.db.decode_message(msg.id, msg.data)

        # send the message
        for topic_name, msg in decoded_msg.items():
            # float messages
            if isinstance(msg, float):
                self.float_pub = rospy.Publisher(
                    f"/ugr/processed/{topic_name}", Float32, queue_size=10
                )
                self.float_pub.publish(msg)

            # string messages
            elif isinstance(msg, cantools.database.can.signal.NamedSignalValue):
                self.str_pub = rospy.Publisher(
                    f"/ugr/processed/{topic_name}", String, queue_size=10
                )
                self.str_pub.publish(str(msg))

            else:
                rospy.logerr(f"Unknown message type: {type(msg)}")
                return
