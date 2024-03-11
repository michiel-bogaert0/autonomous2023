#!/usr/bin/env python3
import cantools
import rospy
from can_msgs.msg import Frame
from dictionaries import messages, publishers


class CanProcessor:
    def __init__(self):
        self.messages = messages
        self.publishers = publishers

        self.db_adress = rospy.get_param(
            "~db_adress", "autonomous2023/ROS/src/can/dbc/motor.dbc"
        )
        self.db = cantools.database.load_file(self.db_adress)

    def receive_can_frame(self, msg: Frame) -> None:
        """Receives CAN frames, converts them to readable format and publishes them to a topic"""

        # decode the message
        decoded_msg = self.db.decode_message(msg.id, msg.data)

        # send the message
        for topic_name, msg in decoded_msg.items():
            # create ros msg
            type_name = str(type(msg))
            if type_name not in self.messages.keys():
                rospy.logwarn(f"Unknown message type: {type_name}")
                return
            publish_msg = self.messages[type_name](msg)

            # publish message
            if topic_name not in self.publishers.keys():
                self.publishers[topic_name] = rospy.Publisher(
                    f"/processed/{topic_name}", type(publish_msg), queue_size=10
                )
            publisher = self.publishers[topic_name]
            publisher.publish(publish_msg)
