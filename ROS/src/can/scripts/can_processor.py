#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from dictionaries import messages, publishers


class CanProcessor:
    def __init__(self, db):
        self.messages = messages
        self.publishers = publishers

        self.db = db

    def receive_can_frame(self, msg: Frame) -> None:
        """Receives CAN frames, converts them to readable format and publishes them to a topic"""

        # decode the message
        try:
            decoded_msg = self.db.decode_message(msg.id, msg.data)
        except KeyError as e:
            # This means that the message is not in the database...
            # Just log a warning and return...
            rospy.logwarn(f"Message not in database {e}")
            return

        # send the message
        for topic_name, msg in decoded_msg.items():
            topic_name = topic_name.lower().replace(" ", "_")

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
