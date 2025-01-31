#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from dictionaries import messages, publishers


class CanProcessor:
    def __init__(self, db, name):
        self.name = name
        self.messages = messages
        self.publishers = publishers
        self.db = db

    # ID: 1A45
    # DLC: 2
    # Data: 00 00

    def receive_can_frame(self, msg: Frame) -> None:
        """Receives CAN frames, converts them to readable format and publishes them to a topic"""

        # decode the message
        try:
            decoded_msg = self.db.decode_message(msg.id, msg.data)
        except KeyError:
            # This means that the message is not in the database...
            # Just log a warning and return...
            rospy.logwarn_once(f"Message not in database, ID {hex(msg.id)}")
            return
        except ValueError:
            return

        # send the message
        for topic_name, msg in decoded_msg.items():
            topic_name = topic_name.lower().replace(" ", "_")

            # create ros msg
            type_name = str(type(msg))
            if type_name not in self.messages.keys():
                rospy.logwarn_once(f"Unknown message type: {type_name}")
                return
            publish_msg = self.messages[type_name](msg)

            # publish message
            if topic_name not in self.publishers.keys():
                self.publishers[topic_name] = rospy.Publisher(
                    f"/ugr/can/{self.name}/processed/{topic_name}",
                    type(publish_msg),
                    queue_size=10,
                )
            publisher = self.publishers[topic_name]
            publisher.publish(publish_msg)
