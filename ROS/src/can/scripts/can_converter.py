#!/usr/bin/env python3
import can
import cantools
import rospy
from can_msgs.msg import Frame
from can_processor import CanProcessor
from node_fixture.managed_node import ManagedNode, NodeManagingStatesEnum
from ugr_msgs.msg import CanFrame


class CanConverter(ManagedNode):
    def __init__(self):
        super().__init__("can_driver_converter")

        self.doConfigure()

        self.listen_on_can()

    def doConfigure(self):
        self.db_adress = rospy.get_param("~db_adress", "motor.dbc")

        db_address = __file__.split("/")[:-1]
        db_address += ["..", "dbc", self.db_adress]
        self.db_adress = "/".join(db_address)

        self.db = cantools.database.load_file(self.db_adress)

        self.can_processor = CanProcessor(db=self.db)

        self.can_pub = self.AddPublisher("ugr/can", Frame, queue_size=10)
        self.can_ids = []
        self.specific_pubs = {}

        # create a bus instance
        self.bus = can.interface.Bus(
            channel=rospy.get_param("~can_interface", "can0"),
            bitrate=rospy.get_param("~can_baudrate", 250000),
            interface="socketcan",
        )

        rospy.Subscriber("ugr/send_can", CanFrame, self.send_on_can)

    def listen_on_can(self) -> None:
        """Listens to CAN and publishes all incoming messages to a topic (still non-readable format)"""
        """can_processor is used to decode the messages and publish them to a specific topic"""

        # iterate over received messages, keeps looping forever
        for msg in self.bus:
            if self.state == NodeManagingStatesEnum.ACTIVE:
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

                self.can_processor.receive_can_frame(can_msg)

            # Check for external shutdown
            if rospy.is_shutdown():
                return

            self.update()

    def send_on_can(self, msg: CanFrame) -> None:
        """ "Sends all messages, published on ugr/send_can, to the can bus"""

        if self.state != NodeManagingStatesEnum.ACTIVE:
            return

        # encode the message
        db_msg = self.db.get_message_by_name(msg.message)

        signals_dict = {}
        for signal in msg.signals:
            signals_dict[signal.key] = signal.value

        try:
            encoded_msg = db_msg.encode(signals_dict)
        except cantools.database.errors.EncodeError:
            rospy.logwarn(f"Message not in database {msg.message}")
            return

        # send the message
        self.bus.send(
            can.Message(
                timestamp=msg.header.stamp.to_sec(),
                is_error_frame=False,
                is_remote_frame=False,
                dlc=len(encoded_msg),
                arbitration_id=db_msg.frame_id,
                data=list(encoded_msg),
            )
        )


if __name__ == "__main__":
    try:
        cp = CanConverter()
    except rospy.ROSInterruptException:
        pass
