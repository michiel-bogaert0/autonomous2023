#!/usr/bin/env python3
import sys

import can
import cantools
import rospy
from can_msgs.msg import Frame
from can_processor import CanProcessor
from node_fixture import roscan_to_serialcan, serialcan_to_roscan
from node_fixture.managed_node import ManagedNode, NodeManagingStatesEnum


class CanConverter(ManagedNode):
    def __init__(self, name):
        super().__init__(name)

        # Exception because of the way CAN works
        self.doConfigure()
        self.listen_on_can()

    def doConfigure(self):
        # Parameters
        dbc_filename = rospy.get_param("~dbc_filename", "lv.dbc")
        self.can_name = rospy.get_param("~can_name", "lv")
        self.can_interface = rospy.get_param("~can_interface", "can0")
        self.can_baudrate = rospy.get_param("~can_baudrate", 250000)

        # load dbc file
        db_address = __file__.split("/")[:-1]
        db_address += ["..", "dbc", dbc_filename]
        self.db_adress = "/".join(db_address)
        self.db = cantools.database.load_file(self.db_adress)

        self.can_ids = []
        self.specific_pubs = {}

        # CAN DBC processor
        self.can_processor = CanProcessor(self.db, self.can_name)

        # create a bus instance
        self.bus = can.interface.Bus(
            channel=self.can_interface,
            bitrate=self.can_baudrate,
            interface="socketcan",
        )

        # Pubs and subs
        self.AddSubscriber("ugr/send_can_raw", Frame, self.send_on_can_raw)
        self.can_pub = self.AddPublisher("ugr/can", Frame, queue_size=10)

    def listen_on_can(self) -> None:
        """Listens to CAN and publishes all incoming messages to a topic (still non-readable format)"""
        """can_processor is used to decode the messages and publish them to a specific topic"""

        # iterate over received messages, keeps looping forever
        for msg in self.bus:
            if self.state == NodeManagingStatesEnum.ACTIVE:
                # Publish globally
                can_msg = serialcan_to_roscan(msg)
                self.can_pub.publish(can_msg)

                # publish message on specific ID topic (without making duplicates of the same topic)
                if can_msg.id not in self.can_ids:
                    self.specific_pubs[str(can_msg.id)] = rospy.Publisher(
                        f"ugr/can/{self.can_name}/{format(can_msg.id, 'x')}",
                        Frame,
                        queue_size=10,
                    )
                    self.can_ids.append(can_msg.id)

                pub = self.specific_pubs[str(can_msg.id)]
                pub.publish(can_msg)

                # Publish a processed version based on DBC
                self.can_processor.receive_can_frame(can_msg)

            # Check for external shutdown
            if rospy.is_shutdown():
                return

            self.update()

    def send_on_can_raw(self, msg: Frame):
        """Subscriber handler for sending raw CAN messages to the bus
            Only works when node is active

        Args:
            msg (Frame): the raw CAN message to be sent
        """
        if self.state != NodeManagingStatesEnum.ACTIVE:
            return

        self.bus.send(roscan_to_serialcan(msg))


if __name__ == "__main__":
    try:
        args = sys.argv

        # Logic to get the name of the node, so multiple instances of this node can be created (for different CAN buses)

        for arg in args:
            if arg.startswith("__name:="):
                name = arg.split("__name:=")[1]

        cp = CanConverter(name)
    except rospy.ROSInterruptException:
        pass
