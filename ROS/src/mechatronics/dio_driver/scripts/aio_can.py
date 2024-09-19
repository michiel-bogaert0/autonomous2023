#!/usr/bin/python3
from functools import partial

import rospy
from can_msgs.msg import Frame
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Bool


class AioCAN(ManagedNode):
    def __init__(self) -> None:
        """This node is responsible for handling the digital inputs and outputs of the car.

        - Sends the the DI at the same rate of the CAN messages.
        - Sends the DO at a fixed rate (parameter ~rate) and immediately when a change is detected.

        Note that this node DOES NOT send the ECU heartbeat of the DIO Bank, as required by the module.
        """
        super().__init__("aiobank_driver_can")

        # Publishers and subscribers. 10 DI and 8 DO
        self.DI_publishers = [
            self.AddPublisher(f"/ai/{i}", Bool, queue_size=10) for i in range(10)
        ]
        self.DO_feedback_publishers = [
            self.AddPublisher(f"/ao/{i}/feedback", Bool, queue_size=10)
            for i in range(2)
        ]
        self.DO_subscribers = [
            self.AddSubscriber(f"/ao/{i}", Bool, partial(self.handle_AO_change, i))
            for i in range(2)
        ]

        self.can_sub = self.AddSubscriber("/can/rx", Frame, self.handle_can_msg)
        self.can_pub = self.AddPublisher("/can/tx", Frame, queue_size=10)

    def handle_AO_change(self, dig_out_id, msg: Bool):
        """When an analog output changes, this function is called to update the state of the digital outputs."""
        if msg.data:
            self.AO_state = self.DO_state | (0x80 >> dig_out_id)
        else:
            self.DO_state = self.DO_state & (~(0x80 >> dig_out_id))

        self.send_over_can()

    def send_over_can(self):
        """
        Sends the current state of the digital outputs over CAN.
        """
        # todo implement
        raise ValueError("to be implemented")
        # msg = Frame()
        # msg.id = 258
        # msg.is_extended = False
        # msg.dlc = 1
        # msg.data = bytearray([self.DO_state, 0, 0, 0, 0, 0, 0, 0])

        # self.can_pub.publish(msg)

    def active(self):
        self.send_over_can()

    def handle_can_msg(self, msg: Frame):
        # Decode the state
        # First 10 bits: DI (10 bits, 0-9)
        # Following 8 bits: DO feedback (8 bits, 10-17)
        # Publish on publishers
        if msg.id != 0x101:
            rospy.logerr(f"Received message with ID {msg.id}, expected 0x101")
            return

        # Inputs
        inputs_state = [msg.data[0] & (0x80 >> i) for i in range(8)]
        inputs_state += [msg.data[1] & (0x80 >> i) for i in range(2)]

        for i, publisher in enumerate(self.DI_publishers):
            publisher.publish(Bool(data=inputs_state[i]))

        # Output feedback
        feedback_state = [msg.data[1] & (0x20 >> i) for i in range(6)]
        feedback_state += [msg.data[2] & (0x80 >> i) for i in range(2)]

        for i, publisher in enumerate(self.DO_feedback_publishers):
            publisher.publish(Bool(data=feedback_state[i]))


driver = AioCAN()
driver.spin()
