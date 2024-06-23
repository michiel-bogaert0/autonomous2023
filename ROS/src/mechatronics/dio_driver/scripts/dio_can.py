#!/usr/bin/python3
from functools import partial

import rospy
from can_msgs.msg import Frame
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Bool


class DioCAN(ManagedNode):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_control", "active")

        self.DI_publishers = [
            rospy.Publisher(f"/di/{i}", Bool, queue_size=10) for i in range(10)
        ]
        self.DO_subscribers = [
            rospy.Subscriber(f"/do/{i}", Bool, partial(self.handle_DO_change, i))
            for i in range(8)
        ]

        self.can_sub = rospy.Subscriber("/can/rx", Frame, self.handle_can_msg)
        self.can_pub = rospy.Publisher("/can/tx", Frame, queue_size=10)

        self.DO_state = 0x0

    def handle_DO_change(self, dig_out_id, msg):
        if msg.data:
            self.DO_state = self.DO_state | (0x80 >> dig_out_id)
        else:
            self.DO_state = self.DO_state & (~(0x80 >> dig_out_id))

        self.send_over_can()

    def send_over_can(self):
        msg = Frame()
        msg.id = 258
        msg.is_extended = False
        msg.dlc = 1
        msg.data = bytearray([self.DO_state, 0, 0, 0, 0, 0, 0, 0])

        self.can_pub.publish(msg)

    def active(self):
        self.send_over_can()

    def handle_can_msg(self, msg: Frame):
        # Decode the state
        # First 10 bits: DI
        # Following 8 bits: DO
        # Publish on publishers

        value = msg.data[0] << 8 + (msg.data[1] & 0xC0)

        for i, publisher in enumerate(self.DI_publishers):
            state = 1 if value & (0x8000 >> i) > 0 else 0
            publisher.publish(Bool(data=state))


driver = DioCAN()
driver.spin()
