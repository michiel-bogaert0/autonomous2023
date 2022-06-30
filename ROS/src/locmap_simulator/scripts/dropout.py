#!/usr/bin/env python3

from functools import partial

import rospy
from locmap_simulator.srv import Dropout, DropoutRequest, DropoutResponse
from node_fixture.node_fixture import AddSubscriber, ROSNode
import rostopic

from time import time_ns

class DropoutNode(ROSNode):
    def __init__(self) -> None:
        super().__init__(
            f"locmap_simulator_dropout_{time_ns()}",
            already_add_subscriber=False,
        )

        self.name = rospy.get_param("~name", f"dropout_{rospy.Time.now().to_nsec()}")
        self.inputs = rospy.get_param("~inputs", [])
        self.outputs = rospy.get_param("~outputs", [])
        self.enabled = rospy.get_param("~default_status", True)

        assert len(self.inputs) == len(self.outputs)

        for (input, output) in zip(self.inputs, self.outputs):
            AddSubscriber(input)(partial(self.handle_message, output))

        self.add_subscribers()

        # Service that allows disabling / enabling topic
        rospy.Service(
            f"dropout/{self.name}", Dropout, partial(self.handle_server_callback, output)
        )

        self.enabled = True

        rospy.loginfo(f"Dropout simulation node '{self.name}' started!")

    def handle_message(self, output, _, msg):
        if self.enabled:
            self.publish(output, msg)

    def handle_server_callback(self, _, req: DropoutRequest):
        self.enabled = req.enable
        rospy.loginfo(
            f"[{self.name}]> Dropout simulation node status: enabled={self.enabled}"
        )
        return DropoutResponse()


node = DropoutNode()
node.start()
