#!/usr/bin/env python3

from functools import partial
from time import time_ns

import rospy
from node_fixture.node_fixture import (
    AddSubscriber,
    ROSNode,
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)

from slam_simulator.srv import Dropout, DropoutRequest, DropoutResponse


class DropoutNode(ROSNode):
    def __init__(self) -> None:
        """
        This node allows someone to make a set of data streams 'drop out' on request.
        The dropout of a single DropoutNode happens simultaniously
        """

        super().__init__(
            f"slam_simulator_dropout_{time_ns()}",
            already_add_subscriber=False,
        )

        self.name = rospy.get_param("~name", f"dropout_{rospy.Time.now().to_nsec()}")
        self.inputs = rospy.get_param("~inputs", [])
        self.outputs = rospy.get_param("~outputs", [])
        self.enabled = rospy.get_param("~default_status", True)

        assert len(self.inputs) == len(self.outputs)

        # Due to the dynamic behaviour of this node the AddSubscriber calls need to be done manually
        # It also couples the output so that it knows to what it should publish the message
        for input, output in zip(self.inputs, self.outputs):
            AddSubscriber(input)(partial(self.handle_message, output))

        self.add_subscribers()

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Service that allows disabling / enabling topic
        rospy.Service(
            f"dropout/{self.name}",
            Dropout,
            partial(self.handle_server_callback, output),
        )

        self.enabled = True

        rospy.loginfo(f"Dropout simulation node '{self.name}' started!")
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM SIM] Dropout Status",
                message="Started.",
            )
        )

    # '_' is actually just a side-effect of manually registering this subscriber instead of using the decorator
    def handle_message(self, output, _, msg):
        """
        Handles an incoming message. Basically sends it to the output topic (or not)

        Args:
            - output: the output topic to publish to
            - msg: the msg to publish
        """

        if self.enabled:
            self.publish(output, msg)

    def handle_server_callback(self, _, req: DropoutRequest):
        """
        Handles a server callback. Based on the received request message it enables/disables this datastream.
        """
        self.enabled = req.enable
        rospy.loginfo(
            f"[{self.name}]> Dropout simulation node status: enabled={self.enabled}"
        )
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM SIM] Dropout Status",
                message=f"{'Enabled' if self.enabled else 'Disabled'}.",
            )
        )
        return DropoutResponse()


node = DropoutNode()
node.start()
