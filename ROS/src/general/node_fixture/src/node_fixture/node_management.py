#!/usr/bin/python3
from functools import partial

import rospy
from fixture import NodeManagingStatesEnum
from node_fixture.srv import (
    GetNodeState,
    GetNodeStateRequest,
    GetNodeStateResponse,
    SetNodeState,
    SetNodeStateRequest,
    SetNodeStateResponse,
)


def set_state(self, name: str, state: str) -> None:
    rospy.service_proxy(f"/node_managing/{name}/set", SetNodeState)(state)


def set_state_active(self, name: str) -> None:
    set_state(self, name, NodeManagingStatesEnum.ACTIVE)


def set_state_inactive(self, name: str) -> None:
    set_state(self, name, NodeManagingStatesEnum.INACTIVE)


def set_state_unconfigured(self, name: str) -> None:
    set_state(self, name, NodeManagingStatesEnum.UNCONFIGURED)


def set_state_finalized(self, name: str) -> None:
    set_state(self, name, NodeManagingStatesEnum.FINALIZED)


class ManagedNode:
    """
    A class representing a managed node.

    Attributes:
    - state (NodeManagingStatesEnum): the current state of the node
    - handlerlist (list): a list of handlers for subscribers
    - publishers (list): a list of publishers
    """

    def __init__(self, name: str):
        """
        Initializes a ManagedNode instance.

        Parameters:
        - name (str): the name of the node
        """
        self.state = NodeManagingStatesEnum.UNCONFIGURED
        self.handlerlist = []
        self.publishers = []
        rospy.Service(
            f"/node_managing/{name}/set", SetNodeState, self.handle_service_set
        )
        rospy.Service(
            f"/node_managing/{name}/get", GetNodeState, self.handle_service_get
        )

    def handle_service_get(self, request: GetNodeStateRequest) -> GetNodeStateResponse:
        """
        Handles a GetNodeState service request.

        Parameters:
        - request (GetNodeStateRequest): the service request

        Returns:
        - GetNodeStateResponse: the service response containing the current state of the node
        """
        return GetNodeStateResponse(state=self.state)

    def handle_service_set(self, request: SetNodeStateRequest) -> SetNodeStateResponse:
        """
        Handles a SetNodeState service request.

        This function handles a SetNodeState service request by transitioning the node
        between different states based on the requested state. The possible state transitions
        are as follows:
        - UNCONFIGURED -> INACTIVE
        - INACTIVE -> UNCONFIGURED
        - INACTIVE -> ACTIVE
        - ACTIVE -> INACTIVE
        - INACTIVE -> FINALIZED
        - ACTIVE -> FINALIZED
        - UNCONFIGURED -> FINALIZED

        Parameters:
        - request (SetNodeStateRequest): the service request

        Returns:
        - SetNodeStateResponse: the service response indicating whether the request was successful
        """
        if (
            self.state == NodeManagingStatesEnum.UNCONFIGURED
            and request.state == NodeManagingStatesEnum.INACTIVE
        ):
            self.doConfigure()
            self.state = NodeManagingStatesEnum.INACTIVE
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.UNCONFIGURED
        ):
            self.doCleanup()
            self.state = NodeManagingStatesEnum.UNCONFIGURED
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.ACTIVE
        ):
            self.doActivate()
            self.state = NodeManagingStatesEnum.ACTIVE
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request.state == NodeManagingStatesEnum.INACTIVE
        ):
            self.doDeactivate()
            self.state = NodeManagingStatesEnum.INACTIVE
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.doShutdown()
            self.state = NodeManagingStatesEnum.FINALIZED
            rospy.signal_shutdown("Node finalized")
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.doShutdown()
            self.state = NodeManagingStatesEnum.FINALIZED
            rospy.signal_shutdown("Node finalized")
        elif (
            self.state == NodeManagingStatesEnum.UNCONFIGURED
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.doShutdown()
            self.state = NodeManagingStatesEnum.FINALIZED
            rospy.signal_shutdown("Node finalized")
        else:
            # invalid state transition
            rospy.loginfo(
                f"Invalid state transition from {self.state} to {request.state}"
            )
            # response that the transition is unsuccesfull
            return SetNodeStateResponse(succes=False)

        # set the state of all publishers
        for pub in self.publishers:
            pub.set_state(self.state)

        # response that the transition is succesfull
        return SetNodeStateResponse(succes=True)

    def doConfigure(self):
        # UNCONFIGURED -> INACTIVE
        pass

    def doActivate(self):
        # INACTIVE -> ACTIVE
        pass

    def doCleanup(self):
        # INACTIVE -> UNCONFIGURED
        pass

    def doDeactivate(self):
        # ACTIVE -> INACTIVE
        pass

    def doShutdown(self):
        # INACTIVE -> FINALIZED
        # ACTIVE -> FINALIZED
        # UNCONFIGURED -> FINALIZED
        pass

    def update(self):
        """
        Updates the node based on its current state.
        """
        if self.state == NodeManagingStatesEnum.ACTIVE:
            self.active()
        elif self.state == NodeManagingStatesEnum.INACTIVE:
            self.inactive()
        elif self.state == NodeManagingStatesEnum.UNCONFIGURED:
            self.unconfigured()
        elif self.state == NodeManagingStatesEnum.FINALIZED:
            self.finalized()

    def active(self):
        # ACTIVE state
        pass

    def inactive(self):
        # INACTIVE state
        pass

    def unconfigured(self):
        # UNCONFIGURED state
        pass

    def finalized(self):
        # FINALIZED state
        pass

    def AddSubscriber(self, topic: str, msg_type, handler):
        """
        Adds a subscriber to the node.

        Parameters:
        - topic (str): the topic to subscribe to
        - msg_type (msg): the message type
        - handler (function): the handler function for the subscriber

        Returns:
        - Subscriber: the subscriber instance
        """
        our_handler = partial(self._make_custom_handler, handler)
        self.handlerlist.append(our_handler)
        return rospy.Subscriber(topic, msg_type, our_handler)

    def _make_custom_handler(self, handler, msg):
        """
        Creates a custom handler for a subscriber.

        Parameters:
        - handler (function): the original handler function
        - msg (msg): the message received by the subscriber

        Returns:
        - Any: the result of the handler function if the node is in the active state, otherwise None
        """
        if self.state == NodeManagingStatesEnum.ACTIVE:
            return handler(msg)

    def AddPublisher(self, topic: str, msg_type, queue_size: int):
        """
        Adds a publisher to the node.

        Parameters:
        - topic (str): the topic to publish to
        - msg_type (msg): the message type
        - queue_size (int): the size of the publisher queue

        Returns:
        - CustomPublisher: the publisher instance
        """
        custompublisher = CustomPublisher(topic, msg_type, queue_size, self.state)
        self.publishers.append(custompublisher)
        return custompublisher


# override the publish method of rospy.Publisher
class CustomPublisher(rospy.Publisher):
    def __init__(self, topic: str, msg_type, queue_size: int, state: str):
        super().__init__(topic, msg_type, queue_size=queue_size)
        self.state = state

    def set_state(self, state: str) -> None:
        self.state = state

    def publish(self, msg):
        """
        Publishes a message if the node is in the ACTIVE state.

        Args:
            msg: The message to be published.
        """
        if self.state == NodeManagingStatesEnum.ACTIVE:
            super().publish(msg)
