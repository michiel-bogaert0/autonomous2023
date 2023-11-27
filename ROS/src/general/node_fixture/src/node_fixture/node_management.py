#!/usr/bin/python3
import os
from functools import partial

import rospkg
import rospy
import yaml
from node_fixture.fixture import NodeManagingStatesEnum
from node_fixture.srv import (
    GetNodeState,
    GetNodeStateRequest,
    GetNodeStateResponse,
    SetNodeState,
    SetNodeStateRequest,
    SetNodeStateResponse,
)
from ugr_msgs.msg import State


def set_state(name: str, state: str) -> None:
    rospy.wait_for_service(f"/node_managing/{name}/set")
    return rospy.ServiceProxy(f"/node_managing/{name}/set", SetNodeState)(state)


def set_state_active(name: str) -> None:
    return set_state(name, NodeManagingStatesEnum.ACTIVE)


def set_state_inactive(name: str) -> None:
    return set_state(name, NodeManagingStatesEnum.INACTIVE)


def set_state_unconfigured(name: str) -> None:
    return set_state(name, NodeManagingStatesEnum.UNCONFIGURED)


def set_state_finalized(name: str) -> None:
    return set_state(name, NodeManagingStatesEnum.FINALIZED)


def configure_node(name: str) -> None:
    rospy.wait_for_service(f"/node_managing/{name}/get")
    data = rospy.ServiceProxy(f"/node_managing/{name}/get", GetNodeState)()
    if data.state == NodeManagingStatesEnum.ACTIVE:
        set_state_inactive(name)
        set_state_unconfigured(name)
    elif data.state == NodeManagingStatesEnum.INACTIVE:
        set_state_unconfigured(name)
    set_state_inactive(name)


def load_params(controller: str, mission: str) -> None:
    """
    Load parameters from a YAML file based on the controller and mission.
    Also takes the car name from the /car parameter.

    Args:
        controller (str): The name of the controller.
        mission (str): The name of the mission.

    Returns:
        None
    """
    pkg_path = rospkg.RosPack().get_path("ugr_launch")
    car = rospy.get_param("/car")
    filename = f"{controller}_{mission}.yaml"
    yaml_path = os.path.join(pkg_path, "config/", car, filename)
    with open(yaml_path, "r") as f:
        dic = yaml.safe_load(f)
    if dic is None:
        return
    for param_name, param_value in get_params(dic):
        param_name = "/" + param_name
        rospy.set_param(param_name, param_value)


def get_params(d: dict):
    """
    Recursively iterates through a dictionary and yields key-value pairs.

    Args:
        d (dict): The dictionary to iterate through.

    Yields:
        tuple: A tuple containing the key-value pair.

    """
    for key, value in d.items():
        if isinstance(value, dict):
            for lkey, lvalue in get_params(value):
                yield (key + "/" + lkey, lvalue)
        else:
            yield (key, value)


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
        self.name = name
        self.state = NodeManagingStatesEnum.UNCONFIGURED
        self.handlerlist = []
        self.publishers = []
        self.statePublisher = rospy.Publisher("/state", State, queue_size=1)
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
        original_state = self.state
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
            # response that the transition is unsuccesful
            return SetNodeStateResponse(succes=False)

        # set the state of all publishers
        for pub in self.publishers:
            pub.set_state(self.state)

        # publish the state of the node
        stateMsg = State()
        stateMsg.prev_state = original_state
        stateMsg.cur_state = self.state
        stateMsg.scope = self.name
        stateMsg.header.stamp = rospy.Time.now()
        self.statePublisher.publish(stateMsg)

        # response that the transition is succesful
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
