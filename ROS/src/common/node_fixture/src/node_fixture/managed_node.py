#!/usr/bin/python3
from functools import partial

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
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


class ManagedNode:
    """
    A class representing a managed node.

    Attributes:
    - state (NodeManagingStatesEnum): the current state of the node
    - handlerlist (list): a list of handlers for subscribers
    - publishers (list): a list of publishers
    """

    def __init__(self, name: str, default_state=NodeManagingStatesEnum.UNCONFIGURED):
        """
        Initializes a ManagedNode instance.

        Parameters:
        - name (str): the name of the node
        - default_state (NodeManagingStatesEnum): the default state of the node
        """

        rospy.init_node(name)

        # First set to warning, to make sure car doesn't start (and we know why)
        self.health = DiagnosticStatus(name="healthchecks", hardware_id=name)
        self.name = name
        self.state = default_state
        self.terminalpub = False
        self.handlerlist = []
        self.publishers = []
        self.healthrate = rospy.Rate(rospy.get_param("~healthrate", 3))
        self.statePublisher = rospy.Publisher("/state", State, queue_size=1)
        self.healthPublisher = rospy.Publisher(
            "/health/nodes", DiagnosticStatus, queue_size=1
        )
        rospy.Service(
            f"/node_managing/{name}/set", SetNodeState, self.handle_service_set
        )
        rospy.Service(
            f"/node_managing/{name}/get", GetNodeState, self.handle_service_get
        )

        self.publish_rate = rospy.get_param("~rate", 10)
        self.rate = rospy.Rate(self.publish_rate)

        # Before going into the loop, set health to OK
        self.set_health(DiagnosticStatus.OK, message="OK")

        if default_state == NodeManagingStatesEnum.ACTIVE or rospy.get_param(
            "~turn_active", False
        ):
            # serice call to set the state to inactive
            rospy.wait_for_service(f"/node_managing/{name}/set", timeout=0.5)
            rospy.ServiceProxy(f"/node_managing/{name}/set", SetNodeState)(
                NodeManagingStatesEnum.INACTIVE
            )
            # service call to set the state to active
            rospy.wait_for_service(f"/node_managing/{name}/set", timeout=0.5)
            rospy.ServiceProxy(f"/node_managing/{name}/set", SetNodeState)(
                NodeManagingStatesEnum.ACTIVE
            )

        elif default_state == NodeManagingStatesEnum.INACTIVE:
            # serice call to set the state to inactive
            rospy.wait_for_service(f"/node_managing/{name}/set", timeout=0.5)
            rospy.ServiceProxy(f"/node_managing/{name}/set", SetNodeState)(
                NodeManagingStatesEnum.INACTIVE
            )

    def spin(self):
        """
        A custom spin function that keeps the node running until it is shutdown.
        This should be used instead of rospy.spin()
        """

        while not rospy.is_shutdown():
            self.spinOnce()

    def spinOnce(self):
        """
        A custom spinOnce function that keeps the node running until it is shutdown.
        This should be used instead of rospy.spinOnce()
        Basically limits the loop rate based on the publish rate of the node (~rate) and runs the update function to publish heartbeats (for example)
        """

        self.update()
        self.rate.sleep()

        if rospy.is_shutdown():
            exit(0)

    def get_health_level(self):
        """
        Returns the health level of the node.
        """
        return self.health.level

    def set_health(
        self, level: int, message: str = "", values: list = None, publish=True
    ):
        """
        Sets the health of the node.

        Args:
            level: The health level of the node.
            message: The message to be published.
            publish: Whether to publish the health immediately.
        """

        if values is None:
            values = []

        self.health.level = level
        self.health.message = message
        self.health.values = [KeyValue(key="state", value=self.state)]
        self.health.values += values

        if level == 1 and self.terminalpub:
            rospy.logwarn(f"\n{self.name}:{message}")
        elif level == 2 and self.terminalpub:
            rospy.logerr(f"\n{self.name}:{message}")

        # Immediately publish health
        if publish:
            self.healthPublisher.publish(self.health)

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
            self.state = NodeManagingStatesEnum.INACTIVE
            self.doConfigure()
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.UNCONFIGURED
        ):
            self.state = NodeManagingStatesEnum.UNCONFIGURED
            self.doCleanup()
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.ACTIVE
        ):
            self.state = NodeManagingStatesEnum.ACTIVE
            self.doActivate()
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request.state == NodeManagingStatesEnum.INACTIVE
        ):
            self.state = NodeManagingStatesEnum.INACTIVE
            self.doDeactivate()
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.state = NodeManagingStatesEnum.FINALIZED
            self.doShutdown()
            rospy.signal_shutdown("Node finalized")
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.state = NodeManagingStatesEnum.FINALIZED
            self.doShutdown()
            rospy.signal_shutdown("Node finalized")
        elif (
            self.state == NodeManagingStatesEnum.UNCONFIGURED
            and request.state == NodeManagingStatesEnum.FINALIZED
        ):
            self.state = NodeManagingStatesEnum.FINALIZED
            self.doShutdown()
            rospy.signal_shutdown("Node finalized")
        else:
            # invalid state transition
            rospy.loginfo(
                f"Invalid state transition in {self.name} from {self.state} to {request.state}"
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

        self.set_health(
            DiagnosticStatus.OK,
            message=f"State changed from {original_state} to {self.state}",
        )

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

        # Publish health
        if self.healthrate.remaining() < rospy.Duration(0):
            self.healthrate.sleep()
            self.healthPublisher.publish(self.health)

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

    def AddPublisher(self, topic: str, msg_type, queue_size: int, latch=False):
        """
        Adds a publisher to the node.

        Parameters:
        - topic (str): the topic to publish to
        - msg_type (msg): the message type
        - queue_size (int): the size of the publisher queue

        Returns:
        - CustomPublisher: the publisher instance
        """
        custompublisher = CustomPublisher(
            topic, msg_type, queue_size, self.state, latch
        )
        self.publishers.append(custompublisher)
        return custompublisher


# override the publish method of rospy.Publisher
class CustomPublisher(rospy.Publisher):
    def __init__(self, topic: str, msg_type, queue_size: int, state: str, latch):
        super().__init__(topic, msg_type, queue_size=queue_size, latch=latch)
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
