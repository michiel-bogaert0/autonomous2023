#!/usr/bin/python3
# from abc import ABC, abstractmethod
from functools import partialmethod

import rospy
from fixture import NodeManagingStatesEnum
from node_fixture.srv import GetNodeState, SetNodeState


class ManagedNode:  # (ABC):
    def __init__(self, name):
        self.state = NodeManagingStatesEnum.UNCONFIGURED
        self.handlerlist = []
        self.publishers = []
        rospy.Service(
            f"/node_managing/{name}/set", GetNodeState, self.handle_service_set
        )
        rospy.Service(
            f"/node_managing/{name}/get", SetNodeState, self.handle_service_get
        )

    def handle_service_get(self):
        return self.state

    def handle_service_set(self, request):
        if (
            self.state == NodeManagingStatesEnum.UNCONFIGURED
            and request == NodeManagingStatesEnum.INACTIVE
        ):
            self.doConfigure()
            self.state = NodeManagingStatesEnum.INACTIVE
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request == NodeManagingStatesEnum.UNCONFIGURED
        ):
            self.doCleanup()
            self.state = NodeManagingStatesEnum.UNCONFIGURED
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request == NodeManagingStatesEnum.ACTIVE
        ):
            self.doActivate()
            self.state = NodeManagingStatesEnum.ACTIVE
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request == NodeManagingStatesEnum.INACTIVE
        ):
            self.doDeactivate()
            self.state = NodeManagingStatesEnum.INACTIVE
        elif (
            self.state == NodeManagingStatesEnum.INACTIVE
            and request == NodeManagingStatesEnum.FINALIZED
        ):
            self.doShutdown()
            self.state = NodeManagingStatesEnum.FINALIZED
        elif (
            self.state == NodeManagingStatesEnum.ACTIVE
            and request == NodeManagingStatesEnum.FINALIZED
        ):
            self.doShutdown()
            self.state = NodeManagingStatesEnum.FINALIZED
        elif (
            self.state == NodeManagingStatesEnum.UNCONFIGURED
            and request == NodeManagingStatesEnum.FINALIZED
        ):
            self.doError()
            self.state = NodeManagingStatesEnum.FINALIZED

        for pub in self.publishers:
            pub.set_state(self.state)

    # @abstractmethod
    def doConfigure(self):
        pass

    # @abstractmethod
    def doActivate(self):
        pass

    # @abstractmethod
    def doCleanup(self):
        pass

    # @abstractmethod
    def doDeactivate(self):
        pass

    # @abstractmethod
    def doShutdown(self):
        pass

    # @abstractmethod
    def doError(self):
        pass

    # @abstractmethod
    def active(self):
        pass

    # @abstractmethod
    def inactive(self):
        pass

    # @abstractmethod
    def unconfigured(self):
        pass

    # @abstractmethod
    def finalized(self):
        pass

    def update(self):
        if self.state == NodeManagingStatesEnum.ACTIVE:
            self.active()
        elif self.state == NodeManagingStatesEnum.INACTIVE:
            self.inactive()
        elif self.state == NodeManagingStatesEnum.UNCONFIGURED:
            self.unconfigured()
        elif self.state == NodeManagingStatesEnum.FINALIZED:
            self.finalized()

    def AddSubscriber(self, topic, msg_type, handler):
        our_handler = partialmethod(self._make_custom_handler, handler)
        self.handlerlist.append(our_handler)
        return rospy.Subscriber(topic, msg_type, self.handlerlist[-1])

    def _make_custom_handler(self, handler, msg):
        if self.state == NodeManagingStatesEnum.ACTIVE:
            return handler(msg)

    def AddPublisher(self, topic, msg_type, queue_size):
        custompublisher = CustomPublisher(topic, msg_type, queue_size, self.state)
        self.publishers.append(custompublisher)
        return custompublisher


# override the publish method of rospy.Publisher
class CustomPublisher(rospy.Publisher):
    def __init__(self, topic, msg_type, queue_size, state):
        super().__init__(topic, msg_type, queue_size=queue_size)
        self.state = state

    def set_state(self, state):
        self.state = state

    def publish(self, msg):
        if self.state == NodeManagingStatesEnum.ACTIVE:
            super().publish(msg)
