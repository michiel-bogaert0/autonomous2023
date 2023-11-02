#!/usr/bin/python3
from abc import ABC, abstractmethod
from functools import partialmethod

import rospy
from node_fixture import NodeManagingStatesEnum
from node_fixture.srv import GetNodeState, SetNodeState


class ManagedNode(ABC):
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

    @abstractmethod
    def doConfigure():
        pass

    @abstractmethod
    def doActivate():
        pass

    @abstractmethod
    def doCleanup():
        pass

    @abstractmethod
    def doDeactivate():
        pass

    @abstractmethod
    def doShutdown():
        pass

    @abstractmethod
    def doError():
        pass

    @abstractmethod
    def Active():
        pass

    @abstractmethod
    def Inactive():
        pass

    @abstractmethod
    def Unconfigured():
        pass

    @abstractmethod
    def Finalized():
        pass

    def update(self):
        if self.state == NodeManagingStatesEnum.ACTIVE:
            self.Active()
        elif self.state == NodeManagingStatesEnum.INACTIVE:
            self.Inactive()
        elif self.state == NodeManagingStatesEnum.UNCONFIGURED:
            self.Unconfigured()
        elif self.state == NodeManagingStatesEnum.FINALIZED:
            self.Finalized()

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
        super().__init__(topic, msg_type, queue_size)
        self.state = state

    def set_state(self, state):
        self.state = state

    def publish(self, msg):
        if self.state == NodeManagingStatesEnum.ACTIVE:
            super().publish(msg)
