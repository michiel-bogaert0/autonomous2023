#!/usr/bin/python3
import enum
from collections import deque
from functools import partial,partialmethod
from pathlib import Path
from typing import Any, Type, get_type_hints
from abc import ABC, abstractmethod

import rospy
import rostopic



class ManagedNode(ABC):
    def __init__(self):
        self.state = "unconfigured"
        self.handlerlist = []
        self.publishers = []
        rospy.Service("node_managing", ManagedNode, self.handle_service)

        

    def handle_service(self, request):
        if (self.state == "unconfigured" and request == "inactive"):
            self.doConfigure()
            self.state = "inactive"
        elif (self.state == "inactive" and request == "unconfigured"):
            self.doCleanup()
            self.state = "unconfigured"
        elif (self.state == "inactive" and request == "active"):
            self.doActivate()
            self.state = "active"
        elif (self.state == "active" and request == "inactive"):
            self.doDeactivate()
            self.state = "inactive"
        elif (self.state == "inactive" and request == "finalized"):
            self.doShutdown()
            self.state = "finalized"
        elif (self.state == "active" and request == "finalized"):
            self.doShutdown()
            self.state = "finalized"
        elif (self.state == "unconfigured" and request == "finalized"):
            self.doError()
            self.state = "finalized"

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


    def AddSubscriber(self, topic, msg_type, handler):
        our_handler = partialmethod(self._make_custom_handler, handler)
        self.handlerlist.append(our_handler)
        return rospy.Subscriber(topic, msg_type, self.handlerlist[-1])
        
    def _make_custom_handler(self, handler, msg):
        if self.state == "active":
            return handler(msg)
    
    def AddPublisher(self, topic, msg_type, queue_size):
        custompublisher = CustomPublisher(topic, msg_type, queue_size,self.state)
        self.publishers.append(custompublisher)
        return custompublisher


#override the publish method of rospy.Publisher
class CustomPublisher(rospy.Publisher):
    def __init__(self, topic, msg_type, queue_size, state):
        super(CustomPublisher, self).__init__(topic, msg_type, queue_size)
        self.state = state

    def set_state(self, state):    
        self.state = state

    def publish(self, msg):
        if self.state == "active":
            super(CustomPublisher, self).publish(msg)
        else:
            pass