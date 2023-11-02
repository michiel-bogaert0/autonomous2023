#!/usr/bin/python3
import rospy
from node_management import ManagedNode
from std_msgs.msg import String


class DummyNode(ManagedNode):
    def __init__(self):
        super().__init__("dummy_node")
        rospy.init_node("dummy_node")
        self.pub = self.AddPublisher("/dummy_topic", String, queue_size=10)
        self.sub = self.AddSubscriber("/dummy_topic2", String, self.callback)
        self.run()

    def callback(self, msg):
        print(msg.data)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(0.1)

    def active(self):
        message = "Dit is een dummy bericht"
        rospy.loginfo("Verzenden: %s", message)
        self.pub.publish(message)
        self.rate.sleep()


node = DummyNode()
