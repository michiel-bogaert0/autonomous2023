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
        rospy.loginfo(msg.data)

    def run(self):
        counter = 0
        while not rospy.is_shutdown():
            counter += 1
            message = "Dit is een dummy bericht " + str(counter)
            rospy.loginfo("Verzenden: %s", message)
            self.pub.publish(message)
            self.update()
            rospy.sleep(0.1)

    def active(self):
        # message = "Dit is een dummy bericht"
        # rospy.loginfo("Verzenden: %s", message)
        # self.pub.publish(message)
        pass


node = DummyNode()
