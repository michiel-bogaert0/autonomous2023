#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from ugr_msgs.msg import PathIds


class PathBuilder(ManagedNode):
    def __init__(self):
        super().__init__("path_builder")
        self.spin()

    def doConfigure(self):
        self.publisher = super().AddPublisher("/output/path", Path, queue_size=10)

    def doActivate(self):
        self.subscriber = super().AddSubscriber(
            "/input/path", PathIds, self.path_callback
        )

    def path_callback(self, msg: PathIds):
        """
        Creates global path, given pathplanning path and IDs
        """
        rospy.loginfo("Received path")
        # rospy.loginfo(msg)
        path = Path()
        path.header = msg.header
        for pose in msg.poses:
            print(pose)
            path.poses.append(pose)

        self.publisher.publish(path)


PathBuilder()
