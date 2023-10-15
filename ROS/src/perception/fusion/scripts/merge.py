#!/usr/bin/env python3
import rospy
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped


class MergeNode:
    def publish(self, msg):
        """
        Just publishes on the topic
        """
        self.result_publisher.publish(msg)

    def __init__(self):
        """
        Subscribes to two Observation topics and
        """
        rospy.init_node("observation_merger_node", anonymous=True)

        rospy.Subscriber(
            "/input/topic1", ObservationWithCovarianceArrayStamped, self.publish
        )
        rospy.Subscriber(
            "/input/topic2", ObservationWithCovarianceArrayStamped, self.publish
        )

        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        rospy.spin()


node = MergeNode()
