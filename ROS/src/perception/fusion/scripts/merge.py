#!/usr/bin/env python3
import rospy
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from visualization_msgs.msg import Marker, MarkerArray

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
        rospy.init_node('observation_merger_node', anonymous=True)
        
        rospy.Subscriber("/ugr/car/observations/lidar", ObservationWithCovarianceArrayStamped, self.publish)
        rospy.Subscriber("/ugr/car/observations/camera", ObservationWithCovarianceArrayStamped, self.publish)

        self.result_publisher = rospy.Publisher("/ugr/car/observations", ObservationWithCovarianceArrayStamped, queue_size=10)
        
        rospy.spin()

node = MergeNode()