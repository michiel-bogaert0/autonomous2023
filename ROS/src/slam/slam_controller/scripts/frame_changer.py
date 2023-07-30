#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class OdomFrameChanger:
    def __init__(self):
        """
        Basically changes the frame id of the message. 
        Used for visualization and comparison to GT purposes
        """
        rospy.init_node('odom_frame_changer')

        self.frame = rospy.get_param("~frame_id", "")
        self.override_timestamp = rospy.get_param("~override_timestamp", False)

        # Subscribe to the Odometry topic
        self.sub = rospy.Subscriber('/input/odom', Odometry, self.odom_callback)

        # Create a publisher for the modified Odometry message
        self.pub = rospy.Publisher('/output/odom', Odometry, queue_size=10)

    def odom_callback(self, msg):
        # Modify the frame id of the Odometry message
        msg.header.frame_id = self.frame
        if self.override_timestamp:
            msg.header.stamp = rospy.Time().now()

        # Publish the modified Odometry message
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        OdomFrameChanger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass