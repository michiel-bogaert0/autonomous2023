#!/usr/bin/env python3

from functools import partial
from math import atan2, cos, pi, sin
from typing import List

import rospy
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import ROSNode
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import quaternion_from_euler


class HeadingEstimation(ROSNode):
    def __init__(self):
        """
        This node estimates the heading based on two GPS's, which must be fairly accurate

        Args:
            max_time_deviation: how much the two messages are allowed to deviate time-wise before calculating heading
            base_link_frame: the base_link_frame of the car
            rate: the maximal publishing rate of the heading. Can be much less than this value in practice
        """

        super().__init__("gps_heading_estimation", False)
    
        self.max_time_deviation = rospy.get_param("~max_time_deviation", 0.1)
        self.rate = rospy.get_param("~rate", 10)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.gps_msgs: List[NavSatFix, NavSatFix] = [None, None]

        # Add subscribers for both GPS's
        rospy.Subscriber(
            "/input/gps0",
            NavSatFix,
            partial(self.handle_gps, gpsIndex=0),
        )
        rospy.Subscriber(
            "/input/gps1",
            NavSatFix,
            partial(self.handle_gps, gpsIndex=1),
        )

        # Publish the heading periodically, limited by self.rate
        self.rosrate = rospy.Rate(self.rate, True)
        while not rospy.is_shutdown():
            self.publish_heading()
            self.rosrate.sleep()

    def publish_heading(self):
        """
        Actually publishes the heading (if heading can be calculated)
        """

        # No message? No heading!
        if not self.gps_msgs[0] or not self.gps_msgs[1]:
            return

        # Time deviates to much? No heading!
        if self.gps_msgs[0].header.stamp.to_sec() - self.gps_msgs[1].header.stamp.to_sec() > self.max_time_deviation:
            return

        # Actually calculate heading
        long0 = self.gps_msgs[0].longitude * 2 * pi / 360
        long1 = self.gps_msgs[1].longitude * 2 * pi / 360
        lat0 = self.gps_msgs[0].latitude * 2 * pi / 360
        lat1 = self.gps_msgs[1].latitude * 2 * pi / 360

        y = sin(long1 - long0) * cos(lat1)
        x = cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(long1 - long0)

        bearing = atan2(y, x) * (-1)

        # Publish as Imu message
        # TODO estimate covariance based on GPS fixes
        msg = Imu()
        msg.header.frame_id = self.base_link_frame
        msg.header.stamp = rospy.Time.from_sec((self.gps_msgs[0].header.stamp.to_sec() + self.gps_msgs[1].header.stamp.to_sec()) / 2)

        x, y, z, w = quaternion_from_euler(0, 0, bearing)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        self.publish("/output/heading", msg)

        self.gps_msgs = [None, None]

    def handle_gps(self, navsatfixMsg: NavSatFix, gpsIndex: int):
        """
        Handles an incoming NavSatFix message

        Args:
            navsatfixMsg: the incoming NavSatFix message
            gpsIndex: the index of the gps (0 or 1)
        """ 
        self.gps_msgs[gpsIndex] = navsatfixMsg

if __name__ == "__main__":
    try:
        estimator = HeadingEstimation()
    except rospy.ROSInterruptException:
        pass
