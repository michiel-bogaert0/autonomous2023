#!/usr/bin/env python3

from collections import deque
from functools import partial
from math import atan2, cos, pi, sin, sqrt

import rospy
from geometry_msgs.msg import Point
from node_fixture.node_fixture import ROSNode
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import quaternion_from_euler

class StreamingMovingAverage:
    """
    Class that averages streamed data (moving average)
    """
    def __init__(self, window_size):
        """
        Args:
            window_size: the size of the window to use
        """
        self.window_size = window_size
        self.values = []
        self.sum = 0

    def process(self, value):
        """
        Processes a new sample

        Args:
            value: the value to process

        Returns:
            the averaged result
        """
        self.values.append(value)
        self.sum += value

        if len(self.values) > self.window_size:
            self.sum -= self.values.pop(0)
        
        return float(self.sum) / len(self.values)

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

        self.max_time_deviation = rospy.get_param("~max_time_deviation", 0.05)
        self.rate = rospy.get_param("~rate", 10)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.averaging_window = rospy.get_param("~averaging_window", 4)
        self.time_offset = rospy.get_param("~time_offset", 0.1)
        self.max_distance = rospy.get_param("~max_distance", 2)
        self.min_distance = rospy.get_param("~min_distance", 0.05)

        self.gps_msgs = [deque([], 2) for i in range(2)]
        self.gps_vel = [None, None]

        self.offset = [0, 0]
        self.heading_yaw = [0, 0]

        self.yaw_averager = StreamingMovingAverage(self.averaging_window)

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

        rospy.spin()

    def publish_heading(self, gpsIndex):
        """
        Actually publishes the heading (if heading can be calculated)
        """

        if len(self.gps_msgs[gpsIndex]) < 2:
            return

        # Take one
        msg0 = self.gps_msgs[gpsIndex][1]       
        msg1 = self.gps_msgs[gpsIndex].popleft()

        current_time_sec = rospy.Time.now().to_sec()

        # Actually calculate heading and distance
        long0 = msg0.longitude * 2 * pi / 360
        long1 = msg1.longitude * 2 * pi / 360
        lat0 = msg0.latitude * 2 * pi / 360
        lat1 = msg1.latitude * 2 * pi / 360

        dlong = long1 - long0
        dlat = lat1 - lat0

        # Distance
        R = 6371000
        a = sin(dlat / 2) ** 2 + cos(lat0) * cos(lat1) * sin(dlong/2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1-a))

        distance = R * c

        bearing = atan2(sin(dlong) * cos(lat1), cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(dlong)) * (-1) - pi/2

        if distance > self.max_distance:
            rospy.logwarn(f"Distance between points of GPS {gpsIndex} is too long (d = {distance} m)")
            return

        if distance < self.min_distance:
            rospy.logwarn(f"Distance between points of GPS {gpsIndex} is too short (d = {distance} m)")
            return

        if abs(bearing - self.offset[gpsIndex] - self.heading_yaw[gpsIndex]) > pi and self.heading_yaw[gpsIndex] != 0:
            self.offset[gpsIndex] += 2 * pi * (1 if bearing - self.heading_yaw[gpsIndex] - self.offset[gpsIndex] > 0 else -1 )

        bearing -= self.offset[gpsIndex]

        self.heading_yaw[gpsIndex] = bearing

        bearing = self.yaw_averager.process(bearing)

        bm = Point(z=bearing)

        self.publish("/output/yaw", bm)

        # Publish as Imu message
        # TODO estimate covariance based on GPS fixes

        msg = Imu()
        msg.header.frame_id = self.base_link_frame
        msg.header.stamp = rospy.Time.from_sec(      
            current_time_sec - self.time_offset
        )

        x, y, z, w = quaternion_from_euler(0, 0, bearing)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        self.publish("/output/heading", msg)

    def handle_gps(self, navsatfixMsg: NavSatFix, gpsIndex: int):
        """
        Handles an incoming NavSatFix message

        Args:
            navsatfixMsg: the incoming NavSatFix message
            gpsIndex: the index of the gps (0 or 1)
        """
        self.gps_msgs[gpsIndex].append(navsatfixMsg)
        self.publish_heading(gpsIndex)

if __name__ == "__main__":
    try:
        estimator = HeadingEstimation()
    except rospy.ROSInterruptException:
        pass
