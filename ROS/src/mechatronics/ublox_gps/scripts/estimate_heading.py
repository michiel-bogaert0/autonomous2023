#!/usr/bin/env python3

from cmath import isnan
from collections import deque
from functools import partial
from math import atan2, cos, pi, sin, sqrt

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, NavSatFix
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


class HeadingEstimation:
    def __init__(self):
        """
        This node estimates the heading based on two GPS's, which must be fairly accurate

        Args:
            base_link_frame: the base_link_frame of the car
            time_offset: the time offset to use when publishing
            max_distance: the maximal distance between two consecutive points to make it "valid"
            min_distance: the minimal distance between two consecutive points to make it "valid"
            max_time_deviation: the maximal time deviation between two NavSatFix msgs for dual GPS estimation
        """
        # ros initialization
        rospy.init_node("gps_heading_estimation")
        self.yaw_pub = rospy.Publisher("/output/yaw", Point, queue_size=10)
        self.heading_pub = rospy.Publisher("/output/heading", Imu, queue_size=10)

        self.rate = rospy.get_param("~rate", 20)
        self.max_time_deviation = rospy.get_param("~max_time_deviation", 0.1)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.averaging_window = rospy.get_param("~averaging_window", 4)
        self.time_offset = rospy.get_param("~time_offset", 0.1)
        self.max_distance = rospy.get_param("~max_distance", 2)
        self.min_distance = rospy.get_param("~min_distance", 0.05)
        self.max_covariance = rospy.get_param("~max_covariance", 0.01)

        self.gps_msgs = [deque([], 2) for i in range(2)]
        self.gps_vel = [None, None]

        self.offset = [0, 0, 0]
        self.heading_yaw = [0, 0, 0]

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

        # Check for the dual gps source, limited by self.rate
        self.rosrate = rospy.Rate(self.rate, True)
        while not rospy.is_shutdown():
            self.estimate_dual_heading()
            self.rosrate.sleep()

    def calculate_heading_from_points(self, msg0: NavSatFix, msg1: NavSatFix):
        """
        Calculates the heading (as the derivative) between msg0 and msg1.
        The path between them is traversed FROM msg0 TO msg1

        The heading is normalized to ROS conventions (East = 0)

        Args:
            msg0, msg1: NavSatFix messages containing long/lat data to calculate heading from

        Returns:
            heading as the derivative from msg0 to msg1, following ROS conventions
            distance between the two points
        """
        long0 = msg0.longitude * 2 * pi / 360
        long1 = msg1.longitude * 2 * pi / 360
        lat0 = msg0.latitude * 2 * pi / 360
        lat1 = msg1.latitude * 2 * pi / 360

        dlong = long1 - long0
        dlat = lat1 - lat0

        R = 6365244  # Taking into account our general latitude
        a = sin(dlat / 2) ** 2 + cos(lat0) * cos(lat1) * sin(dlong / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c

        return (
            atan2(
                sin(dlong) * cos(lat1),
                cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(dlong),
            )
            * (-1)
            - pi / 2,
            distance,
        )

    def estimate_dual_heading(self):
        """
        Tries to calculate the heading based on both GPS's (dual GPS heading)
        """

        if len(self.gps_msgs[0]) < 1:
            return
        if len(self.gps_msgs[1]) < 1:
            return

        msg0 = self.gps_msgs[0][0]
        msg1 = self.gps_msgs[1][0]

        # No message? No heading!
        if not msg0 or not msg1:
            return

        if (
            msg0.position_covariance[0] > self.max_covariance
            or msg1.position_covariance[0] > self.max_covariance
        ):
            rospy.logwarn(
                "[Dual GPS]> Covariance on the GPS's is too big in order to calculate dual GPS heading"
            )
            return

        # Time deviates to much? No heading!
        if (
            abs(msg0.header.stamp.to_sec() - msg1.header.stamp.to_sec())
            > self.max_time_deviation
        ):
            rospy.logwarn("[Dual GPS]> time deviation too big")
            return

        # Actually calculate heading
        # Returns angle from RIGHT GPS (0) to LEFT GPS (1)
        bearing, _ = self.calculate_heading_from_points(msg0, msg1)
        bearing += pi / 2

        # Correct bearing
        if (
            abs(bearing - self.heading_yaw[2] - self.offset[2]) > pi
            and self.heading_yaw[2] != 0
        ):
            while abs(bearing - self.heading_yaw[2] - self.offset[2]) > pi:
                self.offset[2] += (
                    2
                    * pi
                    * (1 if bearing - self.heading_yaw[2] - self.offset[2] > 0 else -1)
                )

        bearing -= self.offset[2]
        self.heading_yaw[2] = bearing

        self.publish_heading(bearing)

    def estimate_single_heading(self, gpsIndex):
        """
        Tries to calculate the heading based on a single GPS source

        Args:
            gpsIndex: the index of the GPS to use (0 or 1)
        """

        if len(self.gps_msgs[gpsIndex]) < 2:
            return

        # Take one
        msg0 = self.gps_msgs[gpsIndex][1]
        msg1 = self.gps_msgs[gpsIndex].popleft()

        # Actually calculate heading and distance
        bearing, distance = self.calculate_heading_from_points(msg0, msg1)

        if distance > self.max_distance:
            rospy.logwarn(
                f"[GPS {gpsIndex}]> Distance between points of GPS {gpsIndex} is too long (d = {distance} m)"
            )
            return

        if distance < self.min_distance:
            rospy.logwarn(
                f"[GPS {gpsIndex}]> Distance between points of GPS {gpsIndex} is too short (d = {distance} m)"
            )
            return

        if (
            abs(bearing - self.heading_yaw[gpsIndex] - self.offset[gpsIndex]) > pi
            and self.heading_yaw[gpsIndex] != 0
        ):
            while (
                abs(bearing - self.heading_yaw[gpsIndex] - self.offset[gpsIndex]) > pi
            ):
                self.offset[gpsIndex] += (
                    2
                    * pi
                    * (
                        1
                        if bearing - self.heading_yaw[gpsIndex] - self.offset[gpsIndex]
                        > 0
                        else -1
                    )
                )

        bearing -= self.offset[gpsIndex]
        self.heading_yaw[gpsIndex] = bearing

        self.publish_heading(bearing)

    def publish_heading(self, bearing_input):
        """
        Actually publishes a heading as both a geometry_msgs/Point and a sensor_msgs/Imu

        Args:
            bearing_input: the bearing (heading, yaw, whatever) to publish
        """

        if isnan(bearing_input):
            return

        bearing = self.yaw_averager.process(bearing_input)
        bm = Point(z=bearing)

        self.yaw_pub.publish(bm)

        # Publish as Imu message
        msg = Imu()
        msg.header.frame_id = self.base_link_frame
        msg.header.stamp = rospy.Time.from_sec(
            rospy.Time.now().to_sec() - self.time_offset
        )

        x, y, z, w = quaternion_from_euler(0, 0, bearing)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        self.heading_pub.publish(msg)

    def handle_gps(self, navsatfixMsg: NavSatFix, gpsIndex: int):
        """
        Handles an incoming NavSatFix message

        Args:
            navsatfixMsg: the incoming NavSatFix message
            gpsIndex: the index of the gps (0 or 1)
        """
        self.gps_msgs[gpsIndex].append(navsatfixMsg)
        self.estimate_single_heading(gpsIndex)


if __name__ == "__main__":
    try:
        estimator = HeadingEstimation()
    except rospy.ROSInterruptException:
        pass
