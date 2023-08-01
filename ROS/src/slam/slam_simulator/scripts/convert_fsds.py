#!/usr/bin/env python3
import math
from tkinter.tix import Tree

import numpy as np
import rospy
import tf
from fs_msgs.msg import Track
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import (DiagnosticArray, DiagnosticStatus,
                                       create_diagnostic_message)
from sensor_msgs.msg import Imu, NavSatFix


class Convert:
    def __init__(self):
        # ROS initialization
        rospy.init_node("converter")
        self.gss_sub = rospy.Subscriber("/input/gss", TwistStamped, self.convertGSS)
        self.imu_sub = rospy.Subscriber("/input/imu", Imu, self.convertIMU)
        self.gt_odom_sub = rospy.Subscriber(
            "/input/gt_odometry", Odometry, self.convertGTOdometry
        )
        self.track_sub = rospy.Subscriber("/input/gt_track", Track, self.convertTrack)
        self.gps_sub = rospy.Subscriber("/fsds/gps", NavSatFix, self.convertGPS)

        self.gss_pub = rospy.Publisher("/input/gss", TwistStamped, queue_size=10)
        self.imu_pub = rospy.Publisher("/input/imu", Imu, queue_size=10)
        self.gt_odom_pub = rospy.Publisher(
            "/input/gt_odometry", Odometry, queue_size=10
        )
        self.track_pub = rospy.Publisher("/input/gt_track", Track, queue_size=10)
        self.gps_pub = rospy.Publisher("/fsds/gps", NavSatFix, queue_size=10)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.gt_world_frame = rospy.get_param("~gt_world_frame", "ugr/gt_car_map")
        self.gt_base_link_frame = rospy.get_param(
            "~gt_base_link_frame", "ugr/gt_car_base_link"
        )
        self.max_mean = rospy.get_param("~max_mean", 1.0)
        self.min_mean = rospy.get_param("~min_mean", 0.0)
        self.mean_rate = rospy.get_param("~mean_rate", 0.1)
        self.std_deviation = rospy.get_param("~standard_deviation", 3)
        self.add_noise = rospy.get_param("~add_noise", True)

        self.br = tf.TransformBroadcaster()

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        print("Convert FSDS Started!")
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM SIM] FSDS Convert Status",
                message="Started.",
            )
        )

    def convertGSS(self, msg: TwistStamped):
        covariance = np.diag([0.1, 0.1, 0, 0, 0, 0])

        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.header.frame_id = self.base_link_frame
        new_msg.twist.twist = msg.twist

        new_msg.twist.twist.linear.x = (
            msg.twist.linear.x**2 + msg.twist.linear.y**2
        ) ** (1 / 2)
        if self.add_noise:
            new_msg.twist.twist.linear.x += np.random.normal(
                min(
                    self.min_mean + new_msg.twist.twist.linear.x * self.mean_rate,
                    self.max_mean,
                ),
                self.std_deviation,
            )
        new_msg.twist.twist.linear.y = 0

        new_msg.twist.covariance = covariance.reshape((1, 36)).tolist()[0]

        self.gss_pub.publish(new_msg)

    def convertIMU(self, msg: Imu):
        msg.header.frame_id = self.base_link_frame

        self.imu_pub.publish( msg)

    def convertGTOdometry(self, msg: Odometry):
        msg.header.frame_id = self.gt_world_frame
        msg.child_frame_id = self.gt_base_link_frame

        # Also publish a transformation
        self.br.sendTransform(
            (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
            msg.header.stamp,
            self.gt_base_link_frame,
            self.gt_world_frame,
        )

        self.gt_odom_pub.publish( msg)

    def convertTrack(self, msg: Track):
        self.track_pub.publish( msg)

    def convertGPS(self, msg: NavSatFix):
        msg.header.frame_id = self.base_link_frame
        self.gps_pub.publish( msg)


if __name__ == "__main__":
    node = Convert()
    rospy.spin()
