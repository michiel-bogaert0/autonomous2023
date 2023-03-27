#!/usr/bin/env python3
import math
from tkinter.tix import Tree

import numpy as np
import rospy
import tf
from fs_msgs.msg import Track
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, ROSNode
from sensor_msgs.msg import Imu, NavSatFix


class Convert(ROSNode):
    def __init__(self):

        super().__init__("converter")

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

        print("Convert FSDS Started!")

    @AddSubscriber("/input/gss")
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

        self.publish("/output/gss", new_msg)

    @AddSubscriber("/input/imu")
    def convertIMU(self, msg: Imu):
        msg.header.frame_id = self.base_link_frame

        self.publish("/output/imu", msg)

    @AddSubscriber("/input/gt_odometry")
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

        self.publish("/output/gt_odometry", msg)

    @AddSubscriber("/input/gt_track")
    def convertTrack(self, msg: Track):
        self.publish("/output/gt_track", msg)

    @AddSubscriber("/fsds/gps")
    def convertGPS(self, msg: NavSatFix):
        msg.header.frame_id = self.base_link_frame
        self.publish("/output/gps", msg)


node = Convert()
node.start()
