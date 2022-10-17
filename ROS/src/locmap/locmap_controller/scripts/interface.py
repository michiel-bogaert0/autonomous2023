#!/usr/bin/env python3
import math
from tkinter.tix import Tree

import numpy as np
import rospy
import tf
from fs_msgs.msg import Cone, Track
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, ROSNode
from sensor_msgs.msg import Imu, NavSatFix
from ugr_msgs.msg import ObservationWithCovariance, ObservationWithCovarianceArrayStamped
from visualization_msgs.msg import MarkerArray


class Convert(ROSNode):
    """
    This node connects locmap to other nodes when a simple remap is not enough.
    """

    def __init__(self):

        super().__init__("locmap_external_interface")

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.gt_world_frame = rospy.get_param("~gt_world_frame", "ugr/gt_car_map")
        self.gt_base_link_frame = rospy.get_param(
            "~gt_base_link_frame", "ugr/gt_car_base_link"
        )

        self.br = tf.TransformBroadcaster()

        rospy.loginfo("LocMap Interface started!")

    @AddSubscriber("/input/gss")
    def convertGSS(self, msg: TwistStamped):
        """
        Converts generic speed sensor messages to the correct one to give to sensor fusion.
        It also converts the speed components to magnitude (in the X-direction) as wished by sensor fusion

        The header gets copied over, BUT the frame is changed to self.base_link_frame
        The timestamp stays the same

        Args:
            - msg: TwistStamped input message

        Returns:
            The converted speed message with constant covariance and velocity magnitude in the X-direction
        """

        covariance = np.diag([0.1, 0.1, 0, 0, 0, 0])

        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.header.frame_id = self.base_link_frame
        new_msg.twist.twist = msg.twist

        new_msg.twist.twist.linear.x = (
            msg.twist.linear.x**2 + msg.twist.linear.y**2
        ) ** (1 / 2)
        new_msg.twist.twist.linear.y = 0

        new_msg.twist.covariance = covariance.reshape((1, 36)).tolist()[0]

        self.publish("/output/gss", new_msg)

    @AddSubscriber("/input/imu")
    def convertIMU(self, msg: Imu):
        """
        Converts the IMU message. Basically only changes the frame to self.base_link_frame
        """

        msg.header.frame_id = self.base_link_frame

        self.publish("/output/imu", msg)

    @AddSubscriber("/input/gt_odometry")
    def convertGTOdometry(self, msg: Odometry):
        """
        Converts a generic Odometry message to something that is useable in LocMap.
        Basically converts the frame ids to the correct ones and is typically used for GT odometry coming from the simulator

        Also publishes a transformation using a TF broadcaster

        Args:
            - msg: the input Odometry message

        Returns:
            The converted Odometry message
        """

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
        """
        Converts a NavSatFix message. Basically only changes self.base_link_frame
        """
        msg.header.frame_id = self.base_link_frame
        self.publish("/output/gps", msg)


node = Convert()
node.start()
