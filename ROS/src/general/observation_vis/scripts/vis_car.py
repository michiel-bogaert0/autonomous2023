#!/usr/bin/env python3
import time
from math import pi

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class VisCar:
    """
    This node visualises a pose (nav_msgs/Odometry) with a car model
    """

    def __init__(self):
        """
        Args:
            car_model: url to car model
            vis_namespace: the namespace to use
            vis_lifetime: how long a marker should stay alive
        """
        #ros initialization
        rospy.init_node(f"slam_vis_obs_{time.time()}")
        self.vis_subscriber = rospy.Subscriber("/input/vis",Odometry,self.handleOdom)
        self.vis_publisher = rospy.Publisher("/output/vis",MarkerArray,queue_size=10)

        self.car_model = rospy.get_param(
            "~car_model",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/pegasus.dae",
        )

        self.vis_namespace = rospy.get_param("~namespace", "slam_vis/car")
        self.vis_lifetime = rospy.get_param("~lifetime", 3)

    def handleOdom(self, msg: Odometry):
        """Handles nav_msgs/Odometry message

        Args:
            msg (Odometry): the message to visualise
        """

        marker_array = MarkerArray()

        marker = Marker()

        marker.header = msg.header
        marker.ns = self.vis_namespace

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = self.car_model
        marker.mesh_use_embedded_materials = True
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.a = 1

        marker.action = Marker.ADD

        marker.id = 0

        marker.pose = msg.pose.pose

        roll, pitch, yaw = euler_from_quaternion(
            [
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            ]
        )

        yaw += pi / 2

        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)

        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w

        marker.lifetime = rospy.Duration(self.vis_lifetime)

        marker_array.markers.append(marker)

        self.vis_publisher.publish(marker_array)


node = VisCar()
rospy.spin()