#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, ROSNode
from visualization_msgs.msg import Marker, MarkerArray
import time

class VisCar(ROSNode):
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

        super().__init__(f"locmap_vis_obs_{time.time()}")
        
        self.car_model = rospy.get_param(
            "~car_model",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/pegasus.dae",
        )

        self.vis_namespace = rospy.get_param("~namespace", "locmap_vis/car")
        self.vis_lifetime = rospy.get_param("~lifetime", 3)

    @AddSubscriber("/input/vis")
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

        marker.lifetime = rospy.Duration(self.vis_lifetime)

        marker_array.markers.append(marker)
        
        self.publish("/output/vis", marker_array)

node = VisCar()
node.start()
