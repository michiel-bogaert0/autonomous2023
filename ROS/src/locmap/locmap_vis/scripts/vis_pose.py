#!/usr/bin/env python3
import rospy
from locmap_vis import LocMapVis
from node_fixture.node_fixture import AddSubscriber, ROSNode
from ugr_msgs.msg import Observations
import time

class VisPose(ROSNode):
    """
    This node connects locmap to other nodes when a simple remap is not enough.
    """

    def __init__(self):

        super().__init__(f"locmap_vis_car_{time.time()}")
        
        self.blue_cone_model_url = rospy.get_param(
            "~cone_models/blue",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/blue_cone_final.dae",
        )
        self.yellow_cone_model_url = rospy.get_param(
            "~cone_models/yellow",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/yellow_cone_final.dae",
        )

        self.use_cones = rospy.get_param("~use_cones", True)        

        self.vis_namespace = rospy.get_param("~namespace", "locmap_vis")
        self.vis_lifetime = rospy.get_param("~lifetime", 3)
        
        self.vis_sample_color = rospy.get_param("~sample_color", "g")
        self.vis_handler = LocMapVis(
            [self.blue_cone_model_url, self.yellow_cone_model_url]
        )

    @AddSubscriber("/input/vis")
    def handleOdom(self, msg: Observations):
    
        marker_array = self.vis_handler.observations_to_markerarray(
            msg, self.vis_namespace + "/observations", 0, False, self.use_cones
        )
        
        self.publish("/output/vis", marker_array)

node = VisPose()
node.start()
