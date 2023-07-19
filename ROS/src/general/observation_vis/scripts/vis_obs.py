#!/usr/bin/env python3
import time

import rospy
from node_fixture.node_fixture import AddSubscriber, ROSNode
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from obs_vis import ObsVis

class ObservationVisualiser(ROSNode):
    """
    This node visualises ugr_msgs/ObservationWithCovarianceArrayStamped (both local and global, doesn't matter)
    """

    def __init__(self):
        """
        Args:
            blue_cone_model_url: url to blue cone model
            yellow_cone_model_url: url to yellow cone model
            use_cones: if True, uses cone models, otherwise uses cylinders
            use_covariance: if True, uses observation covariances to adjust the scale of the cylinders
            vis_namespace: the namespace to use
            vis_lifetime: how long a marker should stay alive
        """
        super().__init__(f"slam_vis_obs_{time.time()}", False)

        self.blue_cone_model_url = rospy.get_param(
            "~cone_models/blue",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/blue_cone_final.dae",
        )
        self.yellow_cone_model_url = rospy.get_param(
            "~cone_models/yellow",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/yellow_cone_final.dae",
        )

        self.use_cones = rospy.get_param("~use_cones", True)
        self.use_covariance = rospy.get_param("~use_covariance", False)

        self.vis_namespace = rospy.get_param("~namespace", "slam_vis")
        self.vis_lifetime = rospy.get_param("~lifetime", 3)
        self.scale = rospy.get_param("~scale", 1)

        self.vis_handler = ObsVis(
            [self.blue_cone_model_url, self.yellow_cone_model_url]
        )

        self.add_subscribers()

    @AddSubscriber("/input/vis")
    def handleObs(self, msg: ObservationWithCovarianceArrayStamped):
        """Handles ugr_mgs/Observations message

        Args:
            msg (ObservationWithCovarianceArrayStamped): the message to visualise
        """
        empty_array = self.vis_handler.delete_markerarray(self.vis_namespace)
        self.publish("/output/vis", empty_array)

        marker_array = self.vis_handler.observations_to_markerarray(
            msg,
            self.vis_namespace,
            0,
            False,
            self.use_cones,
            self.use_covariance,
            self.scale,
        )

        self.publish("/output/vis", marker_array)


node = ObservationVisualiser()
node.start()
