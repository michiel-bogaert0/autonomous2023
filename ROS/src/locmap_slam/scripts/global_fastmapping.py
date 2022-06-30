#!/usr/bin/env python3

from collections import deque

import numpy as np
import rospy
from fastmapping.fastmapping import FastMapping
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from slam.helpers import observations_to_range_bearings
from slam.slam import SLAMNode
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import Observation, Observations


class GlobalFastMapping(SLAMNode):
    def __init__(self):
        super().__init__("global_fastmapping")

        # Extra fastmapping parameters
        self.meas_cov = rospy.get_param(
            "~fastmapping/measurement_covariance", [0.05, 0, 0, 0.001]
        )
        self.threshold_distance = rospy.get_param("~fastmapping/threshold_distance", 2)

        print(self.threshold_distance)
        print(self.meas_cov)

        self.fastmapping = FastMapping(
            self.threshold_distance,
            np.array(self.meas_cov).reshape((2, 2))
        )

        self.observations_receive_buffer = deque([])
        self.observations_processing_buffer = deque([])

        self.previous_timestamp = 0

    def get_predictions(self):

        map = self.fastmapping.get_map()

        return self.particle_state, map.get_landmarks(), map.get_landmark_classes(),  [], [self.particle_state], [1]

    def process_odometry(self, odometry: Odometry):
        pass

    def process_observations(self, observations: Observations):

        # Get the pose to feed to FastMapping (of course using the correct time)
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            self.world_frame, self.base_link_frame, observations.header.stamp
        )

        _, _, yaw = euler_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )

        self.particle_state = np.array(
            [transform.transform.translation.x, transform.transform.translation.y, yaw]
        )
        obs_rb = observations_to_range_bearings(observations)

        self.fastmapping.step(self.particle_state, obs_rb)

        self.publish_result()


node = GlobalFastMapping()
node.start()
