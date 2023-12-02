#!/usr/bin/env python3

# import numpy as np
# import rospy
from sklearn.neighbors import KDTree
from ugr_msgs.msg import (  # ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class NaiveFusion:
    def __init__(self, euclidean_max_fusion_distance):
        self.euclidean_max_fusion_distance = euclidean_max_fusion_distance
        return

    def fuse_observations(self, tf_sensor_msgs):
        """
        Fuse observations using a naive approach
        """
        results = ObservationWithCovarianceArrayStamped()
        # tbd

        return results

    def kd_tree_merger(self, tf_sensor_msgs):
        """
        KDTree data association

        Distances between (transformed) observations are calculated using a KDTree. Detections of
        different sensors are then linked together based on their euclidean distance.
        Isolated cones are treated seperately and are not linked to any other cone.
        """

        # Create list of all observations & points of all incoming messages
        all_observations = []
        for msg in tf_sensor_msgs:
            all_observations.extend(msg.observations)

        all_points = list(
            map(
                lambda obs: [
                    obs.observation.location.x,
                    obs.observation.location.y,
                    obs.observation.location.z,
                ],
                all_observations,
            )
        )

        # Create KDTree of all points
        kdtree_all = KDTree(all_points)
        # Find matching observations

        return kdtree_all

    def kalman_filter(self):
        """
        Kalman filter

        Associated observations are merged using a Kalman filter
        to find an optimal estimate for cone location, type and covariance.
        """
        return
