#!/usr/bin/env python3
# import trajectory_planning_helpers as tph

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from utils.utils_mincurv import (
    B_spline_smoothing,
    calc_splines,
    generate_center_points,
    generate_interpolated_points,
    opt_min_curv,
)


class MinimumCurvature(ManagedNode):
    def __init__(self):
        super().__init__("minimum_curvature")
        self.spin()

    def doConfigure(self):
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.car_width = rospy.get_param("~car_width", 0.5)

        # Publishers for the path and velocity

        self.path_pub = super().AddPublisher("/output/path", Path, queue_size=10)

    def doActivate(self):
        self.map_sub = super().AddSubscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )

    def receive_new_map(self, map):
        cones = np.zeros((len(map.observations), 3))
        for i, obs in enumerate(map.observations):
            cones[i] = [
                obs.observation.location.x,
                obs.observation.location.y,
                obs.observation.observation_class,
            ]

        self.compute(cones, map.header)

    def compute(self, cones, header, msg=Path):
        self.blue_cones = []
        self.yellow_cones = []
        self.center_points = []
        for cone in cones:
            if cone[2] == 0:
                self.blue_cones.append([cone[0], cone[1]])
            elif cone[2] == 1:
                self.yellow_cones.append([cone[0], cone[1]])
        self.center_points = generate_center_points(self.blue_cones, self.yellow_cones)
        self.reference_line = generate_interpolated_points(self.center_points)

        coeffs_x, coeffs_y, M, normvec_normalized = calc_splines(
            path=np.vstack((self.reference_line[:, 0:2], self.reference_line[0, 0:2]))
        )

        alpha_mincurv, curv_error_max = opt_min_curv(
            reftrack=self.reference_line,
            normvectors=normvec_normalized,
            A=M,
            kappa_bound=0.2,
            w_veh=self.car_width,
            closed=True,
        )

        path_result = self.reference_line[:, 0:2] + normvec_normalized * np.expand_dims(
            alpha_mincurv, axis=1
        )

        smoothed_path = B_spline_smoothing(path_result)
        rospy.loginfo("got here 6")

        smoothed_msg = msg
        smoothed_msg.poses = []
        for point in smoothed_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

        self.path_pub.publish(msg)


MinimumCurvature()
