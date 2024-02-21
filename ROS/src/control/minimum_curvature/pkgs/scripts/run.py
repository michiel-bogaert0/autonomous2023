#!/usr/bin/env python3
# import trajectory_planning_helpers as tph

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from utils.utils_mincurv import (
    B_spline_smoothing,
    calc_splines,
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
        # self.map_sub = super().AddSubscriber(
        #     "/input/local_map",
        #     ObservationWithCovarianceArrayStamped,
        #     self.receive_new_map,
        # )
        self.path_sub = super().AddSubscriber(
            "/input/path",
            Path,
            self.receive_new_path,
        )

    def receive_new_path(self, msg: Path):
        self.reference_line = np.array(
            [[p.pose.position.x, p.pose.position.y, 1.05, 1.05] for p in msg.poses]
        )
        self.compute(self.reference_line, msg.header)

    def compute(self, path, header):
        self.reference_line = generate_interpolated_points(path)

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
        msg = Path()
        smoothed_msg = msg
        smoothed_msg.poses = []
        for point in smoothed_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            smoothed_msg.poses.append(pose)
        self.path_pub.publish(msg)


MinimumCurvature()
