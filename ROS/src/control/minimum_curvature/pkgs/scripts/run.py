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

        # Publishers for the path and path_extra
        self.path_pub = super().AddPublisher("/output/path", Path, queue_size=10)
        self.path_extra_pub = super().AddPublisher(
            "/output/path_extra", Path, queue_size=10
        )

        # Subscribers for the path
        self.path_sub = rospy.Subscriber(
            "/input/path",
            Path,
            self.receive_new_path,
        )
        self.reference_line = np.array([])
        self.extra_smoothed = np.array([])
        self.header = None
        self.calculate = False

    def doActivate(self):
        self.calculate = True

    def receive_new_path(self, msg: Path):
        self.reference_line = np.array(
            [[p.pose.position.x, p.pose.position.y, 1.50, 1.50] for p in msg.poses]
        )
        self.header = msg.header

    def active(self):
        if not self.calculate or self.reference_line.size == 0:
            return
        path = self.reference_line
        header = self.header
        self.reference_line = generate_interpolated_points(path)
        reference = np.copy(self.reference_line)
        self.extra_smoothed = np.hstack(
            (B_spline_smoothing(reference[:, 0:2], 2), reference[:, 2:4])
        )

        coeffs_x, coeffs_y, M, normvec_normalized = calc_splines(
            path=np.vstack((self.reference_line[:, 0:2], self.reference_line[0, 0:2]))
        )

        (
            coeffs_x_extra,
            coeffs_y_extra,
            M_extra,
            normvec_normalized_extra,
        ) = calc_splines(
            path=np.vstack((self.extra_smoothed[:, 0:2], self.extra_smoothed[0, 0:2]))
        )

        print(self.reference_line)
        print(type(self.reference_line))
        print("\n")
        # print(self.extra_smoothed)
        # print(type(self.extra_smoothed))

        # print(self.extra_smoothed.size)
        # print(len(normvec_normalized_extra))

        alpha_mincurv, curv_error_max = opt_min_curv(
            reftrack=self.reference_line,
            normvectors=normvec_normalized,
            A=M,
            kappa_bound=0.2,
            w_veh=self.car_width,
            closed=True,
        )

        alpha_mincurv_extra, curv_error_max_extra = opt_min_curv(
            reftrack=self.extra_smoothed,
            normvectors=normvec_normalized_extra,
            A=M_extra,
            kappa_bound=0.2,
            w_veh=self.car_width,
            closed=True,
        )

        path_result = self.reference_line[:, 0:2] + normvec_normalized * np.expand_dims(
            alpha_mincurv, axis=1
        )

        path_result_extra = self.extra_smoothed[
            :, 0:2
        ] + normvec_normalized_extra * np.expand_dims(alpha_mincurv_extra, axis=1)

        print(path_result)
        smoothed_path = B_spline_smoothing(path_result, 2)
        path_extra = B_spline_smoothing(path_result_extra, 2)

        smoothed_msg = Path()
        smoothed_msg.poses = []
        for point in smoothed_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.header = header
            smoothed_msg.poses.append(pose)
        smoothed_msg.header = header
        self.path_pub.publish(smoothed_msg)

        extra_msg = Path()
        extra_msg.poses = []
        for point in path_extra:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.header = header
            extra_msg.poses.append(pose)
        extra_msg.header = header
        self.path_extra_pub.publish(extra_msg)

        self.calculate = False


MinimumCurvature()
