#!/usr/bin/env python3

import time
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode

# from node_fixture.node_manager import set_state_inactive
from ugr_msgs.msg import Boundaries
from utils.utils_mincurv import (  # opt_min_curv,;
    B_spline_smoothing,
    calc_ax_profile,
    calc_head_curv_an,
    calc_spline_lengths,
    calc_splines,
    calc_t_profile,
    calc_vel_profile,
    check_traj,
    create_raceline,
    iqp_handler,
    prep_track,
    result_plots,
)


class MinimumCurvature(ManagedNode):
    def __init__(self):
        super().__init__("minimum_curvature")
        self.spin()

    def doConfigure(self):
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.car_width = rospy.get_param("~car_width", 0.50)
        self.width_margin_left = rospy.get_param("~width_margin_left", 0.10)
        self.width_margin_right = rospy.get_param("~width_margin_right", 0.10)
        self.car_length = rospy.get_param("~car_length", 0.72)
        self.v_max = rospy.get_param("~v_max", 10.0)
        self.drag_coeff = rospy.get_param("~drag_coeff", 1.00)
        self.car_mass = rospy.get_param("~car_mass", 50.00)
        self.imported_bounds = rospy.get_param("~imported_bounds", True)
        self.curvlim = rospy.get_param("~curvlim", 0.5)

        self.stepsize_prep_trajectory = rospy.get_param(
            "~stepsize_prep_trajectory", 0.50
        )
        self.stepsize_prep_boundaries = rospy.get_param(
            "~stepsize_prep_boundaries", 0.10
        )
        self.smoothing_factor_prep_trajectory = rospy.get_param(
            "~smoothing_factor_prep_trajectory", 2.0
        )
        self.smoothing_factor_prep_boundaries = rospy.get_param(
            "~smoothing_factor_prep_boundaries", 0.1
        )
        self.stepsize_opt = rospy.get_param("~stepsize_opt", 0.25)
        self.stepsize_post = rospy.get_param("~stepsize_interp", 0.25)

        self.plot = rospy.get_param("~plot", True)
        self.vel = rospy.get_param("~vel", True)

        self.min_distance = rospy.get_param("~min_distance", 1.5)

        self.car_width_with_margins = (
            self.car_width + self.width_margin_left + self.width_margin_right
        )

        # Publishers for the path, path_extra, path_iqp and reference
        self.path_pub = super().AddPublisher("/output/path_iqp", Path, queue_size=10)
        self.path_extra_pub = super().AddPublisher(
            "/output/path_extra", Path, queue_size=10
        )
        self.path_iqp_pub = super().AddPublisher("/output/path", Path, queue_size=10)
        self.path_ref_pub = super().AddPublisher(
            "/output/path_ref", Path, queue_size=10
        )

        # Subscribers for the path and boundaries
        self.path_sub = rospy.Subscriber(
            "/input/path",
            Path,
            self.receive_new_path,
        )

        self.boundaries_sub = rospy.Subscriber(
            "/input/boundaries",
            Boundaries,
            self.receive_new_boundaries,
        )

        self.reference_line = np.array([])
        self.bound_left = np.array([])
        self.bound_right = np.array([])
        self.extra_smoothed = np.array([])
        self.header = None
        self.calculate = False

        # ggv and ax_max_machines are hardcoded for now
        """
        ggv describes the maximum accelerations of the vehicle on the axis of the velocity, from 0 to max speed, for positive acceleration (ax)
        and lateral direction (ay)

        """
        self.ggv = np.array(
            [
                [0.0, 3.0, 3.0],  # [v [m/s], ax [m/s^2], ay [m/s^2]]
                [0.5, 3.0, 3.0],
                [1.0, 3.0, 3.0],
                [1.5, 3.0, 3.0],
                [2.0, 3.0, 3.0],
                [2.5, 3.0, 3.0],
                [3.0, 3.0, 3.0],
                [3.5, 3.0, 3.0],
                [4.0, 3.0, 3.0],
                [4.5, 3.0, 3.0],
                [5.0, 3.0, 3.0],
                [5.5, 3.0, 3.0],
                [6.0, 3.0, 3.0],
                [6.5, 3.0, 3.0],
                [7.0, 3.0, 3.0],
                [7.5, 3.0, 3.0],
                [8.0, 3.0, 3.0],
                [8.5, 3.0, 3.0],
                [9.0, 3.0, 3.0],
                [9.5, 3.0, 3.0],
                [10.0, 3.0, 3.0],
            ]
        )

        """
        ax_max_machines describes the acceleration resources of the motor at certain velocity thresholds
        (values inbetween are interpolated linearly)

        """
        self.ax_max_machines = np.array(
            [
                [0.0, 3.0],  # [v [m/s], ax [m/s^2]]
                [0.5, 3.0],
                [1.0, 3.0],
                [1.5, 3.0],
                [2.0, 3.0],
                [2.5, 3.0],
                [3.0, 3.0],
                [3.5, 3.0],
                [4.0, 3.0],
                [4.5, 3.0],
                [5.0, 3.0],
                [5.5, 3.0],
                [6.0, 3.0],
                [6.5, 3.0],
                [7.0, 3.0],
                [7.5, 3.0],
                [8.0, 3.0],
                [8.5, 3.0],
                [9.0, 3.0],
                [9.5, 3.0],
                [10.0, 3.0],
            ]
        )

    def doActivate(self):
        self.calculate = True

    def receive_new_path(self, msg: Path):
        self.reference_line = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        )
        self.header = msg.header

    def receive_new_boundaries(self, msg: Boundaries):
        self.cones_left = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.left_boundary.poses]
        )
        self.cones_right = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.right_boundary.poses]
        )

    def active(self):
        if not self.calculate or self.reference_line.size == 0:
            return

        rospy.logerr("Calculating minimum curvature path")
        t_start = time.perf_counter()
        header = self.header

        # Preprocess the track
        t_start_prep = time.perf_counter()

        (
            prepped_track,
            bound_left,
            bound_right,
            coeffs_x,
            coeffs_y,
            M,
            normvec_normalized,
            psi_prepped_track,
            kappa_prepped_track,
            dkappa_prepped_track,
        ) = prep_track(
            trajectory=self.reference_line,
            cones_left=self.cones_left,
            cones_right=self.cones_right,
            min_left_distance=self.min_distance,
            min_right_distance=self.min_distance,
            stepsize_trajectory=self.stepsize_prep_trajectory,
            stepsize_boundaries=self.stepsize_prep_boundaries,
            sf_trajectory=self.smoothing_factor_prep_trajectory,
            sf_boundaries=self.smoothing_factor_prep_boundaries,
            debug_info=False,
            plot_prep=True,
        )

        rospy.logwarn(
            f"Total time elapsed during preprocessing: {time.perf_counter() - t_start_prep}"
        )

        # IQP minimum curvature optimization

        t_start_iqp_optimization = perf_counter()
        (
            alpha_mincurv_iqp,
            reftrack_interp_iqp,
            normvec_normalized_iqp,
            spline_len_iqp,
            psi_reftrack_iqp,
            kappa_reftrack_iqp,
            dkappa_reftrack_iqp,
        ) = iqp_handler(
            reftrack=prepped_track,
            normvectors=normvec_normalized,
            A=M,
            spline_len=calc_spline_lengths(coeffs_x, coeffs_y),
            psi=psi_prepped_track,
            kappa=kappa_prepped_track,
            dkappa=dkappa_prepped_track,
            kappa_bound=0.5,
            w_veh=self.car_width_with_margins,
            print_debug=False,
            plot_debug=False,
            stepsize_interp=self.stepsize_opt,
            iters_min=2,
            curv_error_allowed=0.05,
        )
        rospy.logerr(
            f"Total time elapsed during iqp optimisation: {time.perf_counter() - t_start_iqp_optimization}"
        )

        if self.vel:
            vel_start = time.perf_counter()
            # Interpolate splines to small distances between raceline points
            (
                raceline_interp,
                a_opt,
                coeffs_x_opt,
                coeffs_y_opt,
                spline_inds_opt_interp,
                t_vals_opt_interp,
                s_points_opt_interp,
                spline_lengths_opt,
                el_lengths_opt_interp,
            ) = create_raceline(
                refline=reftrack_interp_iqp[:, :2],
                normvectors=normvec_normalized_iqp,
                alpha=alpha_mincurv_iqp,
                stepsize_interp=self.stepsize_opt,
            )
            # rospy.logerr(f"Raceline interpolated: {raceline_interp}")

            # Calculate heading and curvature
            psi_vel_opt, kappa_opt = calc_head_curv_an(
                coeffs_x=coeffs_x_opt,
                coeffs_y=coeffs_y_opt,
                ind_spls=spline_inds_opt_interp,
                t_spls=t_vals_opt_interp,
            )

            # rospy.logerr(f"Heading: {psi_vel_opt}")
            # rospy.logerr(f"Curvature: {kappa_opt}")

            # Calculate velocity and acceleration profile
            vx_profile_opt = calc_vel_profile(
                ggv=self.ggv,
                ax_max_machines=self.ax_max_machines,
                drag_coeff=self.drag_coeff,
                m_veh=self.car_mass,
                kappa=kappa_opt,
                el_lengths=el_lengths_opt_interp,
                closed=True,
            )
            # rospy.logerr(f"vx_profile: {vx_profile_opt}")

            # Calculate longitudinal acceleration profile
            vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
            ax_profile_opt = calc_ax_profile(
                vx_profile=vx_profile_opt_cl,
                el_lengths=el_lengths_opt_interp,
                eq_length_output=False,
            )
            # rospy.logerr(f"ax_profile: {ax_profile_opt}")

            # Calculate laptime
            t_profile_cl = calc_t_profile(
                vx_profile=vx_profile_opt,
                ax_profile=ax_profile_opt,
                el_lengths=el_lengths_opt_interp,
            )
            rospy.logerr(f"Estimated laptime: {t_profile_cl[-1]}")
            rospy.logerr(
                f"Total time elapsed during velocity and acceleration profile calculation: {time.perf_counter() - vel_start}"
            )

            # Data postprocessing
            trajectory_opt = np.column_stack(
                (
                    s_points_opt_interp,
                    raceline_interp,
                    psi_vel_opt,
                    kappa_opt,
                    vx_profile_opt,
                    ax_profile_opt,
                )
            )
            spline_data_opt = np.column_stack(
                (spline_lengths_opt, coeffs_x_opt, coeffs_y_opt)
            )

            traj_race_cl = np.vstack((trajectory_opt, trajectory_opt[0, :]))
            traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

            rospy.logerr(
                f"Runtime from start to final trajectory was {time.perf_counter() - t_start}"
            )

            bound1, bound2 = check_traj(
                reftrack=reftrack_interp_iqp,
                reftrack_normvec_normalized=normvec_normalized_iqp,
                length_veh=self.car_length,
                width_veh=self.car_width,
                debug=True,
                trajectory=traj_race_cl,
                ggv=self.ggv,
                ax_max_machines=self.ax_max_machines,
                v_max=self.v_max,
                curvlim=self.curvlim,
                mass_veh=self.car_mass,
                dragcoeff=self.drag_coeff,
            )

        if self.plot:
            s_points = np.cumsum(el_lengths_opt_interp[:-1])
            s_points = np.insert(s_points, 0, 0.0)

            plt.plot(s_points, vx_profile_opt)
            plt.plot(s_points, ax_profile_opt)

            plt.grid()
            plt.xlabel("distance in m")
            plt.legend(["vx in m/s", "ax in m/s2"])

            self.bound1_imp = None
            self.bound2_imp = None

            if self.imported_bounds:
                # try to extract four times as many points as in the interpolated version (in order to hold more details)
                n_skip = max(
                    int(self.reference_line.shape[0] / (bound1.shape[0] * 4)), 1
                )

                _, _, _, normvec_imp = calc_splines(
                    path=np.vstack(
                        (
                            self.reference_line[::n_skip, 0:2],
                            self.reference_line[0, 0:2],
                        )
                    )
                )

                self.bound1_imp = self.reference_line[
                    ::n_skip, :2
                ] + normvec_imp * np.expand_dims(self.reference_line[::n_skip, 2], 1)
                self.bound2_imp = self.reference_line[
                    ::n_skip, :2
                ] - normvec_imp * np.expand_dims(self.reference_line[::n_skip, 3], 1)

            # Loopclosure for plots
            trajectory_opt = np.vstack((trajectory_opt, trajectory_opt[0]))
            self.bound_left = np.vstack((self.bound_left, self.bound_left[0]))
            self.bound_right = np.vstack((self.bound_right, self.bound_right[0]))
            self.bound1_imp = np.vstack((self.bound1_imp, self.bound1_imp[0]))
            self.bound2_imp = np.vstack((self.bound2_imp, self.bound2_imp[0]))
            bound1 = np.vstack((bound1, bound1[0]))
            bound2 = np.vstack((bound2, bound2[0]))

            # Plot results
            self.plot_opts = {
                "mincurv_curv_lin": True,  # plot curv. linearization (original and solution based) (mincurv only)
                "raceline": True,  # plot optimized path
                "imported_bounds": True,  # plot imported bounds (analyze difference to interpolated bounds)
                "raceline_curv": True,  # plot curvature profile of optimized path
                "racetraj_vel": True,  # plot velocity profile
                "racetraj_vel_3d": True,  # plot 3D velocity profile above raceline
                "racetraj_vel_3d_stepsize": 1.0,  # [m] vertical lines stepsize in 3D velocity profile plot
                "spline_normals": True,  # plot spline normals to check for crossings
                "mintime_plots": False,
            }  # plot states, controls, friction coeffs etc. (mintime only)

            result_plots(
                plot_opts=self.plot_opts,
                width_veh_opt=self.car_width_with_margins,
                width_veh_real=self.car_width,
                refline=prepped_track[:, :2],
                bound1_imp=self.bound1_imp,
                bound2_imp=self.bound2_imp,
                bound1_interp=bound1,
                bound2_interp=bound2,
                trajectory=trajectory_opt,
                cones_left=self.cones_left,
                cones_right=self.cones_right,
                bound_left=self.bound_left,
                bound_right=self.bound_right,
            )

        # alpha_mincurv, curv_error_max = opt_min_curv(
        #     reftrack=self.reference_line,
        #     normvectors=normvec_normalized,
        #     A=M,
        #     kappa_bound=0.3,
        #     w_veh=self.car_width,
        #     print_debug=False,
        #     plot_debug=False,
        #     closed=True,
        #     psi_s=False,
        #     psi_e=False,
        #     fix_s=False,
        #     fix_e=False,
        # )

        # alpha_mincurv_extra, curv_error_max_extra = opt_min_curv(
        #     reftrack=self.extra_smoothed,
        #     normvectors=normvec_normalized_extra,
        #     A=M_extra,
        #     kappa_bound=0.2,
        #     w_veh=self.car_width,
        #     print_debug=False,
        #     plot_debug=False,
        #     closed=True,
        #     psi_s=False,
        #     psi_e=False,
        #     fix_s=False,
        #     fix_e=False,
        # )

        # path_result = self.reference_line[:, 0:2] + normvec_normalized * np.expand_dims(
        #     alpha_mincurv, axis=1
        # )

        # path_result_extra = self.extra_smoothed[
        #     :, 0:2
        # ] + normvec_normalized_extra * np.expand_dims(alpha_mincurv_extra, axis=1)

        path_result_iqp = reftrack_interp_iqp[
            :, 0:2
        ] + normvec_normalized_iqp * np.expand_dims(alpha_mincurv_iqp, axis=1)

        # smoothed_path = B_spline_smoothing(path_result, 2)
        # path_extra = B_spline_smoothing(path_result_extra, 2)
        path_iqp = B_spline_smoothing(path_result_iqp, smoothing_factor=2)
        # path_ref = B_spline_smoothing(self.path_ref, 2)

        path_iqp = np.vstack((path_iqp, path_iqp[0]))

        # smoothed_msg = Path()
        # smoothed_msg.poses = []
        # for point in smoothed_path:
        #     pose = PoseStamped()
        #     pose.pose.position.x = point[0]
        #     pose.pose.position.y = point[1]
        #     pose.header = header
        #     smoothed_msg.poses.append(pose)
        # smoothed_msg.header = header
        # self.path_pub.publish(smoothed_msg)

        # extra_msg = Path()
        # extra_msg.poses = []
        # for point in path_extra:
        #     pose = PoseStamped()
        #     pose.pose.position.x = point[0]
        #     pose.pose.position.y = point[1]
        #     pose.header = header
        #     extra_msg.poses.append(pose)
        # extra_msg.header = header
        # self.path_extra_pub.publish(extra_msg)

        iqp_msg = Path()
        iqp_msg.poses = []
        for point in path_iqp:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.header = header
            iqp_msg.poses.append(pose)
        iqp_msg.header = header
        self.path_iqp_pub.publish(iqp_msg)
        rospy.logerr("Minimum Curvature path was published!")
        # set_state_inactive("pathplanning")

        # ref_msg = Path()
        # ref_msg.poses = []
        # for point in path_ref:
        #     pose = PoseStamped()
        #     pose.pose.position.x = point[0]
        #     pose.pose.position.y = point[1]
        #     pose.header = header
        #     ref_msg.poses.append(pose)
        # ref_msg.header = header
        # self.path_ref_pub.publish(ref_msg)
        if self.plot:
            plt.show()
        self.calculate = False

        # TODO: Fix ggv acceleration limits
        # TODO: Fix better values for drag, weight, etc...
        # TODO: Fix loop_closure in plots


MinimumCurvature()
