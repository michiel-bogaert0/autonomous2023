#!/usr/bin/env python3

import time
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from ugr_msgs.msg import Boundaries
from utils.utils_mincurv import (
    calc_ax_max_motors,
    calc_ax_profile,
    calc_ggv,
    calc_head_curv_an,
    calc_spline_lengths,
    calc_t_profile,
    calc_vel_profile,
    check_traj,
    create_raceline,
    iqp_handler,
    plot_optimisation,
    prep_track,
    result_plots,
    shortest_path_handler,
)


class MinimumCurvature(ManagedNode):
    def __init__(self):
        super().__init__("minimum_curvature")
        self.spin()

    def doConfigure(self):
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.car_width = rospy.get_param("~car_width", 1.49)
        self.car_length = rospy.get_param("~car_length", 3.21)
        self.car_mass = rospy.get_param("~car_mass", 270.00)

        self.width_margin_left = rospy.get_param("~width_margin_left", 0.15)
        self.width_margin_right = rospy.get_param("~width_margin_right", 0.15)
        self.cone_width = rospy.get_param("~cone_width", 0.24)
        self.min_track_width = rospy.get_param("~min_track_width", 3.00)

        self.car_width_with_margins = (
            self.car_width
            + self.width_margin_left
            + self.width_margin_right
            + 2 * self.cone_width / 2
        )

        self.kappa_max = rospy.get_param("~kappa_max", 0.50)
        self.curv_error_allowed = rospy.get_param("~curv_error_allowed", 0.10)

        self.stepsize_prep_trajectory = rospy.get_param(
            "~stepsize_prep_trajectory", 0.50
        )
        self.stepsize_prep_boundaries = rospy.get_param(
            "~stepsize_prep_boundaries", 0.25
        )
        self.smoothing_factor_prep_trajectory = rospy.get_param(
            "~smoothing_factor_prep_trajectory", 2.0
        )
        self.smoothing_factor_prep_boundaries = rospy.get_param(
            "~smoothing_factor_prep_boundaries", 0.1
        )

        self.stepsize_opt = rospy.get_param("~stepsize_opt", 0.50)
        self.stepsize_post = rospy.get_param("~stepsize_interp", 0.25)

        self.min_iterations_iqp = rospy.get_param("~min_iterations_iqp", 3)
        self.max_iterations_iqp = rospy.get_param("~max_iterations_iqp", 5)

        self.calc_shortest_path = rospy.get_param("~calc_shortest_path", False)

        self.vel_calc = rospy.get_param("~vel_calc", False)
        self.v_max = rospy.get_param("~v_max", 36.00)
        self.T_motor_max = rospy.get_param("~T_motor_max", 140.00)
        self.P_max = rospy.get_param("~P_max", 80000.00)
        self.num_motors = rospy.get_param("~num_motors", 2)
        self.gear_ratio = rospy.get_param("~gear_ratio", 3.405)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.206)
        self.drag_coeff = rospy.get_param("~drag_coeff", 1.649217)
        self.acc_limit_long = rospy.get_param("~acc_limit_long", 15.69)
        self.acc_limit_lat = rospy.get_param("~acc_limit_lat", 15.69)

        # Data needed for velocity profile calculation
        self.vel_data = {
            "car_width": self.car_width,
            "car_length": self.car_length,
            "car_mass": self.car_mass,
            "v_max": self.v_max,
            "T_motor_max": self.T_motor_max,
            "P_max": self.P_max,
            "num_motors": self.num_motors,
            "gear_ratio": self.gear_ratio,
            "wheel_radius": self.wheel_radius,
            "drag_coeff": self.drag_coeff,
            "acc_limit_long": self.acc_limit_long,
            "acc_limit_lat": self.acc_limit_lat,
        }

        # Plotting and debugging
        self.plot = rospy.get_param("~plot", False)
        self.print_debug = rospy.get_param("~print_debug", False)

        if self.plot:
            self.plot_prep = rospy.get_param("~plot_prep", False)
            self.plot_opt = rospy.get_param("~plot_opt", False)
            self.plot_post = rospy.get_param("~plot_post", False)
            self.plot_final = rospy.get_param("~plot_final", False)
        else:
            self.plot_prep = False
            self.plot_opt_shpath = False
            self.plot_opt_iqp = False
            self.plot_post = False
            self.plot_final = False

        if self.print_debug:
            self.print_debug_prep = rospy.get_param("~print_debug_prep", False)
            self.print_debug_opt = rospy.get_param("~print_debug_opt", False)
            self.print_debug_post = rospy.get_param("~print_debug_post", False)
            self.print_debug_final = rospy.get_param("~print_debug_final", False)
        else:
            self.print_debug_prep = False
            self.print_debug_opt = False
            self.print_debug_post = False
            self.print_debug_final = False

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

        # Publishers for the minimum curvature path
        self.path_mincurv_pub = super().AddPublisher(
            "/output/path_mincurv", Path, queue_size=10
        )

        # Declare variables
        self.reference_line = np.array([])
        self.cones_left = np.array([])
        self.cones_right = np.array([])
        self.header = None

        # Variable to control single calculation
        self.calculate = False

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
        if (
            not self.calculate
            or self.reference_line.size == 0
            or self.cones_left.size == 0
            or self.cones_right.size == 0
        ):
            return

        rospy.loginfo("Path and boundaries received!")
        rospy.logerr("Starting minimum curvature calculation...")
        t_start = time.perf_counter()

        # ----------------------------------------------------------------------------------------------------------------------
        # PREPROCESSING TRACK --------------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        t_prep_start = time.perf_counter()
        rospy.loginfo("Starting preprocessing...")
        (
            prepped_track,
            self.bound_left,
            self.bound_right,
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
            min_track_width=self.min_track_width,
            stepsize_trajectory=self.stepsize_prep_trajectory,
            stepsize_boundaries=self.stepsize_prep_boundaries,
            sf_trajectory=self.smoothing_factor_prep_trajectory,
            sf_boundaries=self.smoothing_factor_prep_boundaries,
            debug_info=self.print_debug_prep,
            plot_prep=self.plot_prep,
        )
        rospy.loginfo("Preprocessing done!")

        rospy.logwarn(
            f"Total time elapsed during preprocessing: {time.perf_counter() - t_prep_start}"
        )

        # ----------------------------------------------------------------------------------------------------------------------
        # OPTIONAL: SHORTEST PATH OPTIMISATION ---------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        if self.calc_shortest_path:
            rospy.loginfo("Starting shortest path optimisation...")
            t_start_shortest_path_optimization = perf_counter()

            shortest_path_track = shortest_path_handler(
                width_veh_real=self.car_width,
                width_veh_opt=self.car_width_with_margins,
                ref_track_=prepped_track,
                ref_normvec_norm_=normvec_normalized,
                stepsize_shpath=self.stepsize_opt,
                cones_left_=self.cones_left,
                cones_right_=self.cones_right,
                initial_poses_=self.reference_line,
                ref_psi_=psi_prepped_track,
                ref_kappa_=kappa_prepped_track,
                ref_dkappa_=dkappa_prepped_track,
                bound_left_=self.bound_left,
                bound_right_=self.bound_right,
                debug_info=self.print_debug_opt,
                shpath_plot=self.plot_opt,
            )
            shortest_path_track = shortest_path_track

            rospy.loginfo("Shortest path optimisation done!")
            rospy.logwarn(
                f"Total time elapsed during shortest path optimisation: {time.perf_counter() - t_start_shortest_path_optimization}"
            )
        # ----------------------------------------------------------------------------------------------------------------------
        # IQP MINIMUM CURVATURE OPTIMISATION -----------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # TODO: Write function for IQP optimisation?

        t_start_iqp_optimization = perf_counter()
        (
            alpha_mincurv_iqp,
            track_iqp,
            normvec_normalized_iqp,
            spline_len_iqp,
            psi_track_iqp,
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
            kappa_bound=self.kappa_max,
            w_veh=self.car_width_with_margins,
            print_debug=self.print_debug_opt,
            plot_debug=self.plot_opt,
            stepsize_interp=self.stepsize_opt,
            iters_min=self.min_iterations_iqp,
            curv_error_allowed=self.curv_error_allowed,
        )
        rospy.logerr(
            f"Total time elapsed during iqp optimisation: {time.perf_counter() - t_start_iqp_optimization}"
        )

        if self.plot_opt:
            plot_opts = {
                "opt_method": "iqp_mincurv",
                "calc_vel": False,
                "plot_debug": False,
                "show_plots": True,
                "save_plots": False,
                "folder_path": "utils/data/plots/shortest_path_optimisation/",
            }
            print("Plotting iqp optimisation results...")

            plot_optimisation(
                cones_left_=self.cones_left,
                cones_right_=self.cones_right,
                initial_poses_=self.reference_line,
                width_veh_real=self.car_width,
                width_veh_opt=self.car_width_with_margins,
                ref_track_=prepped_track,
                ref_normvec_norm_=normvec_normalized,
                ref_psi_=psi_prepped_track,
                ref_kappa_=kappa_prepped_track,
                ref_dkappa_=dkappa_prepped_track,
                bound_left_=self.bound_left,
                bound_right_=self.bound_right,
                opt_track_=track_iqp,
                opt_normvec_norm_=normvec_normalized_iqp,
                opt_psi_=psi_track_iqp,
                opt_kappa_=kappa_reftrack_iqp,
                opt_dkappa_=dkappa_reftrack_iqp,
                plot_options=plot_opts,
            )

        if self.vel_calc:
            vel_start = time.perf_counter()

            ggv = calc_ggv(
                v_max=self.v_max,
                acc_limit_long=self.acc_limit_long,
                acc_limit_lat=self.acc_limit_lat,
            )

            ax_max_motors = calc_ax_max_motors(
                v_max=self.v_max,
                T_motor_max=self.T_motor_max,
                P_max=self.P_max,
                num_motors=self.num_motors,
                gear_ratio=self.gear_ratio,
                wheel_radius=self.wheel_radius,
                car_mass=self.car_mass,
            )

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
                refline=track_iqp[:, :2],
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
                ggv=ggv,
                ax_max_machines=ax_max_motors,
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
                reftrack=track_iqp,
                reftrack_normvec_normalized=normvec_normalized_iqp,
                length_veh=self.car_length,
                width_veh=self.car_width,
                debug=True,
                trajectory=traj_race_cl,
                ggv=ggv,
                ax_max_machines=ax_max_motors,
                v_max=self.v_max,
                curvlim=self.kappa_max,
                mass_veh=self.car_mass,
                dragcoeff=self.drag_coeff,
            )

        if self.plot_final:
            s_points = np.cumsum(el_lengths_opt_interp[:-1])
            s_points = np.insert(s_points, 0, 0.0)

            plt.plot(s_points, vx_profile_opt)
            plt.plot(s_points, ax_profile_opt)

            plt.grid()
            plt.xlabel("distance in m")
            plt.legend(["vx in m/s", "ax in m/s2"])

            self.bound1_imp = None
            self.bound2_imp = None

            # Loopclosure for plots
            trajectory_opt = np.vstack((trajectory_opt, trajectory_opt[0]))
            self.bound_left = np.vstack((self.bound_left, self.bound_left[0]))
            self.bound_right = np.vstack((self.bound_right, self.bound_right[0]))
            bound1 = np.vstack((bound1, bound1[0]))
            bound2 = np.vstack((bound2, bound2[0]))

            # Plot results
            self.plot_opts = {
                "mincurv_curv_lin": True,  # plot curv. linearization (original and solution based) (mincurv only)
                "raceline": True,  # plot optimized path
                "imported_bounds": False,  # plot imported bounds (analyze difference to interpolated bounds)
                "raceline_curv": True,  # plot curvature profile of optimized path
                "raceline_curv_3d": True,  # plot 3D curvature profile above raceline
                "raceline_curv_3d_stepsize": 1.0,  # [m] vertical lines stepsize in 3D curvature profile plot
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
                refline=self.reference_line[:, :2],
                bound1_imp=self.bound1_imp,
                bound2_imp=self.bound2_imp,
                bound1_interp=self.bound_left,
                bound2_interp=self.bound_right,
                trajectory=trajectory_opt,
                cones_left=self.cones_left,
                cones_right=self.cones_right,
                bound_left=self.bound_left,
                bound_right=self.bound_right,
            )

        path_mincurv = trajectory_opt[:, 1:3]

        iqp_msg = Path()
        iqp_msg.poses = []
        for point in path_mincurv:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.header = self.header
            iqp_msg.poses.append(pose)
        iqp_msg.header = self.header
        self.path_mincurv_pub.publish(iqp_msg)
        rospy.logerr("Minimum Curvature path was published!")

        self.calculate = False
        rospy.logerr(
            f"Total time elapsed during minimum curvature calculation: {time.perf_counter() - t_start}"
        )

        # TODO: Fix better values for drag
        # TODO: Fix loop_closure in plots


MinimumCurvature()
