#!/usr/bin/env python3

import casadi as cd
import numpy as np
import rospkg
import rospy
import yaml
from environments_gen.bicycle_model_spline import BicycleModelSpline
from matplotlib import pyplot as plt
from optimal_control_gen.MPC_generation import MPC_generation
from optimal_control_gen.ocp import Ocp
from scipy.interpolate import interp1d
from spline_utils import (  # get_boundary_constraints,
    create_spline,
    get_boundary_constraints_casadi,
    project_on_spline,
)


class MPC_gen:
    def __init__(self):
        rospy.init_node("MPC_generation_control")

        self.path_GT_centerline = rospy.get_param(
            "~input_path", "/data_gen/centerline_chicane.yaml"
        )
        self.path_GT_map = rospy.get_param("~input_path", "/data_gen/map_chicane.yaml")

        reverse = True

        # Get centerline from yaml
        self.GT_centerline = []

        arr = self.read_yaml(self.path_GT_centerline)
        for pose in arr["poses"]:
            self.GT_centerline.append(
                [pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]]
            )

        # Close centerline
        self.GT_centerline = np.roll(self.GT_centerline, -5, axis=0)
        self.GT_centerline = np.vstack((self.GT_centerline, self.GT_centerline[0]))

        self.GT_centerline = self.interpolate_arr(self.GT_centerline)

        # Reverse centerline because otherwise spline goes wrong way
        if reverse:
            self.GT_centerline = self.GT_centerline[::-1]

        # Get left and right boundaries from yaml
        self.GT_left_boundary = []
        self.GT_right_boundary = []

        arr = self.read_yaml(self.path_GT_map)
        for obs in arr["observations"]:
            if obs["observation"]["observation_class"] == 0:
                self.GT_left_boundary.append(
                    [
                        obs["observation"]["location"]["x"],
                        obs["observation"]["location"]["y"],
                    ]
                )
            elif obs["observation"]["observation_class"] == 1:
                self.GT_right_boundary.append(
                    [
                        obs["observation"]["location"]["x"],
                        obs["observation"]["location"]["y"],
                    ]
                )

        # Close boundaries
        self.GT_left_boundary.append(self.GT_left_boundary[0])
        self.GT_right_boundary.append(self.GT_right_boundary[0])

        self.GT_left_boundary = self.interpolate_arr(self.GT_left_boundary)
        self.GT_right_boundary = self.interpolate_arr(self.GT_right_boundary)

        # Reverse boundaries
        if reverse:
            self.GT_left_boundary = self.GT_left_boundary[::-1]
            self.GT_right_boundary = self.GT_right_boundary[::-1]

        spline_centerline, curve_centerline = create_spline(
            self.GT_centerline, "r", derivative=False, derivative2=False, plot=True
        )

        self.curve_centerline = curve_centerline
        self.der_centerline = curve_centerline.derivative(o=1)

        car_pos = np.squeeze(curve_centerline(0))
        self.start_pos = car_pos
        taus = np.linspace(0, 1, 10000)
        points_on_spline = curve_centerline(taus)
        der_points = self.der_centerline(taus)
        project_on_spline(points_on_spline, der_points, car_pos, plot=False)

        create_spline(self.GT_left_boundary, "b", plot=True)
        create_spline(self.GT_right_boundary, "y", plot=True)

        # get_boundary_constraints(curve_centerline, 0.2, 1.5, plot=True)

        # print(get_boundary_constraints_casadi(curve_centerline, [0.5, 1.0], 1.5, plot=False))
        # # print(get_boundary_constraints_casadi(curve_centerline, cd.SX([0.1, 0.5, 1.0]), 1.5, plot=False))

        # print(get_boundary_constraints(curve_centerline, 0.1, 1.5, plot=False))
        # print(get_boundary_constraints(curve_centerline, 0.5, 1.5, plot=False))
        # print(get_boundary_constraints(curve_centerline, 1.0, 1.5, plot=False))
        # plt.xlim(-40, 20)
        # plt.ylim(-10, 20)

        # plt.xlim(-20, 50)
        # plt.ylim(-60, 10)
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        # plt.show()

        self.initialize_MPC()

        # self.analyse_cost()

        self.run_mpc()

        # self.save_solution()

        rospy.spin()

    def read_yaml(self, path):
        path = rospkg.RosPack().get_path("mpc_gen") + path

        with open(path, "r") as file:
            arr = yaml.safe_load(file)
        return arr

    def interpolate_arr(self, arr, n=3):
        distance = np.cumsum(np.sqrt(np.sum(np.diff(arr, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        alpha = np.linspace(0, 1, len(arr) * n)
        interpolator = interp1d(distance, arr, kind="cubic", axis=0)
        arr = interpolator(alpha)

        return arr

    def initialize_MPC(self):
        self.car = BicycleModelSpline(dt=0.05)

        self.steering_joint_angle = 0
        self.u = [0, 0, 0]

        self.N = 80
        self.ocp = Ocp(
            self.car.nx,
            self.car.nu,
            curve=self.curve_centerline,
            N=self.N,
            F=self.car.F,
            T=self.car.dt * self.N,
            show_execution_time=True,
            silent=False,
            store_intermediate=True,
            threads=1,
        )
        self.mpc = MPC_generation(self.ocp)

        # State: x, y, heading, steering angle, velocity
        Qn = np.diag([8e-3, 8e-3, 0, 0, 0, 0])

        # Input: acceleration, velocity on steering angle and update to progress parameter
        R = np.diag([1e-5, 5e-3, -1e-1])
        R_delta = np.diag([1e-7, 5e-3, 5e-1])

        self.set_costs(Qn, R, R_delta)

        # constraints
        # TODO: get these from urdf model
        self.max_steering_angle = 5  # same as pegasus.urdf
        self.set_constraints(5, self.max_steering_angle)

    def analyse_cost(self):
        """
        Analyse the cost function
        """
        # X_closed_loop = np.load(
        #     rospkg.RosPack().get_path("mpc_gen") + "/data_gen/X_closed_loop.npy"
        # )
        # U_closed_loop = np.load(
        #     rospkg.RosPack().get_path("mpc_gen") + "/data_gen/U_closed_loop.npy"
        # )

        # Sc = np.zeros((1, self.N))

        # der_curve = self.curve_centerline.derivative(o=1)

        # qs = 1e-1
        # qss = 1e-1
        # qc = 1e-2
        # ql = 1e-2

        # R = np.diag([1e-7, 1e-3, -1e-1])
        # R_delta = np.diag([1e-7, 1e-3, 0])

        # total_cost = 0

        # for i in range(self.N):

        #     point_curve = np.squeeze(self.curve_centerline(X_closed_loop[5][i+1]))
        #     der_point = np.squeeze(der_curve(X_closed_loop[5][i+1]))

        #     print(point_curve)
        #     print(der_point)
        #     phi = np.arctan2(der_point[1], der_point[0]) if np.abs(der_point[1]) > 1e-3 else 0
        #     print(np.degrees(phi))

        #     # Plot x-y and heading
        #     plt.plot(point_curve[0], point_curve[1], "o", c="r")
        #     plt.plot(
        #         [point_curve[0], point_curve[0] + np.cos(phi)],
        #         [point_curve[1], point_curve[1] + np.sin(phi)],
        #         c="m",
        #     )

        #     e_c = np.sin(phi) * (X_closed_loop[0][i] - point_curve[0]) - np.cos(phi) * (
        #         X_closed_loop[1][i] - point_curve[1]
        #     )
        #     e_l = -np.cos(phi) * (X_closed_loop[0][i] - point_curve[0]) - np.sin(phi) * (
        #         X_closed_loop[1][i] - point_curve[1]
        #     )

        #     if i == 0:
        #         u_delta = U_closed_loop[:, i]
        #     else:
        #         u_delta = U_closed_loop[:, i] - U_closed_loop[:, i-1]

        #     contouring_cost = qc * e_c**2
        #     lag_cost = ql * e_l**2
        #     input_cost = U_closed_loop[:, i] @ R @ U_closed_loop[:, i]
        #     input_delta_cost = u_delta @ R_delta @ u_delta
        #     relaxing_cost = qs * Sc + qss * Sc**2

        #     print(f"iteration {i} | contouring cost: {contouring_cost} | lag cost: {lag_cost} | input cost: {input_cost} | input delta cost: {input_delta_cost} | relaxing cost: {relaxing_cost}")

        # total_cost += cost
        plt.show()

        # print(total_cost)

    def set_constraints(self, velocity_limit, steering_limit):
        """
        Set constraints for the MPC
        """
        velocity_limit = 50
        steering_limit = 0.5
        self.wheelradius = 0.1
        self.ocp.subject_to()
        self.ocp.subject_to(
            self.ocp.bounded(
                -velocity_limit / self.wheelradius,
                self.ocp.U[0, :],
                velocity_limit / self.wheelradius,
            )
        )
        self.ocp.subject_to(
            self.ocp.bounded(-steering_limit, self.ocp.U[1, :], steering_limit)
        )
        # Limit angle of steering joint
        self.ocp.subject_to(self.ocp.bounded(-np.pi / 4, self.ocp.X[3, :], np.pi / 4))
        # Limit velocity
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.X[4, :], 30))

        # Progress parameter between 0 and 1
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.X[5, :], 1))

        # Limit update to progress parameter
        # Starts from 0.01 because otherwise casadi optimizes to very small negative values
        self.ocp.subject_to(self.ocp.bounded(0.000001, self.ocp.U[2, :], 0.05))

        # Limit relaxing constraint
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.Sc, 0.1))

        # Make sure neither x and y are zero at the same time by limiting square to be larger than 0
        # der_points = self.ocp.der_centerline(self.ocp.X[5, :].T)
        # self.ocp.subject_to(der_points[:,0] ** 2 + der_points[:, 1] ** 2 > 1e-1)

        # # For loop takes quite long to initialize
        center_points = self.ocp.centerline(self.ocp.X[5, :].T)
        for i in range(self.N + 1):
            self.ocp.subject_to(
                (
                    (
                        self.ocp.X[0, i]
                        # - self.ocp.centerline(self.ocp.X[5, i]).T[0]
                        - center_points[i, 0]
                    )
                    ** 2
                    + (
                        self.ocp.X[1, i]
                        # - self.ocp.centerline(self.ocp.X[5, i]).T[1]
                        - center_points[i, 1]
                    )
                    ** 2
                )
                < (1.5**2)  # + self.ocp.Sc[i] ** 2
            )

        # slopes_inner, intercepts_inner, slopes_outer, intercepts_outer = get_boundary_constraints_casadi(self.curve_centerline, self.der_centerline, self.ocp.X[5, :].T, 1.5)

        # self.ocp.subject_to(
        #     (
        #         slopes_inner * self.ocp.X[0, :].T
        #         + intercepts_inner
        #         - self.ocp.X[1, :].T
        #     )
        #     * (
        #         slopes_outer * self.ocp.X[0, :].T
        #         + intercepts_outer
        #         - self.ocp.X[1, :].T
        #     )
        #     < 0
        # )

        # self.ocp.subject_to(
        #     (
        #         self.ocp.slopes_inner * self.ocp.X[0, :]
        #         + self.ocp.intercepts_inner
        #         - self.ocp.X[1, :]
        #     )
        #     * (
        #         self.ocp.slopes_outer * self.ocp.X[0, :]
        #         + self.ocp.intercepts_outer
        #         - self.ocp.X[1, :]
        #     )
        #     < 0
        # )

        self.ocp._set_continuity(1)

        print("Constraints set")

    def set_costs(self, Qn, R, R_delta):
        """
        Set costs for the MPC
        """
        # qs = 0
        # qss = 0
        qc = 1e-2
        ql = 1e-2

        # State: x, y, heading, steering angle, velocity
        # Qn = np.diag([8e-3, 8e-3, 0, 0, 0])

        # Input: acceleration, velocity on steering angle, update to progress parameter
        # Maximize progress parameter
        R = np.diag([1e-2, 1e-2, 1e-2])
        R_delta = np.diag([1e-2, 1e-2, 1e-2])

        q_obj = 1e-2

        # Avoid division by zero
        # phi = cd.if_else(
        #     cd.fabs(self.ocp.der_curve[1]) < 1e-3,
        #     0,
        #     cd.atan2(self.ocp.der_curve[1], self.ocp.der_curve[0]),
        # )
        # Add small noise to avoid division by zero in derivative
        phi = cd.atan2(self.ocp.der_curve[1] + 1e-10, self.ocp.der_curve[0] + 1e-10)
        e_c = cd.sin(phi) * (self.ocp.x[0] - self.ocp.point_curve[0]) - cd.cos(phi) * (
            self.ocp.x[1] - self.ocp.point_curve[1]
        )
        e_l = -cd.cos(phi) * (self.ocp.x[0] - self.ocp.point_curve[0]) - cd.sin(phi) * (
            self.ocp.x[1] - self.ocp.point_curve[1]
        )
        obj = 1 - self.ocp.x[5]

        self.ocp.running_cost = (
            qc * e_c**2
            + ql * e_l**2
            + self.ocp.u.T @ R @ self.ocp.u
            + self.ocp.u_delta.T @ R_delta @ self.ocp.u_delta
            # + qs @ self.ocp.sc
            # + qss @ self.ocp.sc**2
            + q_obj * obj
        )

    def run_mpc(self):
        initial_state = [self.start_pos[0], self.start_pos[1], 0, 0, 20, 0.01]

        Tau0 = np.linspace(0.00001, 0.99999, self.N + 1)
        X0 = []
        U0 = []

        for tau in Tau0:
            x0 = self.curve_centerline(tau)[0]
            # heading = self.curve_centerline(tau)[1]
            steering_angle = 0

            # der_curve = self.curve_centerline.derivative(o=1)
            der_points = self.der_centerline(tau)[0]

            phi = np.arctan2(der_points[1], der_points[0])

            # calculate heading based on position
            # center_point = n
            # if len(X0) > 0:
            #     heading = np.arctan2(x0[1] - X0[-1][1], x0[0] - X0[-1][0])
            # else:
            #     heading = 0
            heading = phi

            # calculate steering angle based on position
            if len(X0) > 0:
                # Apply inverse bicycle model
                # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
                alpha = heading - X0[-1][2]
                l_d = np.sqrt((x0[0] - X0[-1][0]) ** 2 + (x0[1] - X0[-1][1]) ** 2)

                # Calculate turning radius
                R = (l_d) / (2 * np.sin(alpha))

                steering_angle = np.arctan(self.car.L / R)

                # R = ((x0[0] - X0[-1][0]) ** 2 + (x0[1] - X0[-1][1]) ** 2) / (
                #     2 * (x0[1] - X0[-1][1])
                # )
                # steering_angle = np.arctan2(0.72, R)
                # max_angle = np.pi / 4
                # steering_angle = (steering_angle + max_angle) % (
                #     2 * max_angle
                # ) - max_angle

                # Limit steering angle
                # if steering_angle > self.max_steering_angle:
                #     steering_angle = self.max_steering_angle
                # elif steering_angle < -self.max_steering_angle:
                #     steering_angle = -self.max_steering_angle
            else:
                steering_angle = 0

            # calculate velocity based on position
            if len(X0) > 0:
                velocity = (
                    np.sqrt((x0[0] - X0[-1][0]) ** 2 + (x0[1] - X0[-1][1]) ** 2)
                    / self.car.dt
                )
            else:
                velocity = 20
            # velocity = 50

            # calculate steering angle based on heading
            # if len(X0) > 0:
            #     # prev_steering_angle = X0[-1][3]
            #     steering_angle_delta = np.arctan2(
            #         np.sin(heading - prev_steering_angle),
            #         np.cos(heading - prev_steering_angle),
            #     )
            #     steering_angle = prev_steering_angle + steering_angle_delta
            # else:
            #     steering_angle = 0
            #     steering_angle_delta = 0
            # steering_angle = 0

            x0_state = [x0[0], x0[1], heading, steering_angle, velocity, tau]
            X0.append(x0_state)
            # X0.append(self.curve_centerline(tau).T)
            steering_angle_delta = steering_angle - X0[-1][3]

        # for i in range(len(X0)):
        #     steering_angle = 0
        #     if i == len(X0) - 1:
        #         steering_angle = X0[i-1][3]
        #     # calculate steering angle based on next and previous heading
        #     elif i > 0:
        #         steering_angle = (X0[i+1][2] - X0[i][2]) / self.car.dt

        #     X0[i][3] = steering_angle

        # Plot x-y and heading
        plt.plot([x[0] for x in X0], [x[1] for x in X0], c="b")

        # Plot heading at x-y points
        for i in range(len(X0)):
            plt.plot(
                [X0[i][0], X0[i][0] + np.cos(X0[i][2])],
                [X0[i][1], X0[i][1] + np.sin(X0[i][2])],
                c="r",
            )

        # Plot steering angle at x-y points
        for i in range(len(X0)):
            plt.plot(
                [X0[i][0], X0[i][0] + np.cos(X0[i][3] + X0[i][2])],
                [X0[i][1], X0[i][1] + np.sin(X0[i][3] + X0[i][2])],
                c="g",
            )

        for i in range(self.N):
            acceleration = (
                max(min((X0[i + 1][4] - X0[i][4]), 200), -200)
                / self.car.dt
                * self.wheelradius
            )
            # acceleration = (X0[i + 1][4] - X0[i][4]) / self.car.dt * self.wheelradius
            steering_angle_delta = (
                max(min((X0[i + 1][3] - X0[i][3]), 0.5), -0.5) / self.car.dt
            )
            steering_angle_delta = (X0[i + 1][3] - X0[i][3]) / self.car.dt
            dtau = X0[i + 1][5] - X0[i][5]
            U0.append([acceleration, steering_angle_delta, dtau])

        self.u = U0[0]
        initial_state = X0[0]
        # U0.append([0, 0, 0])
        X0 = np.array(X0).T
        U0 = np.array(U0).T

        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy", X0)
        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy", U0)

        print(X0)
        print(U0)

        (
            slope_inner,
            intercept_inner,
            slope_outer,
            intercept_outer,
        ) = get_boundary_constraints_casadi(
            self.curve_centerline, self.der_centerline, Tau0, 1.5, plot=False
        )

        # plt.show()

        u, info = self.mpc(
            initial_state,
            self.curve_centerline,
            slope_inner,
            intercept_inner,
            slope_outer,
            intercept_outer,
            self.u,
            X0=X0,
            U0=U0,
        )

        self.save_solution()

        # print(self.ocp.debug.value)

        print(f"X_closed_loop: {info['X_sol']}")
        print(f"U_closed_loop: {info['U_sol']}")

        # Save X and U to file
        # path = rospkg.RosPack().get_path("mpc_gen")
        # np.save(path + "/data_gen/X_closed_loop.npy", info["X_sol"])
        # np.save(path + "/data_gen/U_closed_loop.npy", info["U_sol"])

        x_sol = info["X_sol"][0][:]
        y_sol = info["X_sol"][1][:]

        x_sol = np.append(x_sol, x_sol[0])
        y_sol = np.append(y_sol, y_sol[0])

        x_traj = self.interpolate_arr(np.vstack((x_sol, y_sol)).T, n=3)

        x_traj = np.vstack((x_traj, x_traj[0]))
        # plt.plot(x_traj.T[0], x_traj.T[1], c="m")
        plt.plot(info["X_sol"][0][:], info["X_sol"][1][:], c="m")
        plt.plot(info["X_sol"][0][:], info["X_sol"][1][:], "o", c="m")
        print(info["X_sol"][5].shape)
        print(info["X_sol"][1].shape)

        for i in range(self.N + 1):
            plt.plot(
                self.curve_centerline(info["X_sol"][5][i]).T[0],
                self.curve_centerline(info["X_sol"][5][i]).T[1],
                "o",
                c="g",
            )
        get_boundary_constraints_casadi(
            self.curve_centerline,
            self.der_centerline,
            info["X_sol"][5][:],
            1.5,
            plot=False,
        )

        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.show()

    def save_solution(self):
        # Get convergence information
        inf_pr = self.ocp.debug.stats()["iterations"]["inf_pr"]
        inf_du = self.ocp.debug.stats()["iterations"]["inf_du"]
        inf_obj = self.ocp.debug.stats()["iterations"]["obj"]
        print(self.ocp.debug.show_infeasibilities())

        np.savez(
            "/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/solution.npz",
            U_sol_intermediate=self.mpc.U_sol_intermediate,
            X_sol_intermediate=self.mpc.X_sol_intermediate,
            info_pr=inf_pr,
            info_du=inf_du,
            info_obj=inf_obj,
        )


node = MPC_gen()
