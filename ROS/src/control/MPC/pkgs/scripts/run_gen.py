#!/usr/bin/env python3

# from time import perf_counter

# import casadi as cd
import numpy as np
import rospkg
import rospy
import yaml
from environments.bicycle_model import BicycleModel
from matplotlib import pyplot as plt
from optimal_control.MPC_tracking import MPC_tracking
from optimal_control.ocp import Ocp
from scipy.interpolate import interp1d
from scripts.trajectory import Trajectory

# from utils.plotting_mpc import plot_velocity


class MPC_gen_ref:
    def __init__(self):
        rospy.init_node("MPC_gen_tracking")

        self.path_GT_centerline = rospy.get_param(
            "~input_path", "/data/centerline_fsi.yaml"
        )
        self.path_GT_map = rospy.get_param("~input_path", "/data/map_fsi.yaml")

        reverse = False

        # Get centerline from yaml
        self.GT_centerline = []

        arr = self.read_yaml(self.path_GT_centerline)
        for pose in arr["poses"]:
            self.GT_centerline.append(
                [pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]]
            )

        # Close centerline
        # self.GT_centerline = np.roll(self.GT_centerline, -5, axis=0)

        # self.GT_centerline = np.vstack((self.GT_centerline, self.GT_centerline[0]))

        # self.GT_centerline = self.interpolate_arr(self.GT_centerline)

        self.GT_centerline = np.array(self.GT_centerline)

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

        plt.plot(
            self.GT_centerline.T[0],
            self.GT_centerline.T[1],
            c="black",
            label="Centerline",
        )
        plt.plot(self.GT_centerline.T[0][0], self.GT_centerline.T[1][0], "o", c="black")

        plt.plot(self.GT_left_boundary.T[0], self.GT_left_boundary.T[1], c="b")
        plt.plot(self.GT_right_boundary.T[0], self.GT_right_boundary.T[1], c="y")

        # Reverse boundaries
        if reverse:
            self.GT_left_boundary = self.GT_left_boundary[::-1]
            self.GT_right_boundary = self.GT_right_boundary[::-1]

        self.target_speed = 10

        self.trajectory = Trajectory()
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")

        path = (
            rospkg.RosPack().get_path("mpc")
            + "/data/solution_gen_ref_halfspaces_fsg.npz"
        )
        data = np.load(path)
        X_sol_intermediate = data["X_sol_intermediate"]
        last_sol = X_sol_intermediate[-1]
        # plt.plot(last_sol[0], last_sol[1], "--", c="red", label="Solution reference-tracking with halfspaces")
        plt.plot(
            last_sol[0],
            last_sol[1],
            c="red",
            label="Reference-tracking with halfspaces",
        )

        path = (
            rospkg.RosPack().get_path("mpc") + "/data/solution_gen_ref_circles_fsg.npz"
        )
        data = np.load(path)
        X_sol_intermediate = data["X_sol_intermediate"]
        last_sol = X_sol_intermediate[-1]
        # plt.plot(last_sol[0], last_sol[1], "--", c="red", label="Solution reference-tracking with halfspaces")
        plt.plot(
            last_sol[0], last_sol[1], c="m", label="Reference-tracking with circles"
        )

        path = (
            rospkg.RosPack().get_path("mpc_gen")
            + "/data_gen/solution_gen_circles_fsg.npz"
        )
        data = np.load(path)
        X_sol_intermediate = data["X_sol_intermediate"]
        last_sol = X_sol_intermediate[-1]
        # plt.plot(last_sol[0], last_sol[1], "--", c="red", label="Solution reference-tracking with halfspaces")
        plt.plot(last_sol[0], last_sol[1], c="green", label="MPCC with circles")

        path = (
            rospkg.RosPack().get_path("mpc_gen")
            + "/data_gen/solution_gen_circles_lowN_fsg.npz"
        )
        data = np.load(path)
        X_sol_intermediate = data["X_sol_intermediate"]
        last_sol = X_sol_intermediate[-1]
        # plt.plot(last_sol[0], last_sol[1], "--", c="red", label="Solution reference-tracking with halfspaces")
        plt.plot(
            last_sol[0], last_sol[1], c="orange", label="MPCC with circles with tuned N"
        )

        # plt.show()

        self.initialize_MPC()

        # self.analyse_cost()

        self.run_mpc()

        # self.save_solution()

        rospy.spin()

    def read_yaml(self, path):
        path = rospkg.RosPack().get_path("mpc") + path

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
        self.car = BicycleModel(dt=0.05)

        self.steering_joint_angle = 0
        self.u = [0, 0]

        self.N = 434  # fsi
        self.N = 553  # fsc23
        # self.N = 616 # fsg
        # self.N = 1075
        self.ocp = Ocp(
            self.car.nx,
            self.car.nu,
            N=self.N,
            F=self.car.F,
            T=self.car.dt * self.N,
            show_execution_time=True,
            silent=False,
            store_intermediate=True,
            threads=1,
        )
        self.mpc = MPC_tracking(self.ocp)

        # State: x, y, heading, steering angle, velocity
        Qn = np.diag([8e-3, 8e-3, 0, 0, 0])

        # Input: acceleration, velocity on steering angle and update to progress parameter
        R = np.diag([1e-5, 5e-3])
        R_delta = np.diag([1e-7, 5e-3])

        self.set_costs(Qn, R, R_delta)

        # constraints
        # TODO: get these from urdf model
        self.max_steering_angle = 5  # same as pegasus.urdf
        self.set_constraints(5, self.max_steering_angle)

    def set_constraints(self, velocity_limit, steering_limit):
        """
        Set constraints for the MPC
        """
        velocity_limit = 20
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
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.X[4, :], 15))

        # Limit relaxing constraint
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.Sc, 1e-1))

        # This one works with circles, but causes convergence issues
        for i in range(self.N + 1):
            self.ocp.subject_to(
                (
                    (self.ocp.X[0, i] - self.ocp._x_reference[0, i]) ** 2
                    + (self.ocp.X[1, i] - self.ocp._x_reference[1, i]) ** 2
                )
                < (1.2**2)  # + self.ocp.Sc[i]
            )

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
        qs = 0
        qss = 0
        Qn = np.diag([5e-10, 5e-10, 0, 0, 0])
        R = np.diag([1e-5, 4e-3])
        R_delta = np.diag([1e-2, 2e-4])  # * self.actual_speed / self.speed_target]

        self.ocp.running_cost = (
            (self.ocp.x - self.ocp.x_reference).T
            @ Qn
            @ (self.ocp.x - self.ocp.x_reference)
            + self.ocp.u.T @ R @ self.ocp.u
            + self.ocp.u_delta.T @ R_delta @ self.ocp.u_delta
            + qs @ self.ocp.sc
            + qss @ self.ocp.sc**2
        )

    def get_reference_path(self, debug=False):
        self.path_blf = self.GT_centerline

        if len(self.path_blf) == 0:
            return []

        # Find target points based on required velocity and maximum acceleration
        speed_target = self.target_speed
        # max_acceleration = 2.0  # TODO: create param
        # actual_speed = 0
        # calculate distances based on maximum acceleration and current speed
        distances = [((speed_target)) * i * self.car.dt for i in range(self.N + 1)]
        self.closest_index = np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
        self.closest_index = 0

        if debug:
            current_point = self.path_blf[self.closest_index]
            print(current_point)

        # Shift path so that closest point is at index 0
        shifted_path = np.roll(self.path_blf, -self.closest_index, axis=0)
        # Compute the distance between consecutive points
        diff = np.diff(shifted_path, axis=0)
        distances_cumsum = np.linalg.norm(diff, axis=1)

        # Append 0 and calculate cummulative sum
        distances_relative = np.append([0], np.cumsum(distances_cumsum))

        # Find points at specified distances

        # start_time = perf_counter()
        i = 0
        j = 0
        reference_path = []
        while j < len(distances_relative) and i < len(distances):
            if distances_relative[j] < distances[i]:
                j += 1
                continue
            elif distances_relative[j] == distances[i]:
                # i += 1
                # j += 1
                scaling = 1
            elif distances_relative[j] > distances[i]:
                # Required distances between two points so scale between them
                # i += 1

                scaling = 1 - np.abs(
                    (distances[i] - distances_relative[j])
                    / (distances_relative[j] - distances_relative[j - 1])
                )

            reference_path.append(
                shifted_path[j - 1] + scaling * (shifted_path[j] - shifted_path[j - 1])
            )
            if debug:
                print(
                    f"Point {j - 1} and {j} with scaling {scaling} and distance {distances[i]} and relative distance {distances_relative[j]} results in point {reference_path[-1]}"
                )
            i += 1
        # print(f"ref path 1: {reference_path}")

        # mid_time = perf_counter()

        # reference_path = []
        # for i in range(len(distances)):
        #     # Find first value in distances_relative that is greater than distance
        #     for j in range(len(distances_relative)):
        #         if distances_relative[j] < distances[i]:
        #             continue
        #         elif distances_relative[j] == distances[i]:
        #             # Just take point on path
        #             scaling = 1
        #         elif distances_relative[j] > distances[i]:
        #             # Required distances between two points so scale between them
        #             scaling = 1 - np.abs(
        #                 (distances[i] - distances_relative[j])
        #                 / (distances_relative[j] - distances_relative[j - 1])
        #             )

        #         reference_path.append(
        #             shifted_path[j - 1]
        #             + scaling * (shifted_path[j] - shifted_path[j - 1])
        #         )
        #         if debug:
        #             print(
        #                 f"Point {j - 1} and {j} with scaling {scaling} and distance {distances[i]} and relative distance {distances_relative[j]} results in point {reference_path[-1]}"
        #             )
        #         break
        # print(f"ref path 2: {reference_path}")

        # end_time = perf_counter()

        # print(f"Time 1: {mid_time - start_time}")
        # print(f"Time 2: {end_time - mid_time}")

        return reference_path

    def get_boundary_constraints(self, ref_track, width, plot=False):
        path = np.array(ref_track)

        tangent_points = []
        for i in range(len(path)):
            if i == 0:
                tangent_points.append([0, 0])
            else:
                diff = path[i] - path[i - 1]
                tangent = np.array([-diff[1], diff[0]]) / np.linalg.norm(diff)
                tangent_points.append(tangent)
        tangent_points[0] = tangent_points[-1]

        tangent_points = np.array(tangent_points)
        pos_inner = path + width * tangent_points
        pos_outer = path - width * tangent_points

        self.slopes_inner = -tangent_points[:, 0] / tangent_points[:, 1]
        self.intercepts_inner = pos_inner[:, 1] - self.slopes_inner * pos_inner[:, 0]

        self.slopes_outer = -tangent_points[:, 0] / tangent_points[:, 1]
        self.intercepts_outer = pos_outer[:, 1] - self.slopes_outer * pos_outer[:, 0]

        # print(self.slopes_inner)

        if plot:
            plt.plot(path[:, 0], path[:, 1], "o", c="r")
            plt.plot(pos_inner[:, 0], pos_inner[:, 1], "o", c="b")
            plt.plot(pos_outer[:, 0], pos_outer[:, 1], "o", c="y")

            for i in range(len(path)):
                # generate x around point
                x = np.linspace(path[i, 0] - 2, path[i, 0] + 2, 100)
                y_inner = self.slopes_inner[i] * x + self.intercepts_inner[i]
                y_outer = self.slopes_outer[i] * x + self.intercepts_outer[i]
                plt.plot(x, y_inner, c="b")
                plt.plot(x, y_outer, c="y")

    def run_mpc(self):
        initial_state = [0, 0, 0, 0, self.target_speed]

        ref_track = self.get_reference_path(debug=False)
        # print(ref_track)

        self.get_boundary_constraints(ref_track, 1.2, plot=False)

        X0 = []
        U0 = []

        for x0 in ref_track:
            # heading = self.curve_centerline(tau)[1]
            steering_angle = 0

            # calculate heading based on position
            if len(X0) > 0:
                # calculate tangent based on previous and next point
                # tangent_next = ref_track[min(len(ref_track) - 1, ref_track.index(x0) + 1)] - x0
                # tangent_previous = x0 - X0[-1][:2]
                # tangent = tangent_next + tangent_previous
                # phi = np.arctan2(tangent[1], tangent[0])
                phi = np.arctan2(x0[1] - X0[-1][1], x0[0] - X0[-1][0])
            else:
                phi = 0

            # map to 0 to 2pi
            # phi = (phi + 2 * np.pi) % (2 * np.pi)
            heading = phi
            if len(X0) > 0:
                diff_with_prev = heading - X0[-1][2]
                # scale to [-pi, pi]
                diff_with_prev = (diff_with_prev + np.pi) % (2 * np.pi) - np.pi
                heading = X0[-1][2] + diff_with_prev

            # calculate steering angle based on position
            if len(X0) > 0:
                # Apply inverse bicycle model
                # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
                alpha = heading - X0[-1][2]

                # Not sure why this / 2 but gives better results
                alpha /= 2

                l_d = np.sqrt((x0[0] - X0[-1][0]) ** 2 + (x0[1] - X0[-1][1]) ** 2)

                # Calculate turning radius
                R = (l_d) / (2 * np.sin(alpha)) if np.sin(alpha) != 0 else 0

                steering_angle = np.arctan(self.car.L / R) if R != 0 else 0

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

            x0_state = [x0[0], x0[1], heading, steering_angle, velocity]
            X0.append(x0_state)

        # Shift steering angle and velocity one step back
        for i in range(len(X0) - 1):
            X0[i][3] = X0[i + 1][3]
            X0[i][4] = X0[i + 1][4]

        # Set first steering angle equal to second
        X0[0][3] = X0[1][3]

        # Set first heading equal to second
        X0[0][2] = X0[1][2]

        # Plot x-y and heading
        # plt.plot([x[0] for x in X0], [x[1] for x in X0], c="b")

        # Plot heading at x-y points
        # for i in range(len(X0)):
        #     plt.plot(
        #         [X0[i][0], X0[i][0] + np.cos(X0[i][2])],
        #         [X0[i][1], X0[i][1] + np.sin(X0[i][2])],
        #         c="r",
        #     )

        # # Plot steering angle at x-y points
        # for i in range(len(X0)):
        #     plt.plot(
        #         [X0[i][0], X0[i][0] + np.cos(X0[i][3] + X0[i][2])],
        #         [X0[i][1], X0[i][1] + np.sin(X0[i][3] + X0[i][2])],
        #         c="g",
        #     )

        for i in range(self.N):
            acceleration = (X0[i + 1][4] - X0[i][4]) / self.car.dt / self.wheelradius

            steering_angle_delta = X0[i + 1][3] - X0[i][3]
            steering_angle_delta /= self.car.dt
            U0.append([acceleration, steering_angle_delta])

        self.u = U0[0]
        initial_state = X0[0]
        X0 = np.array(X0).T
        U0 = np.array(U0).T

        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy", X0)
        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy", U0)

        print(X0)
        print(U0)

        reference_track = []
        for ref_point in ref_track:
            reference_track.append(
                [
                    ref_point[0],
                    ref_point[1],
                    0,
                    self.steering_joint_angle,
                    self.target_speed,
                ]
            )

        initial_state = [ref_track[0][0], ref_track[0][1], 0, 0, 0]
        initial_state = X0[:, 0]

        u, info = self.mpc(
            initial_state,
            reference_track,
            self.slopes_inner,
            self.intercepts_inner,
            self.slopes_outer,
            self.intercepts_outer,
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

        # plot_velocity(
        #     info["X_sol"].T[:, :2],
        #     self.GT_left_boundary[:-1],
        #     self.GT_right_boundary[:-1],
        #     info["X_sol"].T[:, 4],
        #     info["U_sol"].T[:, 0],
        #     show_plots=True,
        # )

        x_sol = info["X_sol"][0][:]
        y_sol = info["X_sol"][1][:]

        x_sol = np.append(x_sol, x_sol[0])
        y_sol = np.append(y_sol, y_sol[0])

        # x_traj = self.interpolate_arr(np.vstack((x_sol, y_sol)).T, n=3)
        x_traj = np.vstack((x_sol, y_sol)).T

        x_traj = np.vstack((x_traj, x_traj[0]))
        # plt.plot(x_traj.T[0], x_traj.T[1], c="m")
        # plt.plot(info["X_sol"][0][:], info["X_sol"][1][:], c="m", label="Solution reference-tracking with halfspaces")
        # plt.plot(info["X_sol"][0][:], info["X_sol"][1][:], c="m", label="Reference-tracking with circles")
        # plt.plot(info["X_sol"][0][:], info["X_sol"][1][:], "o", c="m")

        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        ax.axis("off")
        ax.legend(bbox_to_anchor=(1.05, 0.9), loc="upper right", fontsize=16)
        # plt.legend()
        plt.tight_layout()
        plt.show()

    def save_solution(self):
        # Get convergence information
        inf_pr = self.ocp.debug.stats()["iterations"]["inf_pr"]
        inf_du = self.ocp.debug.stats()["iterations"]["inf_du"]
        inf_obj = self.ocp.debug.stats()["iterations"]["obj"]
        # print(self.ocp.debug.show_infeasibilities())

        np.savez(
            "/home/ugr/autonomous2023/ROS/src/control/MPC/data/solution.npz",
            U_sol_intermediate=self.mpc.U_sol_intermediate,
            X_sol_intermediate=self.mpc.X_sol_intermediate,
            info_pr=inf_pr,
            info_du=inf_du,
            info_obj=inf_obj,
        )


node = MPC_gen_ref()
