#!/usr/bin/env python3

# from time import perf_counter

import casadi as cd
import numpy as np
import rospy
import tf2_ros as tf
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from environments_gen.bicycle_model_spline import BicycleModelSpline
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import (
    DiagnosticArray,
    NodeManagingStatesEnum,
    ROSNode,
    SLAMStatesEnum,
    StateMachineScopeEnum,
)
from node_fixture.managed_node import ManagedNode
from optimal_control_gen.MPC_generation import MPC_generation
from optimal_control_gen.ocp import Ocp
from sensor_msgs.msg import JointState
from spline_utils import create_spline  # , get_boundary_constraints_casadi
from std_msgs.msg import Float64
from trajectory_gen import Trajectory
from ugr_msgs.msg import State


class MPCSplinesTracking(ManagedNode):
    def __init__(self):
        print("in init")
        rospy.init_node("MPC_splines_tracking_control")
        super().__init__("MPC_splines_tracking_control")

        self.publish_rate = rospy.get_param("~publish_rate", 40)

        self.slam_state = SLAMStatesEnum.IDLE

        rospy.Subscriber("/state", State, self.handle_state_change)
        self.start_sender()
        rospy.spin()

        # spline_centerline, curve_centerline = create_spline(
        #     self.GT_centerline, "r", derivative=False, derivative2=False, plot=True
        # )

        # self.curve_centerline = curve_centerline
        # self.der_centerline = curve_centerline.derivative(o=1)

        # car_pos = np.squeeze(curve_centerline(0))
        # self.start_pos = car_pos
        # taus = np.linspace(0, 1, 10000)
        # points_on_spline = curve_centerline(taus)
        # der_points = self.der_centerline(taus)

        # create_spline(self.GT_left_boundary, "b", plot=True)
        # create_spline(self.GT_right_boundary, "y", plot=True)

        # ax = plt.gca()
        # ax.set_aspect("equal", "datalim")

        # self.initialize_MPC()

        # # self.analyse_cost()

        # self.run_mpc()

        # # self.save_solution()

        # rospy.spin()

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0

        self.exec_times = []

        # Publishers for the controllers
        self.drive_effort_pub = super().AddPublisher(
            "/output/drive_effort_controller/command", Float64, queue_size=10
        )
        self.drive_velocity_pub = super().AddPublisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_velocity_pub = super().AddPublisher(
            "/output/steering_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_position_pub = super().AddPublisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
        )

        self.switched_controllers = False

        # For visualization
        self.vis_pub = super().AddPublisher(
            "/output/target_point",
            PointStamped,
            queue_size=10,  # warning otherwise
        )

        self.x_vis_pub = super().AddPublisher(
            "/output/x_prediction",
            Path,
            queue_size=10,  # warning otherwise
        )

        self.ref_track_pub = super().AddPublisher(
            "/output/ref_track",
            Path,
            queue_size=10,  # warning otherwise
        )

        self.left_line_pub = super().AddPublisher(
            "/output/left_line",
            Path,
            queue_size=10,  # warning otherwise
        )

        self.right_line_pub = super().AddPublisher(
            "/output/right_line",
            Path,
            queue_size=10,  # warning otherwise
        )

        # Diagnostics Publisher
        self.diagnostics_pub = super().AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Subscriber for path
        self.path_sub = super().AddSubscriber(
            "/input/path", Path, self.getPathplanningUpdate
        )

        self.received_path = None

        self.odom_sub = super().AddSubscriber(
            "/input/odom", Odometry, self.get_odom_update
        )

        self.joint_state_sub = super().AddSubscriber(
            "/ugr/car/joint_states", JointState, self.get_joint_states
        )

        self.speed_target = rospy.get_param("/speed/target", 3.0)

        self.steering_transmission = rospy.get_param(
            "ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

        self.initialize_MPC()

    def doActivate(self):
        # Launch ros_control controllers
        rospy.wait_for_service("/ugr/car/controller_manager/switch_controller")
        try:
            switch_controller = rospy.ServiceProxy(
                "/ugr/car/controller_manager/switch_controller", SwitchController
            )

            req = SwitchControllerRequest()
            req.start_controllers = [
                "joint_state_controller",
                "steering_velocity_controller",
                "drive_effort_controller",
            ]
            req.stop_controllers = []
            req.strictness = SwitchControllerRequest.STRICT

            response = switch_controller(req)

            if response.ok:
                # Do this here because some parameters are set in the mission yaml files
                self.trajectory = Trajectory()

            else:
                rospy.logerr("Could not start controllers")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def initialize_MPC(self):
        self.car = BicycleModelSpline(dt=0.1)

        self.steering_joint_angle = 0
        self.u = [0, 0, 0]

        # create random path for initialisatin
        arr = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
        centerline, curve_centerline = create_spline(arr, color="r", plot=False)

        self.N = 10
        self.ocp = Ocp(
            self.car.nx,
            self.car.nu,
            curve=curve_centerline,
            N=self.N,
            F=self.car.F,
            T=self.car.dt * self.N,
            show_execution_time=False,
            silent=True,
            store_intermediate=True,
            threads=1,
            adaptive_boundaries=False,
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

    def set_constraints(self, velocity_limit, steering_limit):
        """
        Set constraints for the MPC
        """
        velocity_limit = 10
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
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.X[4, :], 10))

        # Progress parameter between 0 and 1
        self.ocp.subject_to(self.ocp.bounded(0.00000001, self.ocp.X[5, :], 1))

        # Limit update to progress parameter
        # Starts from 0.01 because otherwise casadi optimizes to very small negative values
        # self.ocp.subject_to(
        #     self.ocp.bounded(
        #         1 / (self.N * 2), self.ocp.U[2, :], 1 / (0.8 * self.N)
        #     )
        # )

        self.ocp.subject_to(self.ocp.bounded(0.001, self.ocp.U[2, :], 0.12))
        # Limit relaxing constraint
        # self.ocp.subject_to(self.ocp.bounded(0, self.ocp.Sc, 0.1))

        # Make sure neither x and y are zero at the same time by limiting square to be larger than 0
        # der_points = self.ocp.der_centerline(self.ocp.X[5, :].T)
        # self.ocp.subject_to(der_points[:,0] ** 2 + der_points[:, 1] ** 2 > 1e-1)

        # # For loop takes quite long to initialize
        # if self.ocp.adaptive_boundaries:
        # center_points = self.ocp.centerline(self.ocp.X[5, :].T)
        # for i in range(self.N + 1):
        #     self.ocp.subject_to(
        #         (
        #             (
        #                 self.ocp.X[0, i]
        #                 # - self.ocp.centerline(self.ocp.X[5, i]).T[0]
        #                 - center_points[i, 0]
        #             )
        #             ** 2
        #             + (
        #                 self.ocp.X[1, i]
        #                 # - self.ocp.centerline(self.ocp.X[5, i]).T[1]
        #                 - center_points[i, 1]
        #             )
        #             ** 2
        #         )
        #         < (1.2**2)  # + self.ocp.Sc[i] ** 2
        #     )
        # else:
        for i in range(self.N + 1):
            self.ocp.subject_to(
                (
                    (
                        self.ocp.X[0, i]
                        # - self.ocp.centerline(self.ocp.X[5, i]).T[0]
                        - self.ocp.center_points[0, i]
                    )
                    ** 2
                    + (
                        self.ocp.X[1, i]
                        # - self.ocp.centerline(self.ocp.X[5, i]).T[1]
                        - self.ocp.center_points[1, i]
                    )
                    ** 2
                )
                < (1.2**2)  # + self.ocp.Sc[i] ** 2
            )

        # (
        #     slopes_inner,
        #     intercepts_inner,
        #     slopes_outer,
        #     intercepts_outer,
        # ) = get_boundary_constraints_casadi(
        #     self.ocp.centerline, self.ocp.der_centerline, self.ocp.X[5, :].T, 1.5
        # )

        # self.ocp.subject_to(
        #     (slopes_inner * self.ocp.X[0, :].T + intercepts_inner - self.ocp.X[1, :].T)
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

        # Make sure last state heading is close to first state heading
        # self.ocp.subject_to(cd.fabs(cd.fmod(self.ocp.X[2, self.N], 2*cd.pi) - cd.fmod(self.ocp.X[2, 0], 2*cd.pi)) < 0.1)
        # self.ocp.subject_to(cd.fabs(cd.fmod(self.ocp.X[3, self.N], 2*cd.pi) - cd.fmod(self.ocp.X[3, 0], 2*cd.pi)) < 0.1)
        # self.ocp.subject_to(cd.fabs(self.ocp.X[4, self.N] - self.ocp.X[3, 0]) < 5)

        self.ocp._set_continuity(1)

        print("Constraints set")

    def set_costs(self, Qn, R, R_delta):
        """
        Set costs for the MPC
        """
        # qs = 1e-2
        # qss = 1e-2
        qc = 1e-7
        ql = 1e1

        # State: x, y, heading, steering angle, velocity
        # Qn = np.diag([8e-3, 8e-3, 0, 0, 0])

        # Input: acceleration, velocity on steering angle, update to progress parameter
        # Maximize progress parameter
        R = np.diag([1e-5, 1e-1, 1e-2])
        R_delta = np.diag([1e-1, 1e-1, 1e-2])

        q_obj = 2e2

        # Avoid division by zero
        # phi = cd.if_else(
        #     cd.fabs(self.ocp.der_curve[1]) < 1e-3,
        #     0,
        #     cd.atan2(self.ocp.der_curve[1], self.ocp.der_curve[0]),
        # )
        # Add small noise to avoid division by zero in derivative
        phi = cd.atan2((self.ocp.der_curve[1] + 1e-10), (self.ocp.der_curve[0] + 1e-10))
        e_c = cd.sin(phi) * (self.ocp.x[0] - self.ocp.point_curve[0]) - cd.cos(phi) * (
            self.ocp.x[1] - self.ocp.point_curve[1]
        )
        e_l = -cd.cos(phi) * (self.ocp.x[0] - self.ocp.point_curve[0]) - cd.sin(phi) * (
            self.ocp.x[1] - self.ocp.point_curve[1]
        )
        obj = -self.ocp.u[2]

        self.ocp.running_cost = (
            qc * e_c**2
            + ql * e_l**2
            + self.ocp.u.T @ R @ self.ocp.u
            + self.ocp.u_delta.T @ R_delta @ self.ocp.u_delta
            # + qs @ self.ocp.sc
            # + qss @ self.ocp.sc**2
            + q_obj * obj
        )

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

    def get_joint_states(self, msg: JointState):
        self.steering_joint_angle = msg.position[msg.name.index("axis_steering")]

    def handle_state_change(self, msg: State):
        if msg.scope == StateMachineScopeEnum.SLAM:
            self.slam_state = msg.cur_state

    def getPathplanningUpdate(self, msg: Path):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        The path should be relative to self.base_link_frame. Otherwise it will transform to it
        """

        if msg is None:
            return

        self.received_path = msg

    def doUpdate(self):
        """
        Actually processes a new path
        The path should be relative to self.base_link_frame. Otherwise it will transform to it
        """

        if self.received_path is None:
            return

        msg = self.received_path

        # Transform received message to self.base_link_frame
        trans = self.tf_buffer.lookup_transform(
            self.base_link_frame,
            msg.header.frame_id,
            msg.header.stamp,
        )
        transformed_path = ROSNode.do_transform_path(msg, trans)

        # Create a new path
        current_path = np.zeros((0, 2))
        for pose in transformed_path.poses:
            current_path = np.vstack(
                (current_path, [pose.pose.position.x, pose.pose.position.y])
            )

        # Save current path and time of transformation to self.trajectory
        self.trajectory.points = current_path
        self.trajectory.time_source = msg.header.stamp
        self.trajectory.path = transformed_path

        self.received_path = None

    def symmetrically_bound_angle(self, angle, max_angle):
        """
        Helper function to bound {angle} to [-max_angle, max_angle]
        """
        return (angle + max_angle) % (2 * max_angle) - max_angle

    def start_sender(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.state == NodeManagingStatesEnum.ACTIVE:
                try:
                    # start_time = perf_counter()
                    self.doUpdate()

                    self.speed_target = rospy.get_param("/speed/target", 3.0)

                    # Stop the car
                    # Do this by switching the controllers and using pure pursuit
                    if self.speed_target == 0.0:
                        if not self.switched_controllers:
                            rospy.wait_for_service(
                                "/ugr/car/controller_manager/switch_controller"
                            )
                            try:
                                switch_controller = rospy.ServiceProxy(
                                    "/ugr/car/controller_manager/switch_controller",
                                    SwitchController,
                                )

                                # Switch to correct controllers for pure pursuit
                                req = SwitchControllerRequest()
                                req.start_controllers = [
                                    "steering_position_controller",
                                    "drive_velocity_controller",
                                ]
                                req.stop_controllers = [
                                    "steering_velocity_controller",
                                    "drive_effort_controller",
                                ]
                                req.strictness = SwitchControllerRequest.STRICT

                                response = switch_controller(req)

                                if response.ok:
                                    self.switched_controllers = True

                                else:
                                    rospy.logerr("Could not start controllers")
                            except rospy.ServiceException as e:
                                rospy.logerr(f"Service call failed: {e}")

                        if self.switched_controllers:
                            # Put target at 2m
                            target = self.trajectory.calculate_target_points([2])[0]
                            # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
                            R = ((target[0] - 0) ** 2 + (target[1] - 0) ** 2) / (
                                2 * (target[1] - 0)
                            )

                            self.steering_cmd.data = self.symmetrically_bound_angle(
                                np.arctan2(1.0, R), np.pi / 2
                            )
                            self.steering_cmd.data /= self.steering_transmission
                            self.steering_position_pub.publish(self.steering_cmd)

                            self.velocity_cmd.data = 0.0
                            self.drive_velocity_pub.publish(self.velocity_cmd)
                            rate.sleep()
                            continue

                        else:
                            # Somehow could not switch controllers, so brake manually
                            self.velocity_cmd.data = -100.0
                            self.steering_cmd.data = 0.0
                            self.drive_effort_pub.publish(self.velocity_cmd)
                            self.steering_velocity_pub.publish(self.steering_cmd)
                            rate.sleep()
                            continue

                    (
                        spline_centerline,
                        curve_centerline,
                    ) = self.trajectory.create_spline()

                    # spline_time = perf_counter()

                    if curve_centerline is None:
                        self.velocity_cmd.data = 0.0
                        self.steering_cmd.data = 0.0
                        self.drive_effort_pub.publish(self.velocity_cmd)
                        self.steering_velocity_pub.publish(self.steering_cmd)
                        rate.sleep()
                        continue

                    # print(curve_centerline(0.1))

                    self.curve_centerline = curve_centerline

                    start_point = np.squeeze(curve_centerline(0))
                    print(f"start point: {start_point}")
                    # heading = np.arctan2(
                    #     np.squeeze(curve_centerline.derivative(o=1)(0.000000))[1],
                    #     np.squeeze(curve_centerline.derivative(o=1)(0.000000))[0],
                    # )
                    # current_state = [start_point[0], start_point[1], heading, self.steering_joint_angle, self.actual_speed, 0.0000001]
                    current_state = [
                        0,
                        0,
                        0,
                        self.steering_joint_angle,
                        self.actual_speed,
                        0.000001,
                    ]
                    if self.u[0] == 0 and self.u[1] == 0:
                        X0, U0 = self.initial_guess()

                        # current_state = [start_point[0], start_point[1], heading, self.steering_joint_angle, self.actual_speed, 0]

                        # print(X0)

                        # Run MPC
                        u, info = self.mpc(
                            current_state,
                            curve_centerline,
                            None,
                            None,
                            None,
                            None,
                            self.u,
                            X0=X0,
                            U0=U0,
                        )
                    else:
                        u, info = self.mpc(
                            current_state,
                            curve_centerline,
                            None,
                            None,
                            None,
                            None,
                            self.u,
                        )

                    self.u = u

                    self.exec_times.append(info["time"])
                    # print(u)

                    # rospy.loginfo(f"X_closed_loop: {info['X_sol']}")
                    # rospy.loginfo(f"x closed loop: {X_closed_loop}")
                    # rospy.loginfo(f"U_closed_loop: {info['U_sol']}")
                    # rospy.loginfo(f"u closed loop: {U_closed_loop}")
                    rospy.loginfo(f"u return: {u}")
                    rospy.loginfo(f"actual speed: {self.actual_speed}")

                    # Visualise MPC prediction
                    self.vis_path(
                        list(zip(info["X_sol"][:][0], info["X_sol"][:][1])),
                        self.x_vis_pub,
                    )

                    if info["success"]:
                        # print("MPC success")
                        self.velocity_cmd.data = u[0]
                        self.steering_cmd.data = u[1]

                        # Publish to velocity and position steering controller
                        self.steering_velocity_pub.publish(self.steering_cmd)
                        self.drive_effort_pub.publish(self.velocity_cmd)

                        # self.save_solution()
                    else:
                        self.save_solution()
                        # print(self.ocp.debug.show_infeasibilities())
                        # rospy.signal_shutdown("MPC failed")
                        # print(self.ocp.debug.show_infeasibilities())
                        self.velocity_cmd.data = u[0]
                        self.steering_cmd.data = u[1]

                        # Publish to velocity and position steering controller
                        self.steering_velocity_pub.publish(self.steering_cmd)
                        self.drive_effort_pub.publish(self.velocity_cmd)

                    # mpc_time = perf_counter()

                    self.save_solution()

                    # stop_time = perf_counter()

                    # print(f"spline time: {spline_time - start_time}")
                    # print(f"mpc time: {mpc_time - spline_time}")
                    # print(f"stop time: {stop_time - mpc_time}")

                except Exception as e:
                    rospy.logwarn(f"MPC has caught an exception: {e}")
                    import traceback

                    print(traceback.format_exc())

            rate.sleep()

    def initial_guess(self):
        Tau0 = np.linspace(0.00001, 0.5, self.N + 1)
        X0 = []
        U0 = []
        der_centerline = self.curve_centerline.derivative(o=1)
        for idx, tau in enumerate(Tau0):
            if idx == 0:
                x0_state = [0, 0, 0, 0, 0, 0]
                X0.append(x0_state)
                continue
            x0 = self.curve_centerline(tau)[0]
            # heading = self.curve_centerline(tau)[1]
            steering_angle = 0

            # der_curve = self.curve_centerline.derivative(o=1)
            der_points = der_centerline(tau)[0]

            phi = np.arctan2(der_points[1], der_points[0])
            print(phi)
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
                R = (l_d) / (2 * np.sin(alpha))

                steering_angle = np.arctan(self.car.L / R)

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

            x0_state = [x0[0], x0[1], heading, steering_angle, velocity, tau]
            X0.append(x0_state)

        # Set first velocity equal to second
        X0[0][4] = X0[1][4]

        for i in range(self.N):
            acceleration = (X0[i + 1][4] - X0[i][4]) / self.car.dt / self.wheelradius

            steering_angle_delta = X0[i + 1][3] - X0[i][3]
            steering_angle_delta /= self.car.dt

            dtau = X0[i + 1][5] - X0[i][5]
            U0.append([acceleration, steering_angle_delta, dtau])

        self.u = U0[0]
        # initial_state = X0[0]
        X0 = np.array(X0).T
        U0 = np.array(U0).T

        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy", X0)
        np.save("/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy", U0)

        return X0, U0

    def vis_path(self, path, publisher, stamp=None, frame_id=None):
        """
        Publishes a path to the specified topic
        """
        if publisher is None or len(path) == 0:
            return

        if stamp is None:
            stamp = self.trajectory.time_source

        if frame_id is None:
            frame_id = self.base_link_frame

        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = frame_id

        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        publisher.publish(path_msg)

    def save_solution(self):
        # Get convergence information
        inf_pr = self.ocp.debug.stats()["iterations"]["inf_pr"]
        inf_du = self.ocp.debug.stats()["iterations"]["inf_du"]
        inf_obj = self.ocp.debug.stats()["iterations"]["obj"]
        # print(self.ocp.debug.show_infeasibilities())

        np.savez(
            "/home/ugr/autonomous2023/ROS/src/control/MPC_gen/data_gen/solution.npz",
            U_sol_intermediate=self.mpc.U_sol_intermediate,
            X_sol_intermediate=self.mpc.X_sol_intermediate,
            info_pr=inf_pr,
            info_du=inf_du,
            info_obj=inf_obj,
            exec_times=self.exec_times,
            path_blf=self.trajectory.path_blf,
        )


node = MPCSplinesTracking()
