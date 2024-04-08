#!/usr/bin/env python3

import numpy as np
import rospkg
import rospy
import tf2_ros as tf
import yaml
from environments.bicycle_model import BicycleModel
from geometry_msgs.msg import PointStamped, PoseStamped
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    NodeManagingStatesEnum,
    ROSNode,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from optimal_control_gen.MPC_tracking import MPC_tracking
from optimal_control_gen.ocp import Ocp
from scipy.interpolate import interp1d
from spline_utils import create_spline, project_on_spline
from std_msgs.msg import Float64
from trajectory import Trajectory
from ugr_msgs.msg import State


class MPC_gen:
    def __init__(self):
        rospy.init_node("MPC_generation_control")

        self.path_GT_centerline = rospy.get_param(
            "~input_path", "/data_gen/centerline_fsc23.yaml"
        )
        self.path_GT_map = rospy.get_param("~input_path", "/data_gen/map_fsc23.yaml")

        # Get centerline from yaml
        self.GT_centerline = []

        arr = self.read_yaml(self.path_GT_centerline)
        for pose in arr["poses"]:
            self.GT_centerline.append(
                [pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]]
            )

        # Close centerline
        self.GT_centerline.append(self.GT_centerline[0])

        self.GT_centerline = self.interpolate_arr(self.GT_centerline)

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

        spline_centerline, curve_centerline = create_spline(
            self.GT_centerline, "r", derivative=False, plot=True
        )

        car_pos = np.array([1, -2])
        taus = np.linspace(0, 1, 10000)
        dcurve = curve_centerline.derivative(o=1)
        points_on_spline = curve_centerline(taus)
        der_points = dcurve(taus)
        project_on_spline(points_on_spline, der_points, car_pos, plot=True)

        create_spline(self.GT_left_boundary, "b", plot=True)
        create_spline(self.GT_right_boundary, "y", plot=True)

        # plt.xlim(-40, 20)
        # plt.ylim(-10, 20)

        plt.xlim(-20, 50)
        plt.ylim(-60, 10)
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.show()

        rospy.spin()

    def read_yaml(self, path):
        path = rospkg.RosPack().get_path("mpc_gen") + path

        with open(path, "r") as file:
            arr = yaml.safe_load(file)
        return arr

    def interpolate_arr(self, arr):
        distance = np.cumsum(np.sqrt(np.sum(np.diff(arr, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        alpha = np.linspace(0, 1, len(arr) * 3)
        interpolator = interp1d(distance, arr, kind="cubic", axis=0)
        arr = interpolator(alpha)

        return arr

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0
        self.speed_start = rospy.get_param("~speed_start", 10)
        self.speed_stop = rospy.get_param("~speed_stop", 50)
        self.distance_start = rospy.get_param("~distance_start", 1.2)
        self.distance_stop = rospy.get_param("~distance_stop", 2.4)

        # Publishers for the controllers
        # Controllers themselves spawned in the state machines respective launch files

        self.velocity_pub = super().AddPublisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_pub = super().AddPublisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
        )
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

        self.speed_target = rospy.get_param("/speed/target", 3.0)
        self.steering_transmission = rospy.get_param(
            "ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

        self.car = BicycleModel(dt=0.05)  # dt = publish rate?

        self.N = 10
        self.np = 2  # Number of control points to keep car close to path
        self.ocp = Ocp(
            self.car.nx,
            self.car.nu,
            np=self.np,
            N=self.N,
            F=self.car.F,
            T=self.car.dt * self.N,
            terminal_constraint=False,
            show_execution_time=False,
            silent=True,
        )

        Q = np.diag([1e-2, 1e-2, 0, 0, 1e-2])
        Qn = np.diag([8e-3, 8e-3, 0, 0, 1e-2])
        R = np.diag([1e-4, 6e-3])

        # Weight matrices for the terminal cost
        P = np.diag([0, 0, 0, 0, 0])

        self.ocp.running_cost = (
            (self.ocp.x - self.ocp.x_ref).T @ Q @ (self.ocp.x - self.ocp.x_ref)
            + (self.ocp.x - self.ocp.x_control).T
            @ Qn
            @ (self.ocp.x - self.ocp.x_control)
            + self.ocp.u.T @ R @ self.ocp.u
        )
        self.ocp.terminal_cost = (
            (self.ocp.x - self.ocp.x_ref).T @ P @ (self.ocp.x - self.ocp.x_ref)
        )

        # constraints
        max_steering_angle = 1
        self.ocp.subject_to(
            self.ocp.bounded(
                -self.speed_target / self.wheelradius,
                self.ocp.U[0, :],
                self.speed_target / self.wheelradius,
            )
        )
        self.ocp.subject_to(
            self.ocp.bounded(-max_steering_angle, self.ocp.U[1, :], max_steering_angle)
        )
        self.mpc = MPC_tracking(self.ocp)

    def doActivate(self):
        # Do this here because some parameters are set in the mission yaml files
        self.trajectory = Trajectory()

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

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
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.state == NodeManagingStatesEnum.ACTIVE:
                try:
                    self.doUpdate()

                    speed_target = rospy.get_param("/speed/target", 3.0)

                    # Change velocity constraints when speed target changes
                    if speed_target != self.speed_target:
                        self.speed_target = speed_target

                        # Reset constraints
                        self.ocp.opti.subject_to()
                        self.ocp.subject_to(
                            self.ocp.bounded(
                                -self.speed_target / self.wheelradius,
                                self.ocp.U[0, :],
                                self.speed_target / self.wheelradius,
                            )
                        )
                        self.ocp.opti.subject_to(
                            self.ocp.opti.bounded(-1, self.ocp.U[1, :], 1)
                        )
                        self.ocp._set_continuity(1)

                    if self.slam_state == SLAMStatesEnum.RACING:
                        # Scale steering penalty based on current speed
                        Q = np.diag([1e-2, 1e-2, 0, 0, 1e-2])
                        Qn = np.diag([8e-3, 8e-3, 0, 0, 1e-2])
                        R = np.diag(
                            [1e-5, 3e-1 * self.actual_speed / self.speed_target]
                        )

                        # Weight matrices for the terminal cost
                        P = np.diag([0, 0, 0, 0, 0])

                        self.ocp.running_cost = (
                            (self.ocp.x - self.ocp.x_ref).T
                            @ Q
                            @ (self.ocp.x - self.ocp.x_ref)
                            + (self.ocp.x - self.ocp.x_control).T
                            @ Qn
                            @ (self.ocp.x - self.ocp.x_control)
                            + self.ocp.u.T @ R @ self.ocp.u
                        )
                        self.ocp.terminal_cost = (
                            (self.ocp.x - self.ocp.x_ref).T
                            @ P
                            @ (self.ocp.x - self.ocp.x_ref)
                        )

                    # Change the look-ahead distance (minimal_distance) based on the current speed
                    if self.actual_speed < self.speed_start:
                        self.minimal_distance = self.distance_start
                    elif self.actual_speed < self.speed_stop:
                        self.minimal_distance = self.distance_start + (
                            self.distance_stop - self.distance_start
                        ) / (self.speed_stop - self.speed_start) * (
                            self.actual_speed - self.speed_start
                        )
                    else:
                        self.minimal_distance = self.distance_stop

                    # Calculate target points
                    distances = [
                        self.minimal_distance * n
                        for n in np.linspace(0, 1, self.np + 2)[1:]
                    ]
                    target_points = self.trajectory.calculate_target_points(distances)

                    target_x, target_y = target_points[-1]
                    control_targets = []
                    for control_target in target_points[:-1]:
                        control_targets.append(
                            [
                                control_target[0],
                                control_target[1],
                                0,
                                0,
                                self.speed_target,
                            ]
                        )

                    if target_x == 0 and target_y == 0:
                        self.diagnostics_pub.publish(
                            create_diagnostic_message(
                                level=DiagnosticStatus.WARN,
                                name="[CTRL MPC] Target Point Status",
                                message="Target point not found.",
                            )
                        )
                        self.velocity_cmd.data = 0.0
                        self.steering_cmd.data = 0.0
                        self.velocity_pub.publish(self.velocity_cmd)
                        self.steering_pub.publish(self.steering_cmd)
                        rate.sleep()
                        continue

                    # rospy.loginfo(f"target_x: {target_x}, target_y: {target_y}")
                    # rospy.loginfo(f"control_targets: {control_targets}")

                    self.mpc.reset()
                    init_state = [0, 0, 0, 0, self.actual_speed]
                    goal_state = [target_x, target_y, 0, 0, self.speed_target]

                    current_state = init_state

                    # Run MPC
                    u, info = self.mpc(current_state, goal_state, control_targets)

                    # X_closed_loop = np.array(self.mpc.X_trajectory)
                    U_closed_loop = np.array(self.mpc.U_trajectory)

                    # rospy.loginfo(f"X_closed_loop: {info['X_sol']}")
                    # rospy.loginfo(f"x closed loop: {X_closed_loop}")
                    # rospy.loginfo(f"U_closed_loop: {info['U_sol']}")
                    # rospy.loginfo(f"u closed loop: {U_closed_loop}")
                    # rospy.loginfo(f"actual speed: {self.actual_speed}")

                    # Publish MPC prediction for visualization
                    x_pred = Path()
                    x_pred.header.stamp = self.trajectory.time_source
                    x_pred.header.frame_id = self.base_link_frame
                    for x, y in zip(info["X_sol"][:][0], info["X_sol"][:][1]):
                        pose = PoseStamped()
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        x_pred.poses.append(pose)

                    self.x_vis_pub.publish(x_pred)

                    self.steering_cmd.data = U_closed_loop[0, 1]
                    self.velocity_cmd.data = (
                        self.actual_speed
                        + U_closed_loop[0, 0] * self.wheelradius * self.car.dt
                    )  # Input is acceleration

                    # Use Pure Pursuit if target speed is 0
                    if self.speed_target == 0.0:
                        R = ((target_x - 0) ** 2 + (target_y - 0) ** 2) / (
                            2 * (target_y - 0)
                        )

                        self.steering_cmd.data = self.symmetrically_bound_angle(
                            np.arctan2(1.0, R), np.pi / 2
                        )

                        self.velocity_cmd.data = 0.0

                    self.diagnostics_pub.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[CTRL MPC] Target Point Status",
                            message="Target point found.",
                        )
                    )

                    # Publish to velocity and position steering controller
                    self.steering_cmd.data /= self.steering_transmission
                    self.steering_pub.publish(self.steering_cmd)

                    self.velocity_cmd.data /= (
                        self.wheelradius
                    )  # Velocity to angular velocity
                    self.velocity_pub.publish(self.velocity_cmd)

                    # Publish target point for visualization
                    point = PointStamped()
                    point.header.stamp = self.trajectory.time_source
                    point.header.frame_id = self.base_link_frame
                    point.point.x = target_x
                    point.point.y = target_y
                    self.vis_pub.publish(point)

                except Exception as e:
                    rospy.logwarn(f"MPC has caught an exception: {e}")
                    import traceback

                    print(traceback.format_exc())

            rate.sleep()


node = MPC_gen()
