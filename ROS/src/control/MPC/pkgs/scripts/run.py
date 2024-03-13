#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf
from environments.bicycle_model import BicycleModel
from geometry_msgs.msg import PointStamped, PoseStamped
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
from node_fixture.managed_node import ManagedNode
from optimal_control.MPC_tracking import MPC_tracking
from optimal_control.ocp import Ocp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory import Trajectory
from ugr_msgs.msg import State


class MPC(ManagedNode):
    def __init__(self):
        rospy.init_node("MPC_tracking_control")
        super().__init__("MPC_tracking_control")
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.slam_state = SLAMStatesEnum.IDLE
        rospy.Subscriber("/state", State, self.handle_state_change)
        self.start_sender()
        rospy.spin()

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0

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

        self.car = BicycleModel(dt=0.05)  # dt = publish rate?

        self.steering_joint_angle = 0
        self.u = [0, 0]

        self.N = 40
        self.ocp = Ocp(
            self.car.nx,
            self.car.nu,
            N=self.N,
            F=self.car.F,
            T=self.car.dt * self.N,
            show_execution_time=False,
            silent=True,
            store_intermediate=False,
        )
        self.mpc = MPC_tracking(self.ocp)

        # Initilize previous input with zeros
        self.prev_input = [[0.0, 0.0]] * self.N

        Qn = np.diag([8e-3, 8e-3, 0, 0, 0])
        R = np.diag([1e-5, 5e-2])
        R_delta = np.diag([1e-1, 5e-3])

        self.set_costs(Qn, R, R_delta)

        # constraints
        # TODO: get these from urdf model
        self.max_steering_angle = 5  # same as pegasus.urdf
        self.set_constraints(self.speed_target * 10, self.max_steering_angle)

    def doActivate(self):
        # Do this here because some parameters are set in the mission yaml files
        self.trajectory = Trajectory()

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

    def set_constraints(self, velocity_limit, steering_limit):
        """
        Set constraints for the MPC
        """
        velocity_limit = 5
        steering_limit = 5
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
        self.ocp._set_continuity(1)

    def set_costs(self, Qn, R, R_delta):
        """
        Set costs for the MPC
        """
        self.ocp.running_cost = (
            (self.ocp.x - self.ocp.x_reference).T
            @ Qn
            @ (self.ocp.x - self.ocp.x_reference)
            + self.ocp.u.T @ R @ self.ocp.u
            + self.ocp.u_delta.T @ R_delta @ self.ocp.u_delta
        )

    def start_sender(self):
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.state == NodeManagingStatesEnum.ACTIVE:
                try:
                    self.doUpdate()
                    ref_track = self.trajectory.get_reference_track(
                        self.car.dt, self.N, self.actual_speed
                    )

                    if len(ref_track) == 0:
                        self.diagnostics_pub.publish(
                            create_diagnostic_message(
                                level=DiagnosticStatus.WARN,
                                name="[CTRL MPC] Reference Track Status",
                                message="No reference track found.",
                            )
                        )
                        self.velocity_cmd.data = 0.0
                        self.steering_cmd.data = 0.0
                        self.velocity_pub.publish(self.velocity_cmd)
                        self.steering_pub.publish(self.steering_cmd)
                        rate.sleep()
                        continue

                    # Get left and right boundary halfspaces
                    # NOTE: works terrible without BE
                    a, b, c, d = self.trajectory.get_tangent_line(ref_track)

                    left_point = ref_track[len(ref_track) // 2]
                    x_points_left = np.linspace(left_point[0] - 5, left_point[0] + 5)
                    y_points_left = a * x_points_left + b
                    self.vis_path(
                        list(zip(x_points_left, y_points_left)), self.left_line_pub
                    )

                    right_point = ref_track[len(ref_track) // 2]
                    x_points_right = np.linspace(right_point[0] - 5, right_point[0] + 5)
                    y_points_right = c * x_points_right + d
                    self.vis_path(
                        list(zip(x_points_right, y_points_right)), self.right_line_pub
                    )

                    speed_target = rospy.get_param("/speed/target", 3.0)

                    # Change velocity constraints when speed target changes
                    # Need other costs for  different speeds as this changes the working environment
                    if speed_target != self.speed_target:
                        self.speed_target = speed_target
                        self.set_constraints(speed_target / 2, self.max_steering_angle)

                    if self.slam_state == SLAMStatesEnum.RACING:
                        # Scale steering penalty based on current speed
                        Qn = np.diag([8, 8, 0, 0, 0])
                        R = np.diag([5e-2, 100])
                        R_delta = np.diag(
                            [10, 0]  # * self.actual_speed / self.speed_target]
                        )

                        self.set_costs(Qn, R, R_delta)

                    reference_track = []
                    for ref_point in ref_track:
                        reference_track.append(
                            [
                                ref_point[0],
                                ref_point[1],
                                0,
                                self.steering_joint_angle,
                                self.speed_target,
                            ]
                        )

                    # self.mpc.reset()
                    # TODO: get steering joint angle from ros control
                    current_state = [
                        0,
                        0,
                        0,
                        self.steering_joint_angle,
                        self.actual_speed,
                    ]

                    # Run MPC
                    # Give reference_track as initial trajectory guess
                    # TODO: also give orientation
                    # Transpose because of shape needed in MPC formulation (nx, N+1)
                    # x0 = np.concatenate(
                    #     (np.array([current_state]).T, np.array(reference_track).T),
                    #     axis=1,
                    # )
                    u, info = self.mpc(
                        current_state, reference_track, a, b, c, d, self.u  # , X0=x0
                    )  # , U0=self.prev_input)
                    self.u = u

                    # X_closed_loop = np.array(self.mpc.X_trajectory)
                    # U_closed_loop = np.array(self.mpc.U_trajectory)
                    self.prev_input = info["U_sol"]

                    # rospy.loginfo(f"X_closed_loop: {info['X_sol']}")
                    # rospy.loginfo(f"x closed loop: {X_closed_loop}")
                    # rospy.loginfo(f"U_closed_loop: {info['U_sol']}")
                    # rospy.loginfo(f"u closed loop: {U_closed_loop}")
                    rospy.loginfo(f"u return: {u}")
                    rospy.loginfo(f"actual speed: {self.actual_speed}")

                    # Publish MPC prediction for visualization
                    self.vis_path(
                        list(zip(info["X_sol"][:][0], info["X_sol"][:][1])),
                        self.x_vis_pub,
                    )

                    # self.steering_cmd.data = U_closed_loop[0, 1]
                    # self.steering_joint_angle = info["X_sol"][3][1]
                    # # Send steering position to ros_control
                    # self.steering_cmd.data = self.steering_joint_angle
                    # self.steering_joint_angle += u[1] * self.car.dt
                    # self.steering_cmd.data = self.steering_joint_angle + u[1] * self.car.dt
                    # # self.steering_cmd.data = -1
                    # self.velocity_cmd.data = (
                    #     self.actual_speed + u[0] * self.wheelradius * self.car.dt
                    # )  # Input is acceleration
                    self.velocity_cmd.data = u[0]
                    self.steering_cmd.data = u[1]

                    # Use Pure Pursuit if target speed is 0
                    # TODO: avoid this + probably wrong
                    if self.speed_target == 0.0:
                        target_x = ref_track[-1][0]
                        target_y = ref_track[-1][1]
                        R = ((target_x - 0) ** 2 + (target_y - 0) ** 2) / (
                            2 * (target_y - 0)
                        )

                        self.steering_cmd.data = self.symmetrically_bound_angle(
                            np.arctan2(1.0, R), np.pi / 2
                        )

                        self.velocity_cmd.data = 0.0

                    # Publish to velocity and position steering controller
                    # self.steering_cmd.data /= self.steering_transmission
                    self.steering_pub.publish(self.steering_cmd)

                    # self.velocity_cmd.data /= (
                    #     self.wheelradius
                    # )  # Velocity to angular velocity
                    # print(f"velocity: {self.velocity_cmd.data}")
                    self.velocity_pub.publish(self.velocity_cmd)

                    self.vis_path(reference_track, self.ref_track_pub)

                except Exception as e:
                    rospy.logwarn(f"MPC has caught an exception: {e}")
                    import traceback

                    print(traceback.format_exc())

            rate.sleep()

    def vis_path(self, path, publisher, stamp=None, frame_id=None):
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


node = MPC()
