#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf
from environments.kinematic_car import KinematicCar
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    NodeManagingStatesEnum,
    create_diagnostic_message,
)
from node_fixture.node_management import ManagedNode
from optimal_control.MPC_tracking import MPC_tracking
from optimal_control.ocp import Ocp
from std_msgs.msg import Float64, Header
from tf2_geometry_msgs import do_transform_pose
from trajectory import Trajectory


class MPC(ManagedNode):
    def __init__(self):
        rospy.init_node("MPC_tracking_control")
        super().__init__("MPC_tracking_control")
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.start_sender()
        rospy.spin()

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.min_speed = rospy.get_param("~speed/min", 0.3)
        self.min_corner_speed = rospy.get_param("~speed/min_corner_speed", 0.7)
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
        self.vis_pub = super().AddPublisher(
            "/output/target_point",
            PointStamped,
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
        self.odom_sub = super().AddSubscriber(
            "/input/odom", Odometry, self.get_odom_update
        )

        self.speed_target = rospy.get_param("~speed/target", 3.0)
        self.steering_transmission = rospy.get_param(
            "ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

        car = KinematicCar(dt=0.05)  # dt = publish rate?
        # car = BicycleModel(dt=0.05)  # dt = publish rate?

        self.N = 10
        self.ocp = Ocp(
            car.nx,
            car.nu,
            N=self.N,
            F=car.F,
            T=car.dt * self.N,
            terminal_constraint=False,
            integrator="euler",
        )

        Q = np.diag([1e-2, 1e-2, 0, 1e-2])
        Q2 = np.diag([1e-2, 1e-2, 0, 1e-2])
        R = np.diag([1e-4, 2e-1])

        # Weight matrices for the terminal cost
        P = np.diag([0, 0, 0, 0])

        self.ocp.running_cost = (
            (self.ocp.x - self.ocp.x_ref).T @ Q @ (self.ocp.x - self.ocp.x_ref)
            + (self.ocp.x - self.ocp.x_ref_mid).T
            @ Q2
            @ (self.ocp.x - self.ocp.x_ref_mid)
            + self.ocp.u.T @ R @ self.ocp.u
        )
        self.ocp.terminal_cost = (
            (self.ocp.x - self.ocp.x_ref).T @ P @ (self.ocp.x - self.ocp.x_ref)
        )

        # constraints
        max_steering_angle = 1
        # max_steering_rate = np.pi / 4 / (5 * car.dt)
        max_v = self.speed_target
        self.ocp.subject_to(self.ocp.bounded(0, self.ocp.U[0, :], max_v))
        self.ocp.subject_to(
            self.ocp.bounded(-max_steering_angle, self.ocp.U[1, :], max_steering_angle)
        )
        self.mpc = MPC_tracking(self.ocp)

    def doActivate(self):
        self.trajectory = Trajectory()

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

    def getPathplanningUpdate(self, msg: Path):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        The path should be relative to self.base_link_frame. Otherwise it will transform to it
        """

        # Transform received message to self.base_link_frame
        trans = self.tf_buffer.lookup_transform(
            self.base_link_frame,
            msg.header.frame_id,
            msg.header.stamp,
        )
        new_header = Header(frame_id=self.base_link_frame, stamp=msg.header.stamp)
        transformed_path = Path(header=new_header)
        for pose in msg.poses:
            pose_t = do_transform_pose(pose, trans)
            transformed_path.poses.append(pose_t)

        # Create a new path
        current_path = np.zeros((0, 2))
        for pose in transformed_path.poses:
            current_path = np.vstack(
                (current_path, [pose.pose.position.x, pose.pose.position.y])
            )

        # Save current path and time of transformation to self.trajectory
        self.trajectory.points = current_path
        self.trajectory.time_source = msg.header.stamp

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
                    speed_target = rospy.get_param("~speed/target", 3.0)
                    rospy.loginfo(f"Speed target: {self.speed_target}")

                    # Change velocity constraints when speed target changes
                    if speed_target != self.speed_target:
                        self.speed_target = speed_target
                        self.ocp.opti.subject_to()
                        self.ocp.opti.subject_to(
                            self.ocp.opti.bounded(
                                0, self.ocp.U[0, :], self.speed_target
                            )
                        )
                        self.ocp.opti.subject_to(
                            self.ocp.opti.bounded(-1, self.ocp.U[1, :], 1)
                        )
                        self.ocp._set_continuity(1)

                    # First try to get a target point
                    # Change the look-ahead distance (minimal_distance)  based on the current speed
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

                    # Calculate target point
                    target_x, target_y = self.trajectory.calculate_target_point(
                        self.minimal_distance,
                    )
                    target_x_mid, target_y_mid = self.trajectory.calculate_target_point(
                        self.minimal_distance / 2,
                    )
                    self.mpc.reset()
                    init_state = [0, 0, 0, self.actual_speed]
                    goal_state = [target_x, target_y, 0, self.speed_target]
                    goal_state_mid = [target_x_mid, target_y_mid, 0, self.speed_target]

                    # goal_state = self.trajectory.points[:self.N+1, :].T
                    # goal_state = np.pad(goal_state, ((0, 2), (0, 0)), mode='constant', constant_values=(0, self.speed_target))

                    self.mpc.X_init_guess = target_x
                    current_state = init_state

                    u, info = self.mpc(current_state, goal_state, goal_state_mid)
                    # current_state = info["X_sol"][:, 1]

                    X_closed_loop = np.array(self.mpc.X_trajectory)
                    U_closed_loop = np.array(self.mpc.U_trajectory)

                    rospy.loginfo(f"X_closed_loop: {X_closed_loop}")
                    rospy.loginfo(f"U_closed_loop: {U_closed_loop}")

                    # self.steering_cmd.data = self.steering_cmd.data * self.steering_transmission + U_closed_loop[0, 1]
                    self.steering_cmd.data = U_closed_loop[0, 1]
                    self.velocity_cmd.data = U_closed_loop[0, 0]

                    self.diagnostics_pub.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[CTRL PP] Target Point Status",
                            message="Target point found.",
                        )
                    )

                    # Go ahead and drive. But adjust speed in corners
                    # self.velocity_cmd.data = self.speed_target

                    # Publish to velocity and position steering controller
                    self.steering_cmd.data /= self.steering_transmission
                    self.steering_pub.publish(self.steering_cmd)

                    self.velocity_cmd.data /= (
                        self.wheelradius
                    )  # Velocity to angular velocity
                    self.velocity_pub.publish(self.velocity_cmd)

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


node = MPC()
