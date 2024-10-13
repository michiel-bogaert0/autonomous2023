#!/usr/bin/env python3
import traceback

import rospy
import tf2_ros as tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import DiagnosticArray, ROSNode, StateMachineScopeEnum
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Float64
from ugr_msgs.msg import State

from .longitudinal_control import LongitudinalControl
from .trajectory import Trajectory


class KinematicTrackingNode(ManagedNode):
    def __init__(self, name):
        super().__init__(name)

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.publish_rate = rospy.get_param("~publish_rate", 10)

        # Publishers for the controllers
        # Controllers themselves spawned in the state machines respective launch files

        self.velocity_pub = super().AddPublisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )

        self.axis0_velocity_pub = super().AddPublisher(
            "/ugr/car/axis0_velocity_controller/command", Float64, queue_size=10
        )
        self.axis1_velocity_pub = super().AddPublisher(
            "/ugr/car/axis1_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_pub = super().AddPublisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
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
        self.vis_pub = super().AddPublisher(
            "/output/target_point", PointStamped, queue_size=10  # warning otherwise
        )

        self.state_sub = super().AddSubscriber(
            "/state/slam", State, self.handle_state_change
        )

        self.spin()

    def doConfigure(self):
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")

        self.wheelradius = rospy.get_param("/ugr/car/wheel/radius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0

        self.received_path = None

        self.speed_target = rospy.get_param("/speed/target", 3.0)
        self.steering_transmission = rospy.get_param(
            "/ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

        self.slam_state = None

    def doActivate(self):
        self.trajectory = Trajectory(self.tf_buffer)
        self.longitudinal_control = LongitudinalControl(self.publish_rate)

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

    def handle_state_change(self, state: State):
        """
        Handles state changes, only for SLAM state
        """
        if state.scope == StateMachineScopeEnum.SLAM:
            self.slam_state = state.cur_state

    def getPathplanningUpdate(self, msg: Path):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        Does not do anything with it, unless the control loop is active
        """
        if msg is None:
            return

        self.received_path = msg

    def doUpdate(self):
        """
        Actually processes a new path
        The path should be relative to world frame. Otherwise it will transform to it
        """

        if self.received_path is None:
            return

        msg = self.received_path

        # Transform received message to world frame
        trans = self.tf_buffer.lookup_transform(
            self.world_frame, msg.header.frame_id, msg.header.stamp
        )
        transformed_path = ROSNode.do_transform_path(msg, trans)

        # Save current path and time of transformation to self.trajectory
        self.trajectory.set_path(transformed_path)
        self.received_path = None

    def symmetrically_bound_angle(self, angle, max_angle):
        """
        Helper function to bound {angle} to [-max_angle, max_angle]
        """
        return (angle + max_angle) % (2 * max_angle) - max_angle

    def __process__(self):
        raise NotImplementedError

    def active(self):
        """
        Start sending updates. If the data is too old, brake.
        """

        try:
            # First fetch new paths, then process current path
            self.doUpdate()

            self.__process__()

            self.velocity_cmd.data = (
                self.longitudinal_control.handle_longitudinal_control(
                    self.trajectory, self.slam_state, self.actual_speed
                )
            )

            # Brake when no path has been found!
            if len(self.trajectory.path_blf) == 0:
                self.velocity_cmd.data = 0

            self.velocity_cmd.data /= self.wheelradius  # Velocity to angular velocity
            self.velocity_pub.publish(self.velocity_cmd)
            self.axis0_velocity_pub.publish(self.velocity_cmd)
            self.axis1_velocity_pub.publish(self.velocity_cmd)

        except Exception as e:
            rospy.logwarn(f"{rospy.get_name()} has caught an exception: {e}")
            rospy.logwarn(traceback.format_exc())
