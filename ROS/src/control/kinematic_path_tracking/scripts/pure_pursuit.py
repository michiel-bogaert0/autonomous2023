#! /usr/bin/python3
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from kinematic_path_tracking.base_class import KinematicTrackingNode
from node_fixture.fixture import (
    DiagnosticStatus,
    SLAMStatesEnum,
    create_diagnostic_message,
)


class PurePursuit(KinematicTrackingNode):
    def __init__(self):
        super().__init__("pure_pursuit_control")

    def doConfigure(self):
        super().doConfigure()

        # For lookahead distance in explo
        self.speed_start_explo = rospy.get_param("~speed_start_explo", 10)
        self.speed_stop_explo = rospy.get_param("~speed_stop_explo", 50)
        self.distance_start_explo = rospy.get_param("~distance_start_explo", 1.2)
        self.distance_stop_explo = rospy.get_param("~distance_stop_explo", 2.4)
        # For lookahead diastance in racing
        self.speed_start_racing = rospy.get_param("~speed_start_racing", 10)
        self.speed_stop_racing = rospy.get_param("~speed_stop_racing", 50)
        self.distance_start_racing = rospy.get_param("~distance_start_racing", 1.2)
        self.distance_stop_racing = rospy.get_param("~distance_stop_racing", 2.4)

        self.L = rospy.get_param("/ugr/car/wheelbase", 0.72)

    def __process__(self):
        """
        Processes the current path and calculates the target point for the car to follow
        """

        # Change the look-ahead distance (minimal_distance)  based on the current speed
        # Exploration
        if self.slam_state == SLAMStatesEnum.EXPLORATION:
            self.calculate_minimal_distance(
                self.speed_start_explo,
                self.speed_stop_explo,
                self.distance_start_explo,
                self.distance_stop_explo,
            )
        # Racing
        elif self.slam_state == SLAMStatesEnum.RACING:
            self.calculate_minimal_distance(
                self.speed_start_racing,
                self.speed_stop_racing,
                self.distance_start_racing,
                self.distance_stop_racing,
            )

        # Calculate target point
        (
            target_x,
            target_y,
            position_target_time,
        ) = self.trajectory.calculate_target_point(self.minimal_distance)

        if target_x == 0 and target_y == 0:
            self.diagnostics_pub.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.ERROR,
                    name="[CTRL PP] Target Point Status",
                    message="No target point found.",
                )
            )
            self.steering_cmd.data = 0.0
            self.steering_pub.publish(self.steering_cmd)

            return

        # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
        R = ((target_x) ** 2 + (target_y) ** 2) / (2 * (target_y))

        self.steering_cmd.data = self.symmetrically_bound_angle(
            np.arctan2(self.L, R), np.pi / 2
        )

        self.steering_cmd.data /= self.steering_transmission

        self.diagnostics_pub.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[CTRL PP] Target Point Status",
                message="Target point found.",
            )
        )

        # Publish to position steering controller
        self.steering_pub.publish(self.steering_cmd)

        # Publish target point for visualization
        point = PointStamped()
        point.header.stamp = position_target_time
        point.header.frame_id = self.base_link_frame
        point.point.x = target_x
        point.point.y = target_y

        self.vis_pub.publish(point)

    def calculate_minimal_distance(
        self, speed_start, speed_stop, distance_start, distance_stop
    ):
        if self.actual_speed < speed_start:
            self.minimal_distance = distance_start
        elif self.actual_speed < speed_stop:
            self.minimal_distance = distance_start + (
                distance_stop - distance_start
            ) / (speed_stop - speed_start) * (self.actual_speed - speed_start)
        else:
            self.minimal_distance = distance_stop


node = PurePursuit()
