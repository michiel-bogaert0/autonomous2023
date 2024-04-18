#! /usr/bin/python3
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from kinematic_path_tracking.base_class import KinematicTrackingNode
from node_fixture.fixture import DiagnosticStatus, create_diagnostic_message


class PurePursuit(KinematicTrackingNode):
    def __init__(self):
        super().__init__("pure_pursuit_control")

    def doConfigure(self):
        super().doConfigure()

        self.speed_start = rospy.get_param("~speed_start", 10)
        self.speed_stop = rospy.get_param("~speed_stop", 50)
        self.distance_start = rospy.get_param("~distance_start", 1.2)
        self.distance_stop = rospy.get_param("~distance_stop", 2.4)

        self.cog_to_front_axle = rospy.get_param("~cog_to_front_axle", 0.72)
        self.reference_pose = [self.cog_to_front_axle, 0]

    def __process__(self):
        """
        Processes the current path and calculates the target point for the car to follow
        """

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
        R = (
            (target_x - self.reference_pose[0]) ** 2
            + (target_y - self.reference_pose[1]) ** 2
        ) / (2 * (target_y - self.reference_pose[1]))

        self.steering_cmd.data = self.symmetrically_bound_angle(
            np.arctan2(1.0, R), np.pi / 2
        )
        self.steering_cmd.data /= self.steering_transmission

        self.diagnostics_pub.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[CTRL PP] Target Point Status",
                message="Target point found.",
            )
        )

        # Publish to velocity and position steering controller
        self.steering_pub.publish(self.steering_cmd)

        # Publish target point for visualization
        point = PointStamped()
        point.header.stamp = position_target_time
        point.header.frame_id = self.base_link_frame
        point.point.x = target_x
        point.point.y = target_y

        self.vis_pub.publish(point)


node = PurePursuit()
