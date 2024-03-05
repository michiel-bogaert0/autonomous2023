import numpy as np
import rospy
from node_fixture.fixture import SLAMStatesEnum


class LongitudinalControl:
    def __init__(self, publish_rate):
        """
        Initializes the LongitudinalControl class.

        Args:
            publish_rate: The rate at which control commands are published.

        Attributes:
            publish_rate: The rate at which control commands are published.
            speed_minimal_distance: The minimal distance at which a target point can be found.
            speed_target: The target speed defined per mission and vehicle.
            max_acceleration: The maximum acceleration.
            min_speed: The minimum speed.
            max_speed: The maximum speed.
            straight_ratio: The ratio for straight path control. Ratios higher than this ratio indicate a straight line.
            adaptive_velocity (bool): Flag indicating whether adaptive velocity is enabled.
        """
        self.publish_rate = publish_rate
        self.speed_minimal_distance = rospy.get_param("~speed/minimal_distance", 12.0)
        self.speed_target = rospy.get_param("/speed/target", 3.0)
        self.max_acceleration = rospy.get_param("~max_acceleration", 2)
        self.min_speed = rospy.get_param("~speed/min", 3.0)
        self.max_speed = rospy.get_param("~speed/max", 15.0)
        self.straight_ratio = rospy.get_param("~straight_ratio", 0.96)
        self.adaptive_velocity = rospy.get_param("/speed/adaptive_velocity", False)

    def handle_longitudinal_control(self, trajectory, slam_state, actual_speed):
        """
        Handles the longitudinal control for the vehicle. Based on self.adaptive_velocity, the target speed is calculated.

        Args:
            trajectory (Trajectory): The trajectory class.
            slam_state (SLAMStatesEnum): The current state of the SLAM system.
            actual_speed (float): The current actual speed of the vehicle.

        Returns:
            float: The target speed for the vehicle.

        """
        self.speed_target = rospy.get_param("/speed/target", 3.0)

        if self.adaptive_velocity and slam_state == SLAMStatesEnum.RACING:
            return self.get_velocity_target(trajectory, actual_speed)
        else:
            return self.speed_target

    def get_velocity_target(self, trajectory, actual_speed):
        """
        Calculates the target velocity based on the given trajectory and actual speed.
        With this given trajectory, a target point is found and a ratio is calculated.
        The ratio is then used to calculate the target speed.

        Args:
            trajectory (Trajectory): The trajectory object containing the path information.
            actual_speed (float): The current actual speed of the vehicle.

        Returns:
            float: The target velocity to be achieved.

        """
        speed_target_x, speed_target_y, _ = trajectory.calculate_target_point(
            self.speed_minimal_distance
        )

        if speed_target_x == 0 and speed_target_y == 0:
            return self.speed_target

        # Calculate ratio
        direct_distance = np.sqrt(speed_target_x**2 + speed_target_y**2)
        ratio = direct_distance / self.speed_minimal_distance

        # Calculate max speed based on max acceleration
        max_allowed_speed = actual_speed + self.max_acceleration / self.publish_rate

        # Calculate speed target
        speed_candidate = max(
            self.min_speed
            + (ratio - 0.6) / (1 - 0.6) * (self.max_speed - self.min_speed),
            0,
        )

        # Update speed target, not higher than max allowed speed
        return min(speed_candidate, max_allowed_speed)
