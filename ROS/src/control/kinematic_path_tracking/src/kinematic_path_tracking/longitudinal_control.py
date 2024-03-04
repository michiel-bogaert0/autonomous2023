import numpy as np
import rospy
from node_fixture.fixture import SLAMStatesEnum


class LongitudinalControl:
    def __init__(self, publish_rate):
        self.publish_rate = publish_rate
        self.speed_minimal_distance = rospy.get_param("~speed/minimal_distance", 12.0)
        self.speed_target = rospy.get_param("/speed/target", 3.0)
        self.max_acceleration = rospy.get_param("~max_acceleration", 2)
        self.min_speed = rospy.get_param("~speed/min", 3.0)
        self.max_speed = rospy.get_param("~speed/max", 15.0)
        self.straight_ratio = rospy.get_param("~straight_ratio", 0.96)
        self.adaptive_velocity = rospy.get_param("/speed/adaptive_velocity", False)

    def handle_longitudinal_control(self, trajectory, slam_state, actual_speed):
        self.speed_target = rospy.get_param("/speed/target", 3.0)

        if self.adaptive_velocity and slam_state == SLAMStatesEnum.RACING:
            return self.get_velocity_target(trajectory, actual_speed)
        else:
            return self.speed_target

    def get_velocity_target(self, trajectory, actual_speed):
        mission = rospy.get_param("/mission", "")

        speed_target_x, speed_target_y, _ = trajectory.calculate_target_point(
            self.speed_minimal_distance, mission
        )

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
