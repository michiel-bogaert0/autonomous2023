import numpy as np
import rospy
from node_fixture.fixture import SLAMStatesEnum


class LongitudinalControl:
    def __init__(
        self,
        speed_minimial_distance,
        speed_target,
        max_acceleration,
        publish_rate,
        min_speed,
        max_speed,
        straight_ratio,
    ):
        self.speed_minimial_distance = speed_minimial_distance
        self.speed_target = speed_target
        self.max_acceleration = max_acceleration
        self.publish_rate = publish_rate
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.straight_ratio = straight_ratio
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
            self.speed_minimial_distance, mission
        )

        # Calculate ratio
        direct_distance = np.sqrt(speed_target_x**2 + speed_target_y**2)
        ratio = direct_distance / self.speed_minimial_distance

        # Calculate max speed based on max acceleration
        max_allowed_speed = actual_speed + self.max_acceleration / self.publish_rate

        # Calculate speed target
        speed_candidate = max(
            self.min_speed
            + (ratio - 0.6) / (1 - 0.6) * (self.max_speed - self.min_speed),
            0,
        )

        # Update speed target, not higher than max allowed speed
        return (
            min(speed_candidate, max_allowed_speed)
            if ratio < self.straight_ratio
            else speed_candidate
        )
