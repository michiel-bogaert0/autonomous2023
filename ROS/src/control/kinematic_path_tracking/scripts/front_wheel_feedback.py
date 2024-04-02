#! /usr/bin/python3
import numpy as np
import rospy
from kinematic_path_tracking.base_class import KinematicTrackingNode


class FrontWheelFeedback(KinematicTrackingNode):
    def __init__(self):
        """
        See https://arxiv.org/pdf/1604.07446.pdf as a reference

        This controller extends the method of the paper by not directly calculating the steering angle (as proposed in the paper),
        but by adding a PID controller to the transversal error.
        """

        super().__init__("front_wheel_feedback_control")

    def doConfigure(self):
        super().doConfigure()

        # PID for the transversal error thingy
        self.Kp = rospy.get_param("~Kp", 0.5)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.Kd = rospy.get_param("~Kd", 0.0)

        self.t = rospy.Time.now().to_sec()

        self.integral = 0.0
        self.last_error = 0.0

    def __process__(self):
        """
        Processes the current path and calculates the target point for the car to follow
        """

        # Calculate the steering angle
        dt = rospy.Time.now().to_sec() - self.t
        self.t = rospy.Time.now().to_sec()

        # Calculate the transversal error
        trans_error, heading_error = self.trajectory.calculate_transversal_error()

        pid_e = trans_error / self.actual_speed

        self.integral += pid_e * dt

        pid_u = (
            self.Kp * pid_e
            + self.Ki * self.integral
            + self.Kd * (pid_e - self.last_error) / dt
        )

        self.last_error = pid_e

        # Convert to steering angle
        self.steering_cmd.data = np.arctan(pid_u) + heading_error
        self.steering_cmd.data /= self.steering_transmission

        # Velocity point
        self.speed_target = rospy.get_param("/speed/target", 3.0)

        # Publish to velocity and position steering controller
        self.steering_pub.publish(self.steering_cmd)


node = FrontWheelFeedback()
