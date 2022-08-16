import math


class PID:
    """
    Simple helper class for PID
    """

    def __init__(self, Kp, Ki, Kd, reset_rotation=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0
        self.pre_error = 0

        self.reset_rotation = reset_rotation

    def __call__(self, error: float) -> float:
        """
        Returns the PID control signal, given the current error and internal state
        """

        if self.reset_rotation:
            error = PID.pi_to_pi(error)

            self.integral = self.integral + error
            pid = PID.pi_to_pi(self.Kp * error + self.Kd * (error - self.pre_error) + self.Ki * self.integral)
        else:
            self.integral = self.integral + error
            pid = self.Kp * error + self.Kd * (error - self.pre_error) + self.Ki * self.integral

        self.pre_error = error

        return pid

    @staticmethod
    def pi_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi