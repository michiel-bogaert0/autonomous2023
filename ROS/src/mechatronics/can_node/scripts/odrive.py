import can
import cantools
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistWithCovariance, TwistWithCovarianceStamped


class OdriveConverter:
    def __init__(self, bus=None):
        self.vel_right = rospy.Publisher(
            "/output/vel0",
            TwistWithCovarianceStamped,
            queue_size=10,
        )
        self.vel_left = rospy.Publisher(
            "/output/vel1",
            TwistWithCovarianceStamped,
            queue_size=10,
        )

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 8 * 2.54 / 100)  # in m
        self.axis0_frame = rospy.get_param("~axis0/frame", "ugr/car_base_link/axis0")
        self.axis1_frame = rospy.get_param("~axis1/frame", "ugr/car_base_link/axis1")

        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../odrive.dbc")
        )

        if bus is not None:
            self.bus = bus


    def handle_vel_msg(
        self, msg: can.Message, axis_id: int, cmd_id: int
    ) -> None:
        """Publishes an ODrive velocity message to the correct ROS topic

        Args:
            msg: the CAN message
        """
        can_msg = self.odrive_db.decode_message(cmd_id, msg.data)

        # Encoder estimate
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = (
            self.axis0_frame if axis_id == 1 else self.axis1_frame
        )
        twist_msg.header.stamp = rospy.Time.from_sec(msg.timestamp)

        twist_msg.twist = TwistWithCovariance()

        # TODO Use actual covariance measurements (first we need data to estimate these)
        twist_msg.twist.covariance = (
            np.diag([0.1, 0.1, 0, 0, 0, 0]).reshape((1, 36)).tolist()[0]
        )
        twist_msg.twist.twist = Twist()

        speed = can_msg["Vel_Estimate"]  # in rev/s
        speed *= np.pi * self.wheel_diameter
        twist_msg.twist.twist.linear.x = (
            speed if axis_id == 1 else -speed
        )  # The left wheel is inverted

        if axis_id == 1:
            self.vel_right.publish(twist_msg)
        elif axis_id == 2:
            self.vel_left.publish(twist_msg)
