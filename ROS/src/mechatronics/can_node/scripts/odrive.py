import cantools
from can import Message
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistWithCovariance, TwistWithCovarianceStamped
from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState
from node_fixture import serialcan_to_roscan

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

        self.power = rospy.Publisher(
            "/output/power",
            BatteryState,
            queue_size=10
        )

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 8 * 2.54 / 100)  # in m
        self.axis0_frame = rospy.get_param("~axis0/frame", "ugr/car_base_link/axis0")
        self.axis1_frame = rospy.get_param("~axis1/frame", "ugr/car_base_link/axis1")

        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../odrive.dbc")
        )

        if bus is not None:
            self.bus = bus

        self.t = rospy.Time.now().to_sec()


    def handle_power_msg(
        self, msg: Frame, cmd_id: int
    ):
        can_msg = self.odrive_db.decode_message(cmd_id, msg.data)
        voltage = can_msg["Vbus_Voltage"]

        # Interpolate using values from this site (seems to match quite well) https://blog.ampow.com/lipo-voltage-chart/
        x =[19.64, 21.65, 22.12, 22.24, 22.36, 22.48, 22.6, 22.72, 22.77, 22.89, 23.01, 23.13, 23.25, 23.48, 23.72, 23.9, 24.14, 24.49, 24.67, 24.9, 25.2]
        y = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]

        percentage = min(max(np.interp(voltage, x, y) / 100, 0.0), 1.0)

        bat_state = BatteryState()
        bat_state.voltage = voltage
        bat_state.percentage = percentage

        self.power.publish(bat_state)

    def handle_vel_msg(
        self, msg: Frame, axis_id: int, cmd_id: int
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
        twist_msg.header.stamp = rospy.Time.now()
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
