import can
import cantools
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistWithCovariance, Twist
import numpy as np


class OdriveConverter:
    def __init__(self, bus = None):
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
        self.axis0_frame = rospy.get_param("~axis0/frame", "ugr/car_axis0")
        self.axis1_frame = rospy.get_param("~axis1/frame", "ugr/car_axis1")

        self.odrive_db = cantools.database.load_file(rospy.get_param("~odrive_dbc", "../odrive.dbc"))

        if bus is not None:
            self.bus = bus

    def handle_odrive_vel_msg(self, msg : can.Message, axis_id: int, cmd_id: int) -> None:
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
        twist_msg.twist.covariance = np.diag([0.1, 0.1, 0, 0, 0, 0]).reshape((1, 36)).tolist()[0]
        twist_msg.twist.twist = Twist()

        speed = can_msg["Vel_Estimate"]  # in rev/s
        speed *= np.pi * self.wheel_diameter
        twist_msg.twist.twist.linear.x = speed if axis_id == 1 else -speed # The left wheel is inverted

        if axis_id == 1:
            self.vel_right.publish(twist_msg)
        elif axis_id == 2:
            self.vel_left.publish(twist_msg)

class OdriveController:
    def __init__(self):
        self.vel_right = rospy.Publisher(
            "/output/vel0",
            TwistWithCovarianceStamped,
            queue_size=10,
        )

        self.odrive_db = cantools.database.load_file(rospy.get_param("~odrive_dbc", "../odrive.dbc"))

        # Test code
        for i in range(100):
            self.publish_torque_command(i/100, 0)
            self.publish_torque_command(i/100, 1)

    def publish_torque_command(self, torque: float, axis: int) -> None:
        """Publishes a drive command with a given torque to the ODrive
        
        Args:
            torque: the requested torque
            axis: 0 is right, 1 is left
        """
        if self.bus is None:
            rospy.logerr("The ODrive package was not configured to send messages, please run it as a separate node.")
            return
        
        torque_msg = self.odrive_db.get_message_by_name('Set_Input_Torque')
        data = torque_msg.encode({'Input_Torque': torque})

        can_id = axis << 5 | 0xE

        self.bus.send(can.Message(arbitration_id=can_id, data=data))




if __name__ == "__main__":
    try:
        # create a bus instance
        bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        odc = OdriveController(bus)
    except rospy.ROSInterruptException:
        pass
