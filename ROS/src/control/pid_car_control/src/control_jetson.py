import rospy
import cantools
import can
from fs_msgs.msg import ControlCommand

MAX_VELOCITY = 3 # in rev/s

class OdriveController:
    def __init__(self, bus=None):
        rospy.init_node("jetson_controller")

        self.bus = bus
        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../../../mechatronics/can_node/odrive.dbc")
        )

        self.pub = rospy.Subscriber(
            "/input/drive_command", ControlCommand, self.publish_odrive_command
        )

    def publish_drive_command(self, msg: ControlCommand) -> None:
        """Gets called on each ControlCommand
        """
        if msg.brake > 0:
            self.set_odrive_velocity(0, 1)
            self.set_odrive_velocity(0, 2)
            return

        self.set_odrive_velocity(msg.throttle * MAX_VELOCITY, 1)
        self.set_odrive_velocity(msg.throttle * MAX_VELOCITY, 2)

        # TODO also actuate the steering


    def set_odrive_velocity(self, vel: float, axis: int) -> None:
        """Publishes a drive command with a given velocity to the ODrive

        Args:
            vel: the requested velocity
            axis: 1 is right, 2 is left
        """
        if self.bus is None:
            rospy.logerr(
                "The ODrive package was not configured to send messages, please run it as a separate node."
            )
            return

        vel_msg = self.odrive_db.get_message_by_name("Set_Input_Vel")
        data = vel_msg.encode({"Input_Torque_FF": 0})
        data = vel_msg.encode({"Input_Vel": vel})

        can_id = axis << 5 | 0xE

        self.bus.send(
            can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        )


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
