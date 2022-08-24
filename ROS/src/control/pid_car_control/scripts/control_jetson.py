#!/usr/bin/env python3
import rospy
import cantools
import can
from fs_msgs.msg import ControlCommand

MAX_VELOCITY = 5  # in rev/s
STEER_MAX_STEP = 1000  # This must correspond to the value on the RC Teensy

CAN_NODE_ID = 0xE0
CAN_REBOOT_ID = 0x1
CAN_STEER_ID = 0x3

class JetsonController:
    def __init__(self, bus=None):
        rospy.init_node("jetson_controller")

        self.bus = bus
        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../../../mechatronics/can_node/odrive.dbc")
        )

        self.control_cmd_sub = rospy.Subscriber(
            "/input/control_command", ControlCommand, self.publish_drive_command
        )

        rospy.spin()

    def publish_drive_command(self, msg: ControlCommand) -> None:
        """Gets called on each ControlCommand
        """
        if msg.brake > 0:
            self.set_odrive_velocity(0, 1)
            self.set_odrive_velocity(0, 2)
            return

        # self.set_odrive_velocity(msg.throttle * MAX_VELOCITY, 1)
        # self.set_odrive_velocity(-1 * msg.throttle * MAX_VELOCITY, 2)

        self.set_steering_setpoint(msg.steering)
        
    def set_steering_setpoint(self, steering: float) -> None:
        """Sends a CAN message to actuate the steering system

        The input must be converted from [-1, 1] to a steering range [-STEER_MAX_STEP, STEER_MAX_STEP]
        as defined in the RC Teensy code.

        Args:
            steering: a float between -1 and 1
        """

        steering = int(steering * STEER_MAX_STEP)
        id = CAN_NODE_ID << 5 | CAN_STEER_ID

        msg = can.Message(
            arbitration_id=id, data=steering.to_bytes(2, "little", signed=True)
        )

        self.bus.send(msg)

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
        data = vel_msg.encode({"Input_Torque_FF": 0, "Input_Vel": vel})

        can_id = axis << 5 | 0x00D

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

        odc = JetsonController(bus)

    except rospy.ROSInterruptException:
        pass