#!/usr/bin/env python3
import rospy
import cantools
import can
from std_msgs.msg import Empty
from fs_msgs.msg import ControlCommand
from math import pi

CAN_NODE_ID = 0xE0
CAN_REBOOT_ID = 0x1
CAN_STEER_ID = 0x3

class JetsonController:
    def __init__(self, bus=None):
        rospy.init_node("jetson_controller")

        # Can stuff
        self.bus = bus
        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../../../mechatronics/can_node/odrive.dbc")
        )

        # Subscriptions
        self.control_cmd_sub = rospy.Subscriber(
            "/input/control_command", ControlCommand, self.publish_drive_command
        )
        self.start_sub = rospy.Subscriber("/input/start", Empty, self.send_start_signal)
        self.stop_sub = rospy.Subscriber("/input/stop", Empty, self.send_stop_signal)

        # Parameters
        self.max_velocity = rospy.get_param("~max_velocity", 3) / (pi * 8 * 2.54 / 100) # Default is in rev/s, but launch file param must be in m/s
        self.steer_max_step = rospy.get_param("~steer_max_step", 1600);

        # Local parameters
        self.is_running = False

        rospy.spin()

    def send_start_signal(self, msg : Empty) -> None:
        """ sets the modus on the value received from res"""
        self.is_running = True

    def send_stop_signal(self, msg : Empty) -> None:
        """ sets the modus on the value received from res, and actually stops Odrive"""
        # send value to odrive to stop
        self.is_running = False
        self.set_odrive_velocity(0, 1)
        self.set_odrive_velocity(0, 2)

    def publish_drive_command(self, msg: ControlCommand) -> None:
        """Gets called on each ControlCommand
        """
        if msg.brake > 0 or not self.is_running:
            self.set_odrive_velocity(0, 1)
            self.set_odrive_velocity(0, 2)
            return

        self.set_odrive_velocity(msg.throttle * self.max_velocity, 1)
        self.set_odrive_velocity(-1 * msg.throttle * self.max_velocity, 2)

        self.set_steering_setpoint(msg.steering)
        
    def set_steering_setpoint(self, steering: float) -> None:
        """Sends a CAN message to actuate the steering system

        The input must be converted from [-1, 1] to a steering range [-self.steer_max_step, self.steer_max_step]
        as defined in the RC Teensy code.

        Args:
            steering: a float between -1 and 1
        """

        steering = int(steering * self.steer_max_step)
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