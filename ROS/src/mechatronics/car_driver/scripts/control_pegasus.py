#!/usr/bin/env python3
import rospy
import cantools
import can
from std_msgs.msg import Empty
from fs_msgs.msg import ControlCommand
from math import pi
from can_msgs.msg import Frame
from ugr_msgs.msg import State
from node_fixture import AutonomousStatesEnum, SLAMStatesEnum

CAN_NODE_ID = 0xE0
CAN_REBOOT_ID = 0x1
CAN_STEER_ID = 0x3
ODRIVE_VELOCITY_CONTROL_MODE = 2
ODRIVE_PASSTHROUGH_INPUT_MODE = 1


class PegasusController:
    def __init__(self):
        rospy.init_node("pegasus_controller")

        # Can stuff
        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../odrive.dbc")
        )

        # Subscriptions
        self.control_cmd_sub = rospy.Subscriber(
            "/input/control_command", ControlCommand, self.publish_drive_command
        )
        self.state_sub = rospy.Subscriber("/state", State, self.handle_state)

        # Publishers
        self.bus = rospy.Publisher("/output/can", Frame, queue_size=10)

        # Parameters
        self.max_velocity = rospy.get_param("~max_velocity", 1) / (
            pi * 8 * 2.54 / 100
        )  # Default is in rev/s, but launch file param must be in m/s
        self.steer_max_step = rospy.get_param("~steer_max_step", 1600)

        # Local parameters
        self.is_running = False

        # Set ODrive to velocity control mode
        self.set_odrive_velocity_control()

        rospy.spin()

    def handle_state(self, state: State):
        """
        Handles incoming state changes

        Args:
            state: A State message notifying this node of state changes
        """

        # Only allowed to control if: AS Drive and SLAM not finished
        if state.scope == "autonomous":
            self.is_running = state.cur_state == AutonomousStatesEnum.ASDRIVE
        elif state.scope == "slam" and self.is_running:
            self.is_running = state.cur_state != SLAMStatesEnum.FINISHED or state.cur_state != SLAMStatesEnum.FINISHING

        # If not running, stop!
        if not self.is_running:
            self.set_odrive_velocity(0, 1)
            self.set_odrive_velocity(0, 2)

    def publish_drive_command(self, msg: ControlCommand) -> None:
        """Gets called on each ControlCommand"""
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
        id = CAN_NODE_ID << 2 | CAN_STEER_ID

        msg = Frame()
        msg.id = id
        msg.data = steering.to_bytes(2, "little", signed=True)
        msg.dlc = 2

        self.bus.publish(msg)

    def set_odrive_velocity_control(self):
        """Sends a CAN message to switch the ODrive to velocity control mode"""

        ctrl_msg = self.odrive_db.get_message_by_name("Set_Controller_Mode")
        data = ctrl_msg.encode(
            {
                "Input_Mode": ODRIVE_PASSTHROUGH_INPUT_MODE,
                "Control_Mode": ODRIVE_VELOCITY_CONTROL_MODE,
            }
        )

        msg = Frame()
        msg.id = 1 << 5 | 0x00D
        msg.data = data
        msg.dlc = len(data)

        self.bus.publish(msg)  # Right

        msg.id = 2 << 5 | 0x00D
        self.bus.publish(msg)  # Left

    def set_odrive_velocity(self, vel: float, axis: int) -> None:
        """Publishes a drive command with a given velocity to the ODrive

        Args:
            vel: the requested velocity
            axis: 1 is right, 2 is left
        """

        vel_msg = self.odrive_db.get_message_by_name("Set_Input_Vel")
        data = vel_msg.encode({"Input_Torque_FF": 0, "Input_Vel": vel})

        can_id = axis << 5 | 0x00D
        
        msg = Frame()
        msg.id = can_id
        msg.data = data
        msg.dlc = len(data)

        self.bus.publish(msg)

if __name__ == "__main__":

    odc = PegasusController()
