from node_fixture.node_fixture import ROSNode,AddSubscriber
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Empty

MAX_VELOCITY = 3 #in revs/s

class ResNode(RosNode):
    def __init__(self,bus=None):
        super.__init__("ResNode")
        self.modus =  "Brake"
        self.velocity = 0
        self.bus = bus
        self.odrive_db = cantools.database.load_file(
            rospy.get_param("~odrive_dbc", "../../../mechatronics/can_node/odrive.dbc")
        )
    

    @AddSubscriber("/input/start")
    def send_start_signal(msg:Empty)->None:
        """ sets the modus on the value received from res"""
        # send value to odrive to start
        self.modus = "Start"

    @AddSubscriber("/input/stop")
    def send_start_signal(msg:Empty)->None:
        """ sets the modus on the value received from res"""
        # send value to odrive to start
        self.modus = "Brake"




    @AddSubscriber("/input/drive_command")
    def update_control_velocity(msg:ControlCommand)->None:
        velocity_setpoint = msg.throttle*MAX_VELOCITY #set velocity based on controlcommand
        
        if self.modus == "Brake":
            self.velocity = 0
        else:
            self.velocity = velocity_setpoint
        
        self.set_odrive_velocity(self.velocity,1)
        self.set_odrive_velocity(self.velocity,2)






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

        can_id = axis << 5 | 0x00E

        self.bus.send(
            can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        )

if __name__ == "__main__":
    try:
        # create a bus instance
        bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        ResController = ResNode(bus)s

    except rospy.ROSInterruptException:
        pass