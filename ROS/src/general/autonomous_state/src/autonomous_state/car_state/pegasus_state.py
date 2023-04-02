import rospy
from can_msgs.msg import Frame
from car_state import carStateEnum, CarState
from node_fixture import AutonomousStatesEnum, AutonomousMission
import can
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
import rosparam

class PegasusState(CarState):

    """
    Example implementation for Pegasus

    Note: Not everything is monitored in Pegasus.
    When this is the case, it is clearly documented
    """

    def __init__(self) -> None:

        rospy.Subscriber("/input/can", Frame, self.handle_can)

        self.res_go_signal = False
        self.res_estop_signal = False

        self.odrive_hb = rospy.Time.now().to_sec()
        self.teensy_hb = rospy.Time.now().to_sec()

        self.as_ready_time = rospy.Time.now().to_sec()

        self.odom = Odometry()
        self.control = ControlCommand()

        self.state = {
            "TS": carStateEnum.UNKOWN,
            "ASMS": carStateEnum.UNKOWN,
            "R2D": carStateEnum.UNKOWN,
            "ASB": carStateEnum.UNKOWN,
            "EBS": carStateEnum.UNKOWN,
        }

        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        self.as_state = AutonomousStatesEnum.ASOFF

    def handle_can(self, frame: Frame):
        """
        Handles incoming CAN message, but as a subscriber callback
        This way, we can put all HW/SW interfacing code in a single CAN driver.

        TS: 
            We consider the TS to be 'on' if the ODrive sends heartbeat messages
        
        ASB:
            We don't have actual brakes, so the ASB is considered to be 'activated' when
            the ODrive sends heartbeat messages

        ASMS:
            The switch in the back. The Teensy reports the status of the switch

        R2D: (T 14.9.3)
            Is only set to 'on' when AS is at least 5 seconds in ASReady and 
            the RES reports 'Go' being pressed

        EBS:
            If we do not receive heartbeats from the teensy, that means that the power
            has been cut off from the AS controllers. This automatically means EBS has been triggered

            If the E-stop of the RES has been triggered, the power will be cut, which also means the EBS
            has been triggered.
            
            Triggering EBS from within the AS is not possible in Pegasus. 

            Disabling the EBS ('off') is also not possible. EBS is either 'on' (armed) or 'activated'
        """
        
        # ODrive
        axis_id = frame.id >> 5
        if axis_id == 1 or axis_id == 2:
            cmd_id = frame.id & 0b11111

            # Heartbeat message odrive
            if cmd_id == 1:
                self.odrive_hb = rospy.Time.now().to_sec()
        
        # RES
        if frame.id == 0x191:
            self.res_go_signal = (frame.data[0] & 0b0000100) >> 2
            self.res_estop_signal = (frame.data[0] & 0b0000001)
        
        # Teensy 
        node_id = frame.id >> 5
        if node_id == 0xE0:
            cmd_id = frame.id & 0b11111

            # Heartbeat message Teensy
            if cmd_id == 0x4:
                self.teensy_hb = rospy.Time.now().to_sec()

            # ASMS message Teensy
            if cmd_id == 0x5:
                self.state["ASMS"] = frame.data[0]

    def update(self, state: AutonomousStatesEnum):

        # On a state transition, start 5 second timeout
        if state == AutonomousStatesEnum.ASREADY and self.as_state != AutonomousStatesEnum.ASREADY:
            self.as_ready_time = rospy.Time.now().to_sec()

        self.as_state = state

    
    def send_status_over_can(self):
        # https://www.formulastudent.de/fileadmin/user_upload/all/2022/rules/FSG22_Competition_Handbook_v1.1.pdf

        
        # 0x500 whatever
        # 0x501 whatever

        # 0x502

        data = [0, 0, 0, 0, 0]

        # Bit 0 - 2
        bits = 0
        if self.as_state == AutonomousStatesEnum.ASOFF:
            bits = 1
        elif self.as_state == AutonomousStatesEnum.ASREADY:
            bits = 2
        elif self.as_state == AutonomousStatesEnum.ASDRIVE:
            bits = 3
        elif self.as_state == AutonomousStatesEnum.ASEMERGENCY:
            bits = 4
        elif self.as_state == AutonomousStatesEnum.ASFINISHED:
            bits = 5

        data[0] |= bits

        # Bit 3 - 4
        bits = 0
        if self.state["EBS"] == carStateEnum.OFF:
            bits = 1
        elif self.state["EBS"] == carStateEnum.ON:
            bits = 2
        elif self.state["EBS"] == carStateEnum.ACTIVATED:
            bits = 3

        data[0] |= bits << 3 

        # Bits 5 - 7
        bits = 0
        mission = rosparam.get_param("/mission") if rosparam.has_param("/mission") else ""

        if mission == AutonomousMission.ACCELERATION:
            bits = 1
        elif mission == AutonomousMission.SKIDPAD:
            bits = 2
        elif mission == AutonomousMission.TRACKDRIVE:
            bits = 3
        elif mission == AutonomousMission.EBSTEST:
            bits = 4
        elif mission == AutonomousMission.INPSPECTION:
            bits = 5
        elif mission == AutonomousMission.AUTOCROSS:
            bits = 6

        data[0] |= bits << 5
        
        # De rest whatever

        canmsg = can.Message(
            arbitration_id=0x502,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(canmsg)


    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """

        t = rospy.Time.now().to_sec()

        # R2D
        if self.res_go_signal and t - self.as_ready_time > 5.0:
            self.state["R2D"] = carStateEnum.ON
        elif self.as_state != AutonomousStatesEnum.ASDRIVE:
            self.state["R2D"] = carStateEnum.OFF

        # TS and ASB
        self.state["TS"] = t - self.odrive_hb < 0.2
        self.state["ASB"] = t - self.odrive_hb < 0.2

        # EBS
        self.state["EBS"] = carStateEnum.ACTIVATED if self.res_estop_signal or t - self.teensy_hb < 0.2 else carStateEnum.ON

        return self.state
