import time

import can
import rosparam
import rospy
from can_msgs.msg import Frame
from car_state import CarState, CarStateEnum
from node_fixture import AutonomousMission, AutonomousStatesEnum, serialcan_to_roscan
from std_msgs.msg import Bool, Float64


class Gen4State(CarState):
    def __init__(self) -> None:
        rospy.Subscriber("/ugr/car/can/rx", Frame, self.handle_can)
        rospy.Subscriber("/dio_driver_1/DI1", Bool, self.handle_sdc)  # sdc status
        rospy.Subscriber(
            "/dio_driver_1/DI2", Bool, self.handle_ts
        )  # start button side (driverless) --> TS
        rospy.Subscriber(
            "/dio_driver_1/DI3", Bool, self.handle_asms
        )  # ASMS status button
        rospy.Subscriber(
            "/dio_driver_1/DI4", Bool, self.handle_watchdog
        )  # watchdog status
        rospy.Subscriber(
            "/iologik/input1", Float64, self.handle_front_bp
        )  # front brake pressure
        rospy.Subscriber(
            "/iologik/input2", Float64, self.handle_rear_bp
        )  # rear brake pressure
        self.ebs1 = rospy.Publisher("/dio_driver_1/DO1", Bool, queue_size=10)  # ebs1
        self.ebs2 = rospy.Publisher("/dio_driver_1/DO2", Bool, queue_size=10)  # ebs2
        self.watchdog_trigger = rospy.Publisher(
            "/dio_driver_1/DO3", Bool, queue_size=10
        )  # watchdog trigger
        self.sdc_out = rospy.Publisher(
            "/dio_driver_1/DO4", Bool, queue_size=10
        )  # sdc out, not sure if i have to send it periodically
        self.res_go_signal = False
        self.res_estop_signal = False
        self.front_bp = None
        self.rear_bp = None
        self.as_ready_time = rospy.Time.now().to_sec()
        # DBS ACTIVATED VANAF WNR JE DIE EERSTE TEST HEBT GDN, ANDERS GWN ON
        self.state = {
            "TS": CarStateEnum.UNKOWN,
            "ASMS": CarStateEnum.UNKOWN,
            "R2D": CarStateEnum.UNKOWN,
            "ASB": CarStateEnum.ON,  # assumed to be on
            "EBS": CarStateEnum.UNKOWN,
        }

        self.bus = rospy.Publisher("/ugr/car/can/tx", Frame, queue_size=10)

        self.as_state = AutonomousStatesEnum.ASOFF

        time.sleep(1)

    def handle_can(self, frame: Frame):
        """
        Handles incoming CAN message, but as a subscriber callback
        This way, we can put all HW/SW interfacing code in a single CAN driver.
        """

        # RES
        if frame.id == 0x191:
            self.res_go_signal = (frame.data[0] & 0b0000100) >> 2
            self.res_estop_signal = not (frame.data[0] & 0b0000001)
            if self.res_estop_signal:
                self.activate_EBS()

    def activate_EBS(self):
        # activate EBS here
        # ebs1&2 activate bekabeling beide pinnen laag zetten

        self.state["EBS"] = CarStateEnum.ACTIVATED
        self.ebs1.publish(Bool(data=False))
        self.ebs2.publish(Bool(data=False))
        # sdc open yeeten

    def update(self, state: AutonomousStatesEnum):
        # On a state transition, start 5 second timeout
        if (
            state == AutonomousStatesEnum.ASREADY
            and self.as_state != AutonomousStatesEnum.ASREADY
        ):
            self.as_ready_time = rospy.Time.now().to_sec()

        self.as_state = state

        # watchdog togglen vanboven uitzetten onderaan hoog

        self.send_status_over_can()

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
        if self.state["EBS"] == CarStateEnum.OFF:
            bits = 1
        elif self.state["EBS"] == CarStateEnum.ON:
            bits = 2
        elif self.state["EBS"] == CarStateEnum.ACTIVATED:
            bits = 3

        data[0] |= bits << 3

        # Bits 5 - 7
        bits = 0
        mission = rosparam.get_param("/mission") if rospy.has_param("/mission") else ""

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
        elif mission == AutonomousMission.MANUAL:
            bits = 7

        data[0] |= bits << 5

        # De rest whatever

        canmsg = can.Message(
            arbitration_id=0x502,
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def handle_sdc(self, dio1: Bool):
        if dio1.data:  # what does 1 and 0 mean
            self.activate_EBS()

    def handle_ts(self, dio2: Bool):
        self.state["TS"] = CarStateEnum.ON if dio2.data else CarStateEnum.OFF

    def handle_asms(self, dio3: Bool):
        self.state["AMS"] = CarStateEnum.ON if dio3.data else CarStateEnum.OFF

    def handle_watchdog(self, dio4: Bool):
        if not dio4.data:  # what does 1 and 0 mean
            self.activate_EBS()

    def handle_front_bp(self, front_bp: Float64):
        self.front_bp = front_bp.data

    def handle_rear_bp(self, rear_bp: Float64):
        self.rear_bp = rear_bp.data

    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """

        if not self.inital_checkup_done:
            self.initial_checkup()
        else:
            self.monitor()

        t = rospy.Time.now().to_sec()

        # R2D
        if self.res_go_signal and t - self.as_ready_time > 5.0:
            self.state["R2D"] = CarStateEnum.ACTIVATED
        elif self.as_state != AutonomousStatesEnum.ASDRIVE:
            self.state["R2D"] = CarStateEnum.OFF

        # ASB and EBS to on? -> later

        # self.state["ASB"] = (
        #     (
        #         CarStateEnum.ACTIVATED
        #         if self.state["R2D"] == CarStateEnum.OFF
        #         else CarStateEnum.ON
        #     )
        #     if t - self.odrive_hb < 0.2
        #     else CarStateEnum.OFF
        # )
        # TS, ASMS and EBS were assigned before

        return self.state
