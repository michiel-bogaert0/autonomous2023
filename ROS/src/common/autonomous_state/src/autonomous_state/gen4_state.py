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
            "/iologik/input1", Float64, self.handle_air_pressure1
        )  # EBS 1 air pressure
        rospy.Subscriber(
            "/iologik/input2", Float64, self.handle_air_pressure2
        )  # EBS 2 air pressure

        # probably remove these two
        rospy.Subscriber(
            "/iologik/input3", Float64, self.handle_front_ebs_bp
        )  # front ebs brake pressure
        rospy.Subscriber(
            "/iologik/input4", Float64, self.handle_rear_ebs_bp
        )  # rear brake pressure
        self.ebs_arm = rospy.Publisher("/dio_driver_1/DO1", Bool, queue_size=10)  # ebs1
        self.dbs_arm = rospy.Publisher("/dio_driver_1/DO2", Bool, queue_size=10)  # ebs2
        self.dbs = rospy.Publisher("/iologik/output1", Float64, queue_size=10)
        self.watchdog_trigger = rospy.Publisher(
            "/dio_driver_1/DO3", Bool, queue_size=10
        )  # watchdog trigger
        self.watchdog_reset = rospy.Publisher(
            "/dio_driver_1/DO4", Bool, queue_size=10
        )  # watchdog reset
        self.sdc_out = rospy.Publisher(
            "/dio_driver_1/DO4", Bool, queue_size=10
        )  # sdc out start low, high when everything is ok and low in case of error
        self.initial_checkup_busy = (
            False  # indicates whether we are waiting 200ms for watchdog
        )
        self.initial_checkup_done = False
        self.toggling_watchdog = True
        self.res_go_signal = False
        self.res_estop_signal = False
        self.watchdog_status = True  # not sure
        self.sdc_status = True  # not sure
        self.air_pressure1 = None
        self.air_pressure2 = None
        self.front_ebs_bp = None
        self.rear_ebs_bp = None
        self.as_ready_time = rospy.Time.now().to_sec()
        # DBS ACTIVATED VANAF WNR JE DIE EERSTE TEST HEBT GDN, ANDERS GWN ON
        self.state = {
            "TS": CarStateEnum.UNKNOWN,
            "ASMS": CarStateEnum.UNKNOWN,
            "R2D": CarStateEnum.UNKNOWN,
            "ASB": CarStateEnum.ON,  # assumed to be on
            "EBS": CarStateEnum.UNKNOWN,
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
        self.ebs_arm.publish(Bool(data=False))
        self.dbs_arm.publish(Bool(data=False))
        self.sdc_out.publish(Bool(data=False))

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

    def initial_checkup(self):
        # ??? to do: cm1 and cm2 filled, sj1 and sj2 closed
        self.initial_checkup_busy = True
        # mission needs to be selected
        if not (rospy.has_param("/mission") and rospy.get_param("/mission") != ""):
            self.activate_EBS()

        # ASMS needs to be activated
        if self.state["ASMS"] != CarStateEnum.ACTIVATED:
            self.activate_EBS()

        # check air pressures
        if not (
            self.air_pressure1 > 8
            and self.air_pressure2 > 8
            and self.air_pressure1 < 9.5
            and self.air_pressure2 < 9.5
        ):
            self.activate_EBS()

        # watchdog OK?
        if not self.watchdog_status:
            self.activate_EBS()

        # is SDC closed?
        if not self.sdc_status:
            self.initial_checkup_busy = False
            return  # hopefully in the next iteration this will be True, but no need for EBS

        # stop toggling watchdog
        self.toggling_watchdog = False

        # wait 200 ms
        time.sleep(0.200)

        # check whether the watchdog is indicating error
        if self.watchdog_status:
            self.activate_EBS()

        # check if sdc went open
        if self.sdc_status:
            self.activate_EBS()

        # check ebs brake pressures
        if not (
            self.front_ebs_bp > 90
            and self.rear_ebs_bp > 90
            and self.front_ebs_bp < 150
            and self.rear_ebs_bp < 150
        ):
            self.activate_EBS()

        # start toggling watchdog
        self.toggling_watchdog = True

        # reset watchdog
        self.watchdog_reset.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS()

        # trigger ebs
        self.ebs_arm.publish(Bool(data=False))
        self.dbs_arm.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being built up as expected
        if not (
            self.front_ebs_bp > 40
            and self.rear_ebs_bp > 40
            and self.front_ebs_bp < 80
            and self.rear_ebs_bp < 80
        ):
            self.activate_EBS()

        # release ebs
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS()

        # trigger dbs
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=False))

        time.sleep(0.200)

        # check whether pressure is being built up as expected
        if not (
            self.front_ebs_bp > 40
            and self.rear_ebs_bp > 40
            and self.front_ebs_bp < 80
            and self.rear_ebs_bp < 80
        ):
            self.activate_EBS()

        # release dbs
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS()

        # set PPR setpoint, actuate brake with DBS
        self.dbs.publish(Float64(data=30))  # 30 bar, placeholder

        # check whether pressure is being built up as expected
        if not (self.front_ebs_bp > 20 and self.rear_ebs_bp > 20):
            self.activate_EBS()

        # if TS is active, we are done, otherwise we have to wait
        if self.state["TS"] == CarStateEnum.ACTIVATED:
            self.initial_checkup_done = True

        self.initial_checkup_busy = False

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
        if not dio1.data:  # 1 means ok
            self.activate_EBS()
        self.sdc_

    def handle_ts(self, dio2: Bool):
        self.state["TS"] = CarStateEnum.ACTIVATED if dio2.data else CarStateEnum.OFF

    def handle_asms(self, dio3: Bool):
        self.state["AMS"] = CarStateEnum.ACTIVATED if dio3.data else CarStateEnum.OFF

    def handle_watchdog(self, dio4: Bool):
        if not dio4.data:  # 1 means ok
            self.watchdog_status = dio4.data
            self.activate_EBS()  # not 100% sure

    def handle_air_pressure1(self, air_pressure1: Float64):
        self.air_pressure1 = air_pressure1.data

    def handle_air_pressure2(self, air_pressure2: Float64):
        self.air_pressure2 = air_pressure2.data

    # these will probably have to be done with can
    def handle_front_ebs_bp(self, front_ebs_bp: Float64):
        self.front_ebs_bp = front_ebs_bp.data

    def handle_rear_ebs_bp(self, rear_ebs_bp: Float64):
        self.rear_ebs_bp = rear_ebs_bp.data

    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """
        if self.toggling_watchdog:
            self.watchdog_trigger.publish(Bool(data=True))
            time.sleep(0.005)
            self.watchdog_trigger.publish(Bool(data=False))

        if (
            not self.initial_checkup_done and not self.initial_checkup_busy
        ):  # if we are busy waiting 200ms, just continue
            self.initial_checkup()
        elif self.initial_checkup_done:
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
