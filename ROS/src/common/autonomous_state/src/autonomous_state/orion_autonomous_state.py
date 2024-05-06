import time

import can
import rosparam
import rospy
from can_msgs.msg import Frame
from car_state import CarState, CarStateEnum
from diagnostic_msgs.msg import DiagnosticStatus
from node_fixture import AutonomousMission, AutonomousStatesEnum, serialcan_to_roscan
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty


class OrionAutonomousState(CarState):
    def __init__(self, autonomous_controller) -> None:
        self.disable_ebs_service = rospy.Service(
            "/disable_ebs", Empty, self.disable_ebs
        )
        self.autonomous_controller = autonomous_controller
        rospy.Subscriber("/ugr/car/can/rx", Frame, self.handle_can)
        rospy.Subscriber("/dio_driver_1/DI1", Bool, self.handle_sdc)  # sdc status
        rospy.Subscriber(
            "/dio_driver_1/DI3", Bool, self.handle_asms
        )  # ASMS status button
        rospy.Subscriber(
            "/dio_driver_1/DI4", Bool, self.handle_watchdog
        )  # watchdog status
        rospy.Subscriber("/dio_driver_1/DI5", Bool, self.handle_bypass)  # bypass status
        rospy.Subscriber(
            "/iologik/input1", Float64, self.handle_air_pressure1
        )  # EBS 1 air pressure
        rospy.Subscriber(
            "/iologik/input2", Float64, self.handle_air_pressure2
        )  # EBS 2 air pressure

        rospy.Subscriber(
            "/iologik/input3", Float64, self.handle_front_ebs_bp
        )  # front ebs brake pressure
        rospy.Subscriber(
            "/iologik/input4", Float64, self.handle_rear_ebs_bp
        )  # rear ebs brake pressure
        self.ebs_arm = rospy.Publisher("/dio_driver_1/DO1", Bool, queue_size=10)
        self.dbs_arm = rospy.Publisher("/dio_driver_1/DO2", Bool, queue_size=10)
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
        self.initial_checkup_busy = False  # indicates whether we are waiting 200ms for watchdog in initial checkup
        self.initial_checkup_done = False
        self.boot_done = False
        self.monitored_once = False
        self.toggling_watchdog = True
        self.res_go_signal = False
        self.res_estop_signal = False
        self.watchdog_status = True
        self.sdc_status = True
        self.monitoring = True
        self.ts_pressed = False
        self.elvis_critical_fault = None
        self.elvis_status = None
        self.air_pressure1 = None
        self.air_pressure2 = None
        self.front_ebs_bp = None
        self.rear_ebs_bp = None
        self.hbs = {
            "PDU": rospy.Time.now().to_sec(),
            "ELVIS": rospy.Time.now().to_sec(),
            "DB": rospy.Time.now().to_sec(),
            "ASSI": rospy.Time.now().to_sec(),
            "MC": rospy.Time.now().to_sec(),
            "RES": rospy.Time.now().to_sec(),
            "air_pressure1": rospy.Time.now().to_sec(),
            "air_pressure2": rospy.Time.now().to_sec(),
            "front_ebs_bp": rospy.Time.now().to_sec(),
            "rear_ebs_bp": rospy.Time.now().to_sec(),
        }
        self.as_ready_time = rospy.Time.now().to_sec()
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
        # DB_Commands
        if frame.id == 768:
            self.ts_pressed = bool(frame.data[0] & 0b01000000)
            self.state["R2D"] = (
                CarStateEnum.ON  # not yet activated, still need RES
                if bool(frame.data[0] & 0b10000000)
                and self.state["TS"] == CarStateEnum.ACTIVATED
                else CarStateEnum.OFF
            )

        # RES
        if frame.id == 0x191:
            self.res_hb = rospy.Time.now().to_sec()
            self.res_go_signal = (frame.data[0] & 0b0000100) >> 2
            self.res_estop_signal = not (frame.data[0] & 0b0000001)
            if self.res_estop_signal:
                self.activate_EBS(1)

        # LV ECU HBS
        # PDU
        if frame.id == 16:
            self.hbs["PDU"] = rospy.Time.now().to_sec()
        # ELVIS TODO use the critical fault and status of elvis
        if frame.id == 2:
            self.hbs["ELVIS"] = rospy.Time.now().to_sec()
            self.elvis_critical_fault = (frame.data[0] >> 3) & 0b00000111
            self.elvis_status = (frame.data[0] >> 6) & 0b00000111

        # DB
        if frame.id == 3:
            self.hbs["DB"] = rospy.Time.now().to_sec()
        # ASSI
        if frame.id == 4:
            self.hbs["ASSI"] = rospy.Time.now().to_sec()

        # MC CAN HB
        if frame.id == 2147492865:
            self.hbs["MC"] = rospy.Time.now().to_sec()

    def activate_EBS(self, error_message="unknown"):
        self.state["EBS"] = CarStateEnum.ACTIVATED
        self.state["TS"] = CarStateEnum.OFF
        self.state["R2D"] = CarStateEnum.OFF
        self.ebs_arm.publish(Bool(data=False))
        self.dbs_arm.publish(Bool(data=False))
        self.sdc_out.publish(Bool(data=False))
        self.initial_checkup_busy = False
        self.autonomous_controller.set_health(
            DiagnosticStatus.ERROR, "EBS activated, reason: " + error_message
        )
        self.monitoring = False

    def update(self, state: AutonomousStatesEnum):
        # On a state transition, start 5 second timeout
        if (
            state == AutonomousStatesEnum.ASREADY
            and self.as_state != AutonomousStatesEnum.ASREADY
        ):
            self.as_ready_time = rospy.Time.now().to_sec()

        self.as_state = state

        self.send_status_over_can()

    def send_heartbeat(self):
        canmsg = can.Message(
            arbitration_id=0,
            data=[0b00000001],
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def send_status_over_can(self):
        # https://www.formulastudent.de/fileadmin/user_upload/all/2022/rules/FSG22_Competition_Handbook_v1.1.pdf

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

        canmsg = can.Message(
            arbitration_id=0x502,
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

        state_to_bits = {
            CarStateEnum.UNKNOWN: 0,
            CarStateEnum.OFF: 1,
            CarStateEnum.ON: 2,  # TS_READY or R2D_READY
            CarStateEnum.ACTIVATED: 3,  # TS_ACTIVE or R2D_START
        }

        # Get the bits for the TS and R2D states
        ts_bits = state_to_bits.get(self.state["TS"], 0b00)
        r2d_bits = state_to_bits.get(self.state["R2D"], 0b00)

        # Create the data
        # Bits 0-1: TS state
        # Bits 2-3: R2D state
        data = [(ts_bits & 0b11) | ((r2d_bits & 0b11) << 2)]

        canmsg = can.Message(
            arbitration_id=0x999,  # placeholder
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    # runs at 15 hz
    def get_state(self):
        if not self.boot_done:
            self.boot()

        if (
            not self.initial_checkup_done and self.toggling_watchdog
        ):  # toggling watchdog in continuous monitoring is done in monitor()
            self.watchdog_trigger.publish(Bool(data=True))
            time.sleep(0.005)
            self.watchdog_trigger.publish(Bool(data=False))

        if (
            not self.initial_checkup_done
            and self.boot_done
            and not self.initial_checkup_busy
            and self.monitoring
        ):
            self.initial_checkup()
        elif self.boot_done and self.initial_checkup_done and self.monitoring:
            self.monitor()

        if (
            self.state["TS"] == CarStateEnum.ON
            and self.ts_pressed
            and self.initial_checkup_done
            and self.monitored_once
            and self.elvis_status == 1
        ):  # TODO change elvis status here
            self.state["TS"] = CarStateEnum.ACTIVATED

        if (
            self.state["R2D"] == CarStateEnum.ON
            and self.res_go_signal
            and self.state["TS"] == CarStateEnum.ACTIVATED
        ):
            self.state["R2D"] = CarStateEnum.ACTIVATED

        return self.state

    def handle_sdc(self, dio1: Bool):
        self.sdc_status = dio1.data

    def handle_asms(self, dio3: Bool):
        return

    #     self.state["ASMS"] = CarStateEnum.ON if dio3.data else CarStateEnum.OFF

    def handle_watchdog(self, dio4: Bool):
        self.watchdog_status = dio4.data

    def handle_bypass(self, dio5: Bool):
        self.bypass_status = dio5.data
        self.state["ASMS"] = CarStateEnum.ON if dio5.data else CarStateEnum.OFF

    def handle_air_pressure1(self, air_pressure1: Float64):
        self.air_pressure1 = air_pressure1.data
        self.hbs["air_pressure1"] = rospy.Time.now().to_sec()

    def handle_air_pressure2(self, air_pressure2: Float64):
        self.air_pressure2 = air_pressure2.data
        self.hbs["air_pressure2"] = rospy.Time.now().to_sec()

    def handle_front_ebs_bp(self, front_ebs_bp: Float64):
        self.front_ebs_bp = front_ebs_bp.data
        self.hbs["front_ebs_bp"] = rospy.Time.now().to_sec()

    def handle_rear_ebs_bp(self, rear_ebs_bp: Float64):
        self.rear_ebs_bp = rear_ebs_bp.data
        self.hbs["rear_ebs_bp"] = rospy.Time.now().to_sec()

    def handle_disable_ebs(self, req):
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=True))
        self.state["EBS"] = CarStateEnum.OFF

    def boot(self):
        # check heartbeats of low voltage systems, motorcontrollers, RES and sensors
        for hb in self.hbs:
            if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
                return

        # watchdog OK?
        if not self.watchdog_status:
            return

        self.boot_done = True
        self.sdc_out.publish(Bool(data=True))
        time.sleep(0.200)

    def initial_checkup(self):
        self.initial_checkup_busy = True
        # mission needs to be selected
        if not (rospy.has_param("/mission") and rospy.get_param("/mission") != ""):
            self.activate_EBS(2)

        # ASMS needs to be on
        if self.state["ASMS"] != CarStateEnum.ON:
            self.activate_EBS(4)

        # check air pressures
        if not (
            self.air_pressure1 > 8
            and self.air_pressure2 > 8
            and self.air_pressure1 < 9.5
            and self.air_pressure2 < 9.5
        ):
            self.activate_EBS(5)

        # watchdog OK?
        if not self.watchdog_status:
            self.activate_EBS(6)

        # is SDC closed?
        if not self.sdc_status:
            self.initial_checkup_busy = False
            return  # hopefully in the next iteration this will be True, but no need for EBS

        # stop toggling watchdog
        self.toggling_watchdog = False

        # wait 200ms
        time.sleep(0.200)

        # check whether the watchdog is indicating error
        if self.watchdog_status:
            self.activate_EBS(7)

        # check if sdc went open
        if not self.sdc_status:
            self.activate_EBS(9)

        # check ebs brake pressures
        if not (
            self.front_ebs_bp > 90
            and self.rear_ebs_bp > 90
            and self.front_ebs_bp < 150
            and self.rear_ebs_bp < 150
        ):
            self.activate_EBS(10)

        # start toggling watchdog
        self.toggling_watchdog = True

        # reset watchdog
        self.watchdog_reset.publish(Bool(data=True))
        time.sleep(0.200)
        self.watchdog_reset.publish(Bool(data=False))

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS(11)

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
            self.activate_EBS(12)

        # release ebs
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS(11)

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
            self.activate_EBS(12)

        # release dbs
        self.ebs_arm.publish(Bool(data=True))
        self.dbs_arm.publish(Bool(data=True))

        time.sleep(0.200)

        # check whether pressure is being released as expected
        if not (self.front_ebs_bp < 10 and self.rear_ebs_bp < 10):
            self.activate_EBS(11)

        # set PPR setpoint, actuate brake with DBS
        self.dbs.publish(Float64(data=25))  # 25 bar, placeholder

        # check whether pressure is being built up as expected
        if not (self.front_ebs_bp > 20 and self.rear_ebs_bp > 20):
            self.activate_EBS(12)

        self.state["ASB"] = CarStateEnum.ACTIVATED
        self.state["EBS"] = CarStateEnum.ON
        self.state["TS"] = CarStateEnum.ON

        self.initial_checkup_done = True
        self.initial_checkup_busy = False

    def monitor(self):
        # is SDC closed?
        if not self.sdc_status:
            if (
                self.front_ebs_bp > 0
                and self.rear_ebs_bp > 0
                and self.front_ebs_bp < 100
                and self.rear_ebs_bp < 100
            ):
                self.monitoring = False
            else:
                self.activate_EBS(8)

        # check heartbeats of low voltage systems, motorcontrollers, RES and sensors
        for i, hb in enumerate(self.hbs):
            if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
                self.activate_EBS(13 + i)

        # check ipc, sensors and actuators
        if (
            self.autonomous_controller.get_health_level() == DiagnosticStatus.ERROR
            or self.as_state == AutonomousStatesEnum.ASEMERGENCY
        ):
            self.activate_EBS(23)

        # check output signal of watchdog
        if not self.watchdog_status:
            self.activate_EBS(6)

        # check brake pressures
        if not (
            self.front_ebs_bp > 0
            and self.rear_ebs_bp > 0
            and self.front_ebs_bp < 100
            and self.rear_ebs_bp < 100
        ):
            self.activate_EBS(10)

        # check air pressures
        if not (
            self.air_pressure1 > 8
            and self.air_pressure2 > 8
            and self.air_pressure1 < 9.5
            and self.air_pressure2 < 9.5
        ):
            self.activate_EBS(5)

        # check if bypass relay is still functioning
        if not self.bypass_status:
            self.activate_EBS(25)

        # toggle watchdog
        self.watchdog_trigger.publish(Bool(data=True))
        time.sleep(0.005)
        self.watchdog_trigger.publish(Bool(data=False))

        self.send_heartbeat()
        self.monitored_once = True
