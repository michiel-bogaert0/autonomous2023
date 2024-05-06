import time

import can
import rospy
from can_msgs.msg import Frame
from car_state import CarState, CarStateEnum
from diagnostic_msgs.msg import DiagnosticStatus
from node_fixture import serialcan_to_roscan
from std_msgs.msg import Bool, Float64


class OrionManualState(CarState):
    def __init__(self, manual_controller) -> None:
        rospy.Subscriber("/ugr/car/can/rx", Frame, self.handle_can)
        self.manual_controller = manual_controller
        self.bus = rospy.Publisher("/ugr/car/can/tx", Frame, queue_size=10)
        self.state = {
            "TS": CarStateEnum.UNKNOWN,
            "R2D": CarStateEnum.UNKNOWN,
        }
        self.initial_checkup_busy = False
        self.initial_checkup_done = False
        self.monitored_once = False
        self.toggling_watchdog = True
        self.res_go_signal = False
        self.res_estop_signal = False
        self.watchdog_status = True
        self.sdc_status = True
        self.ts_pressed = False
        self.monitoring = True
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
            "air_pressure1": rospy.Time.now().to_sec(),
            "air_pressure2": rospy.Time.now().to_sec(),
            "front_ebs_bp": rospy.Time.now().to_sec(),
            "rear_ebs_bp": rospy.Time.now().to_sec(),
        }
        rospy.Subscriber("/dio_driver_1/DI1", Bool, self.handle_sdc)  # sdc status
        rospy.Subscriber(
            "/dio_driver_1/DI4", Bool, self.handle_watchdog
        )  # watchdog status
        rospy.Subscriber("/dio_driver_1/DI5", Bool, self.handle_bypass)  # bypass status
        rospy.Subscriber(
            "/dio_driver_1/DI3", Bool, self.handle_asms
        )  # ASMS status button
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

        self.watchdog_trigger = rospy.Publisher(
            "/dio_driver_1/DO3", Bool, queue_size=10
        )  # watchdog trigger
        self.watchdog_reset = rospy.Publisher(
            "/dio_driver_1/DO4", Bool, queue_size=10
        )  # watchdog reset
        self.sdc_out = rospy.Publisher(
            "/dio_driver_1/DO4", Bool, queue_size=10
        )  # sdc out start low, high when everything is ok and low in case of error

        time.sleep(
            1
        )  # wait for everything to start up (also give sensors time to send data)

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 768:
            self.ts_pressed = bool(frame.data[0] & 0b01000000)
            self.state["R2D"] = (
                CarStateEnum.ACTIVATED
                if bool(frame.data[0] & 0b10000000)
                and self.state["R2D"] == CarStateEnum.ON
                else CarStateEnum.OFF
            )

        # LV ECU HBS
        # PDU
        if frame.id == 16:
            self.hbs["PDU"] = rospy.Time.now().to_sec()
        # ELVIS TODO maybe add critical fault and elvis status
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

    def send_heartbeat(self):
        canmsg = can.Message(
            arbitration_id=0,
            data=[0b00000001],
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def send_status_over_can(self):
        data = [0, 0, 0, 0, 0]

        # Bit 0 - 2
        bits = 1  # ASOFF
        data[0] |= bits

        # Bit 3 - 4
        bits = 1  # EBS OFF
        data[0] |= bits << 3

        # Bits 5 - 7
        bits = 7  # MANUAL
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

    def handle_sdc(self, dio1: Bool):
        self.sdc_status = dio1.data

    def handle_asms(self, dio3: Bool):
        return
        # self.state["ASMS"] = CarStateEnum.ON if dio3.data else CarStateEnum.OFF

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
        if (
            self.state["TS"] == CarStateEnum.ACTIVATED
            and self.front_ebs_bp > 8
            and self.rear_ebs_bp > 8
        ):
            self.state["R2D"] = CarStateEnum.ON

    def handle_rear_ebs_bp(self, rear_ebs_bp: Float64):
        self.rear_ebs_bp = rear_ebs_bp.data
        self.hbs["rear_ebs_bp"] = rospy.Time.now().to_sec()
        if (
            self.state["TS"] == CarStateEnum.ACTIVATED
            and self.front_ebs_bp > 8
            and self.rear_ebs_bp > 8
        ):
            self.state["R2D"] = CarStateEnum.ON

    def send_error_to_db(self, error_message="unknown"):
        if self.state["TS"] == CarStateEnum.ACTIVATED:
            self.state["TS"] = CarStateEnum.ON
        self.initial_checkup_done = False
        self.elvis_status = 0  # reset TODO
        self.ts_pressed = False
        self.monitored_once = False
        self.initial_checkup_busy = False
        self.autonomous_controller.set_health(
            DiagnosticStatus.ERROR, "Error detected, reason: " + error_message
        )

    def get_state(self):
        if (
            not self.initial_checkup_done and self.toggling_watchdog
        ):  # toggling watchdog in continuous monitoring is done in monitor()
            self.watchdog_trigger.publish(Bool(data=True))
            time.sleep(0.005)
            self.watchdog_trigger.publish(Bool(data=False))

        if not self.initial_checkup_done and not self.initial_checkup_busy:
            self.initial_checkup()
        elif self.initial_checkup_done and self.monitoring:
            self.monitor()

        if (
            self.state["TS"] == CarStateEnum.ON
            and self.ts_pressed
            and self.initial_checkup_done
            and self.monitored_once
            and self.elvis_status == 1
        ):  # TODO change elvis status
            self.state["TS"] = CarStateEnum.ACTIVATED
        self.send_status_over_can()  # not sure whether this needs to be sent all the time
        return self.state

    def initial_checkup(self):
        self.initial_checkup_busy = True

        # ASMS needs to be off
        if self.state("ASMS") != CarStateEnum.OFF:
            self.send_error_to_db("ASMS not off")

        # check air pressures
        if not (self.air_pressure1 < 1 and self.air_pressure2 < 1):
            self.send_error_to_db("Air pressures not in range")

        # watchdog OK?
        if not self.watchdog_status:
            self.send_error_to_db("Watchdog not OK")

        # is SDC closed?
        if not self.sdc_status:
            self.initial_checkup_busy = False
            return  # hopefully in the next iteration this will be True, no error yet

        self.state["TS"] = CarStateEnum.ON
        self.initial_checkup_busy = False
        self.initial_checkup_done = True

    def monitor(self):
        # is SDC closed?
        if not self.sdc_status:
            self.send_error_to_db("SDC open")

        # check heartbeats of low voltage systems, motorcontrollers and sensors
        for hb in self.hbs:
            if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
                self.send_error_to_db("Heartbeat missing, system: " + hb)

        # check ipc, sensors and actuators
        if self.manual_controller.get_health_level() == DiagnosticStatus.ERROR:
            self.send_error_to_db("IPC, sensors or actuators not OK")

        # check output signal of watchdog
        if not self.watchdog_status:
            self.send_error_to_db("Watchdog not OK")

        # check air pressures
        if not (self.air_pressure1 < 1 and self.air_pressure2 < 1):
            self.send_error_to_db("Air pressures not in range")

        # check if bypass is closed
        if self.bypass_status:
            self.send_error_to_db("Bypass closed")

        # toggle watchdog
        self.watchdog_trigger.publish(Bool(data=True))
        time.sleep(0.005)
        self.watchdog_trigger.publish(Bool(data=False))

        self.send_heartbeat()
        self.monitored_once = True
