import time

import can
import rospy
from can_msgs.msg import Frame
from car_state import CarState, CarStateEnum
from node_fixture import serialcan_to_roscan
from std_msgs.msg import Bool, Float64


class OrionManualState(CarState):
    def __init__(self) -> None:
        rospy.Subscriber("/ugr/car/can/rx", Frame, self.handle_can)
        self.bus = rospy.Publisher("/ugr/car/can/tx", Frame, queue_size=10)
        self.state = {
            "TS": CarStateEnum.UNKNOWN,
            "R2D": CarStateEnum.UNKNOWN,
        }
        self.initial_checkup_busy = False
        self.initial_checkup_done = False
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
        self.mc_can_hb = rospy.Time.now().to_sec()
        self.lv_can_hbs = {
            "PDU": rospy.Time.now().to_sec(),
            "ELVIS": rospy.Time.now().to_sec(),
            "DB": rospy.Time.now().to_sec(),
            "ASSI": rospy.Time.now().to_sec(),
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
        )  # sdc out start low, high when everything is ok and low in case of errorµµ

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 768:
            self.ts_pressed = bool(frame.data[0] & 0b01000000)
            self.state["R2D"] = (
                CarStateEnum.ON
                if bool(frame.data[0] & 0b10000000)
                else CarStateEnum.OFF
            )

        # LV ECU HBS
        # PDU
        if frame.id == 16:
            self.lv_can_hbs["PDU"] = rospy.Time.now().to_sec()
        # ELVIS TODO maybe add critical fault and elvis status
        if frame.id == 2:
            self.lv_can_hbs["ELVIS"] = rospy.Time.now().to_sec()
            self.elvis_critical_fault = (frame.data[0] >> 3) & 0b00000111
            self.elvis_status = (frame.data[0] >> 6) & 0b00000111
        # DB
        if frame.id == 3:
            self.lv_can_hbs["DB"] = rospy.Time.now().to_sec()
        # ASSI
        if frame.id == 4:
            self.lv_can_hbs["ASSI"] = rospy.Time.now().to_sec()

        # MC CAN HB
        if frame.id == 2147492865:
            self.mc_can_hb = rospy.Time.now().to_sec()

    def initial_checkup(self):
        self.initial_checkup_busy = True

        # ASMS needs to be off
        if self.state("ASMS") != CarStateEnum.OFF:
            self.send_error_to_db()

        # check air pressures
        if not (
            self.air_pressure1 > 8
            and self.air_pressure2 > 8
            and self.air_pressure1 < 9.5
            and self.air_pressure2 < 9.5
        ):
            self.send_error_to_db()

        # watchdog OK?
        if not self.watchdog_status:
            self.send_error_to_db()

        # is SDC closed?
        if not self.sdc_status:
            self.initial_checkup_busy = False
            return  # hopefully in the next iteration this will be True, no error yet

        # stop toggling watchdog
        self.toggling_watchdog = False

        # wait 200ms
        time.sleep(0.200)

        # check whether the watchdog is indicating error
        if self.watchdog_status:
            self.send_error_to_db()

        # check if sdc went open
        if self.sdc_status:
            self.send_error_to_db()

        # close sdc, not sure if it should be done here
        self.sdc_out.publish(Bool(data=True))

        self.state["TS"] = CarStateEnum.ON
        self.initial_checkup_busy = False
        self.initial_checkup_done = True

    def monitor(self):
        return

    def send_heartbeat(self):
        canmsg = can.Message(
            arbitration_id=0,
            data=[0b00000001],
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def send_status_over_can(self):
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
        self.state["ASMS"] = CarStateEnum.ON if dio3.data else CarStateEnum.OFF

    def handle_watchdog(self, dio4: Bool):
        self.watchdog_status = dio4.data

    def handle_bypass(self, dio5: Bool):
        self.bypass_status = dio5.data

    def handle_air_pressure1(self, air_pressure1: Float64):
        self.air_pressure1 = air_pressure1.data

    def handle_air_pressure2(self, air_pressure2: Float64):
        self.air_pressure2 = air_pressure2.data

    def handle_front_ebs_bp(self, front_ebs_bp: Float64):
        self.front_ebs_bp = front_ebs_bp.data

    def handle_rear_ebs_bp(self, rear_ebs_bp: Float64):
        self.rear_ebs_bp = rear_ebs_bp.data

    def send_error_to_db(self, error):
        self.initial_checkup_busy = False

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
            and self.elvis_status == 1
        ):  # TODO change elvis status
            self.state["TS"] = CarStateEnum.ACTIVATED
        self.send_status_over_can()  # not sure whether this needs to be sent all the time
        return self.state
