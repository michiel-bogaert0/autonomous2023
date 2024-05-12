#! /usr/bin/python3
from time import sleep

import can
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture import (
    StateMachineScopeEnum,
    create_diagnostic_message,
    serialcan_to_roscan,
)
from node_fixture.fixture import (
    AutonomousStatesEnum,
    NodeManagingStatesEnum,
    OrionStateEnum,
)
from node_fixture.node_manager import NodeManager
from std_msgs.msg import Bool, Float64, Header
from ugr_msgs.msg import Frame, State


class OrionState(NodeManager):
    """
    This the main controller for the car and the first node to "activate"
    It handles the main state machine of the car
    """

    def __init__(self) -> None:
        super().__init__("car_state", NodeManagingStatesEnum.ACTIVE)
        rospy.Subscriber("/ugr/car/can/rx", Frame, self.handle_can)
        self.bus = rospy.Publisher("/ugr/car/can/tx", Frame, queue_size=10)
        self.initial_checkup_busy = False
        self.initial_checkup_done = False
        self.driverless_mode = "idle"
        self.boot_done = False
        self.res_go_signal = False
        self.res_estop_signal = False
        self.watchdog_status = True
        self.sdc_status = True
        self.ts_pressed = False
        self.monitoring = True
        self.elvis_status = None
        self.air_pressure1 = None
        self.air_pressure2 = None
        self.front_bp = None
        self.rear_bp = None
        self.driverless_mode = None
        self.hbs = {
            "PDU": rospy.Time.now().to_sec(),
            "ELVIS": rospy.Time.now().to_sec(),
            "DB": rospy.Time.now().to_sec(),
            "ASSI": rospy.Time.now().to_sec(),
            "MC": rospy.Time.now().to_sec(),
            "air_pressure1": rospy.Time.now().to_sec(),  # probably useless
            "air_pressure2": rospy.Time.now().to_sec(),
            "front_bp": rospy.Time.now().to_sec(),
            "rear_bp": rospy.Time.now().to_sec(),
        }
        self.state_to_bits = {
            OrionStateEnum.INIT: 0,
            OrionStateEnum.TS_READY: 1,
            OrionStateEnum.TS_ACTIVATING: 2,
            OrionStateEnum.TS_ACTIVE: 3,
            OrionStateEnum.R2D_READY: 4,
            OrionStateEnum.R2D: 5,
            OrionStateEnum.ERROR: 6,
        }
        rospy.Subscriber("/dio_driver_1/DI1", Bool, self.handle_sdc)  # sdc status
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
            "/iologik/input3", Float64, self.handle_front_bp
        )  # front ebs brake pressure
        rospy.Subscriber(
            "/iologik/input4", Float64, self.handle_rear_bp
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

        sleep(
            1
        )  # wait for everything to start up (also give sensors time to send data)
        self.spin()

    def doActivate(self):
        self.change_state(OrionStateEnum.INIT)
        self.change_as_state(AutonomousStatesEnum.ASOFF)
        self.state_publisher = rospy.Publisher(
            "/state", State, queue_size=10, latch=True
        )
        self.orion_state_publisher = rospy.Publisher(
            "/state/car", State, queue_size=10, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.as_state_publisher = rospy.Publisher(
            "/state/as", State, queue_size=10, latch=True
        )

        self.car_name = rospy.get_param("car", "orion")

        self.driverless_mode = None

        while not self.configure_nodes():
            sleep(0.1)
            if rospy.is_shutdown():
                return

        self.change_state(self.state)

    def handle_bypass(self, msg: Bool):
        if msg.data != self.bypass_status:
            self.bypass_status = msg.data
            if self.bypass_status:
                self.driverless_mode = "driverless"
                self.activate_nodes("driverless", self.driverless_mode)
            else:
                self.driverless_mode = "manual"
                self.activate_nodes("manual", self.driverless_mode)

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 768:
            self.ts_pressed = bool(frame.data[0] & 0b01000000)
            self.r2d_pressed = bool(frame.data[0] & 0b10000000)

        # LV ECU HBS
        # PDU
        if frame.id == 16:
            self.hbs["PDU"] = rospy.Time.now().to_sec()

        # ELVIS
        if frame.id == 2:
            self.air1 = bool(frame.data[0] & 0b00100000)
            self.air2 = bool(frame.data[0] & 0b00010000)

            if bool(frame.data[0] & 0b00000001):
                self.hbs["ELVIS"] = rospy.Time.now().to_sec()
            critical_fault = (frame.data[0] & 0b00001100) >> 2
            if critical_fault != 0:
                self.send_error_to_db(25 + critical_fault)

        # DB
        if frame.id == 3:
            self.hbs["DB"] = rospy.Time.now().to_sec()
        # ASSI
        if frame.id == 4:
            self.hbs["ASSI"] = rospy.Time.now().to_sec()

        # MC CAN HB
        if frame.id == 2147492865:
            self.hbs["MC"] = rospy.Time.now().to_sec()

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

        state_bits = self.state_to_bits.get(self.state, 0b0000)

        # Bit 0: Heartbeat
        # Bits 1-4: State
        data = [(0b1) | ((state_bits & 0b1111) << 1)]

        canmsg = can.Message(
            arbitration_id=1,
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def change_state(self, new_state: OrionStateEnum):
        """
        Actually changes state of this machine and publishes change

        Args:
            new_state: state to switch to.
        """

        if new_state == self.state:
            return

        self.state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,  # maybe change this to manual
                prev_state=self.state,
                cur_state=new_state,
            )
        )
        self.orion_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.state,
                cur_state=new_state,
            )
        )
        self.state = new_state

    def change_as_state(self, new_state: AutonomousStatesEnum):
        """
        Actually changes state of this machine and publishes change

        Args:
            new_state: state to switch to.
        """

        if new_state == self.as_state:
            return

        self.as_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.as_state,
                cur_state=new_state,
            )
        )
        self.orion_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.as_state,
                cur_state=new_state,
            )
        )
        self.as_state = new_state

    def active(self):
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: Orion state", str(self.state)
            )
        )
        if self.driverless_mode is None:
            return

        if self.state == OrionStateEnum.INIT:
            if self.boot():
                self.change_state(OrionStateEnum.TS_READY)
        elif self.state == OrionStateEnum.TS_READY:
            if not self.initial_checkup_done:
                if not self.initial_checkup_busy:
                    self.initial_checkup()
            elif self.monitor() and self.ts_pressed:
                self.ts_pressed = False
                self.change_state(OrionStateEnum.TS_ACTIVATING)

        elif self.state == OrionStateEnum.TS_ACTIVATING:
            if self.air1 and self.air2:
                self.change_state(OrionStateEnum.TS_ACTIVE)
            self.monitor()

        elif self.state == OrionStateEnum.TS_ACTIVE:
            if self.front_bp > 11 and self.rear_bp > 11:
                self.change_state(OrionStateEnum.R2D_READY)
            self.monitor()

        elif self.state == OrionStateEnum.R2D_READY:
            if self.front_bp < 10 or self.rear_bp < 10:
                self.change_state(OrionStateEnum.TS_ACTIVE)
            elif self.r2d_pressed:
                self.r2d_pressed = False
                self.change_state(OrionStateEnum.R2D)
            self.monitor()

        elif self.state == OrionStateEnum.R2D:
            self.monitor()

        self.send_status_over_can()

        if self.driverless_mode:
            self.update_as()

    def update_as(self):
        # self.diagnostics_publisher.publish(
        #     create_diagnostic_message(
        #         DiagnosticStatus.ERROR
        #         if self.as_state == AutonomousStatesEnum.ASEMERGENCY
        #         else DiagnosticStatus.OK,
        #         "[GNRL] STATE: AS state",
        #         str(self.as_state),
        #     )
        # )
        # self.diagnostics_publisher.publish(
        #     create_diagnostic_message(
        #         DiagnosticStatus.OK, "[GNRL] STATE: Car state", str(self.ccs)
        #     )
        # )

        # if self.get_health_level() == DiagnosticStatus.ERROR:
        #     self.car.activate_EBS(0)

        # if self.ccs["EBS"] == CarStateEnum.ACTIVATED:
        #     if self.mission_finished and self.vehicle_stopped:
        #         self.change_state(AutonomousStatesEnum.ASFINISHED)

        #     # ! This line is here to prevent rapid toggles between ASFINISHED and ASEMERGENCY as a result of self.vehicle_stopped rapidly switching
        #     # ! In a normal FS car this isn't a problem because you have to apply both EBS and the brakes in order to get the vehicle to a "standstill" state
        #     # ! But for pegasus (and currently simulation also) we can't really "apply the brakes"
        #     elif self.as_state != AutonomousStatesEnum.ASFINISHED:
        #         self.change_state(AutonomousStatesEnum.ASEMERGENCY)

        # elif self.ccs["EBS"] == CarStateEnum.ON:
        #     if self.mission_finished and self.vehicle_stopped:
        #         if self.car_name == "pegasus" or self.car_name == "simulation":
        #             self.car.activate_EBS()
        #         else:
        #             self.car.activate_EBS(0)

        #     if (
        #         rospy.has_param("/mission")
        #         and rospy.get_param("/mission") != ""
        #         and self.ccs["ASMS"] == CarStateEnum.ON
        #         and (
        #             self.ccs["ASB"] == CarStateEnum.ON
        #             or self.ccs["ASB"] == CarStateEnum.ACTIVATED
        #         )
        #         and self.ccs["TS"] == CarStateEnum.ON
        #     ):
        #         if self.ccs["R2D"] == CarStateEnum.ACTIVATED:
        #             self.change_state(AutonomousStatesEnum.ASDRIVE)

        #         else:
        #             if (
        #                 self.ccs["ASB"] == CarStateEnum.ACTIVATED
        #                 and self.get_health_level() == DiagnosticStatus.OK
        #             ):
        #                 self.change_state(AutonomousStatesEnum.ASREADY)
        #             else:
        #                 self.change_state(AutonomousStatesEnum.ASOFF)
        #     else:
        #         self.change_state(AutonomousStatesEnum.ASOFF)

        self.car.update(self.as_state)

    def handle_sdc(self, dio1: Bool):
        self.sdc_status = dio1.data

    def handle_watchdog(self, dio4: Bool):
        self.watchdog_status = dio4.data

    def handle_air_pressure1(self, air_pressure1: Float64):
        self.air_pressure1 = (air_pressure1.data - 4) * 10 / 16
        self.hbs["air_pressure1"] = rospy.Time.now().to_sec()

    def handle_air_pressure2(self, air_pressure2: Float64):
        self.air_pressure2 = (air_pressure2.data - 4) * 10 / 16
        self.hbs["air_pressure2"] = rospy.Time.now().to_sec()

    def handle_front_bp(self, front_bp: Float64):
        self.front_bp = (front_bp.data - 4) * 250 / 16
        self.hbs["front_bp"] = rospy.Time.now().to_sec()

    def handle_rear_bp(self, rear_bp: Float64):
        self.rear_bp = (rear_bp.data - 4) * 250 / 16
        self.hbs["rear_bp"] = rospy.Time.now().to_sec()

    def send_error_to_db(self, error_code=0):
        self.change_state(OrionStateEnum.ERROR)
        self.initial_checkup_done = False
        self.sdc_out.publish(Bool(data=False))
        self.elvis_status = 0  # reset TODO
        self.ts_pressed = False
        self.r2d_pressed = False
        self.initial_checkup_busy = False
        self.set_health(DiagnosticStatus.ERROR, "Error detected, code: " + error_code)
        can_msg = can.Message(
            arbitration_id=0x505, data=[error_code], is_extended_id=False
        )
        self.bus.publish(serialcan_to_roscan(can_msg))

    def boot(self):
        # check heartbeats of low voltage systems, motorcontrollers and sensors
        for i, hb in enumerate(self.hbs):
            if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
                self.send_error_to_db(13 + i)
                return False

        # watchdog OK?
        if not self.watchdog_status:
            self.send_error_to_db(6)
            return False

        # check ipc, sensors and actuators
        if self.get_health_level() == DiagnosticStatus.ERROR:
            self.send_error_to_db(23)
            return False

        self.boot_done = True
        self.sdc_out.publish(Bool(data=True))

        return True

    def initial_checkup(self):
        self.initial_checkup_busy = True

        # ASMS needs to be off
        if not self.bypass_status:
            self.send_error_to_db(3)
            return False

        # check air pressures
        if not (self.air_pressure1 < 1 and self.air_pressure2 < 1):
            self.send_error_to_db(5)
            return False

        # watchdog OK?
        if not self.watchdog_status:
            self.send_error_to_db(6)
            return False

        self.initial_checkup_busy = False
        self.initial_checkup_done = True
        self.sdc_out.publish(Bool(data=True))

        return True

    def monitor(self):
        # is SDC closed?
        if not self.sdc_status:
            self.send_error_to_db(8)
            return False

        # check heartbeats of low voltage systems, motorcontrollers and sensors
        for i, hb in enumerate(self.hbs):
            if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
                self.send_error_to_db(13 + i)
                return False

        # check ipc, sensors and actuators
        if self.get_health_level() == DiagnosticStatus.ERROR:
            self.send_error_to_db(23)
            return False

        # check output signal of watchdog
        if not self.watchdog_status:
            self.send_error_to_db(6)
            return False

        # check air pressures
        if not (self.air_pressure1 < 1 and self.air_pressure2 < 1):
            self.send_error_to_db(5)
            return False

        # check if bypass is closed
        if self.bypass_status:
            self.send_error_to_db(24)
            return False

        # toggle watchdog
        self.watchdog_trigger.publish(Bool(data=True))
        sleep(0.005)
        self.watchdog_trigger.publish(Bool(data=False))

        return True


node = OrionState()
