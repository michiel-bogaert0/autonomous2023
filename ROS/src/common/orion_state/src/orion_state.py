#! /usr/bin/python3
from functools import partial
from time import sleep

import can
import cantools
import rospy
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture import (
    StateMachineScopeEnum,
    create_diagnostic_message,
    serialcan_to_roscan,
)
from node_fixture.fixture import (
    AutonomousStatesEnum,
    DrivingModeStatesEnum,
    NodeManagingStatesEnum,
    OrionStateEnum,
)
from node_fixture.node_manager import NodeManager
from scheduling import JobScheduler
from std_msgs.msg import Bool, Float64, Header
from ugr_msgs.msg import State


class OrionState(NodeManager):
    """
    This the main controller for the car and the first node to "activate"
    It handles the main state machine of the car
    """

    def __init__(self) -> None:
        super().__init__("orion_state", NodeManagingStatesEnum.ACTIVE)

        # Job scheduler (synchronous!)
        self.job_scheduler = JobScheduler()

        # Subscribers
        # DI signals
        for di_signal in self.di_signals:
            self.AddSubscriber(
                f"/dio/in/{di_signal}", Bool, partial(self.handle_IO, di_signal)
            )

        # DO feedback
        for do_feedback in self.do_feedback:
            self.AddSubscriber(
                f"/dio/feedback/{do_feedback}",
                Bool,
                partial(self.handle_IO, do_feedback),
            )

        # AI signals
        for ai_signal in self.ai_signals:
            self.AddSubscriber(
                f"/aio/in/{ai_signal}", Float64, partial(self.handle_IO, ai_signal)
            )

        # Publishers
        # DO Signals
        self.do_publishers = {
            "ts_btn": None,
            "wd_reset": None,
            "arm_ebs": None,
            "arm_dbs": None,
            "wd_trigger": None,
            "sdc_close": None,
        }
        for do_signal in self.do_publishers:
            self.do_publishers[do_signal] = self.AddPublisher(
                f"/dio/out/{do_signal}", Bool, queue_size=10
            )

        # CAN TX
        self.bus = self.AddPublisher("/ugr/can/lv/tx", Frame, queue_size=10)

        # States and other
        self.state_publisher = self.AddPublisher(
            "/state", State, queue_size=10, latch=True
        )
        self.orion_state_publisher = self.AddPublisher(
            "/state/car", State, queue_size=10, latch=True
        )
        self.diagnostics_publisher = self.AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.as_state_publisher = self.AddPublisher(
            "/state/as", State, queue_size=10, latch=True
        )

        self.wd_trigger_status = False
        self.wd_trigger_enable = True
        self.as_ready_transitioned = False

        self.spin()

    def doConfigure(self):
        # State machine states
        self.car_state = OrionStateEnum.INIT
        self.as_state = AutonomousStatesEnum.ASOFF
        self.driving_mode = DrivingModeStatesEnum.MANUAL

        # IO signals
        self.di_signals = {
            "ts_btn_ok": False,
            "ecu_ok": False,
            "asms_status": False,
            "dv_btn": False,
            "wd_assert": False,
            "bypass_status": False,
            "imd_ok": False,
            "sdc_out": False,
            "wd_ok": False,
        }

        # DO feedback
        self.do_feedback = {
            "ts_btn": False,
            "wd_reset": False,
            "arm_ebs": False,
            "arm_dbs": False,
            "wd_trigger": False,
            "sdc_close": False,
        }

        # CAN inputs
        self.can_inputs = {
            "air1": False,
            "air2": False,
        }

        # Latched actions from CAN
        self.can_actions = {
            "ts_pressed": False,
            "r2d_pressed": False,
        }

        self.ai_signals = {
            "air_pressure1": 0,
            "air_pressure2": 0,
            "front_bp": 0,
            "rear_bp": 0,
        }

        self.hbs = {
            "PDU": rospy.Time.now().to_sec(),
            "ELVIS": rospy.Time.now().to_sec(),
            "DB": rospy.Time.now().to_sec(),
            "ASSI": rospy.Time.now().to_sec(),
            "MC": rospy.Time.now().to_sec(),
        }
        self.state_to_bits = {
            OrionStateEnum.INIT: 0,
            OrionStateEnum.TS_READY: 1,
            OrionStateEnum.TS_ACTIVATING: 2,
            OrionStateEnum.TS_ACTIVE: 3,
            OrionStateEnum.R2D_READY: 4,
            OrionStateEnum.R2D: 5,
            OrionStateEnum.ERROR: 6,
            OrionStateEnum.SDC_OPEN: 6,
        }

        # load dbc
        dbc_filename = rospy.get_param("~db_adress", "lv.dbc")
        db_address = __file__.split("/")[:-1]
        db_address += ["..", "dbc", dbc_filename]
        self.db_adress = "/".join(db_address)
        self.db = cantools.database.load_file(self.db_adress)

        # Boot and configure all nodes
        while not self.configure_nodes():
            sleep(0.1)
            if rospy.is_shutdown():
                return

    def doActivate(self):
        # Start default in INIT and ASOFF
        self.change_state(OrionStateEnum.INIT)
        self.change_as_state(AutonomousStatesEnum.ASOFF)

        # Initial checkup
        checks_ok, msg = self.initial_checkup()
        if not checks_ok:
            self.change_state(OrionStateEnum.ERROR)
            self.set_health(
                DiagnosticStatus.ERROR,
                f"Initial checkup failed in mode '{self.driving_mode}'. Got error '{msg}'",
            )

        # Activate nodes for driving mode
        self.activate_nodes(self.driving_mode, None)

    def change_state(self, new_state: OrionStateEnum):
        """
        Actually changes state of this machine and publishes change

        Args:
            new_state: state to switch to.
        """

        if new_state == self.car_state:
            return

        # Remove timeouts
        self.job_scheduler.remove_job_by_tag("timeout_state")

        self.state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.CAR,
                prev_state=self.car_state,
                cur_state=new_state,
            )
        )
        self.orion_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.CAR,
                prev_state=self.car_state,
                cur_state=new_state,
            )
        )
        print(f"New CAR state: {new_state}")
        self.car_state = new_state

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
        print(f"New AS state: {new_state}")
        self.as_state = new_state

    #
    # SUBSCRIPTION HANDLERS
    #

    def handle_IO(self, io_name, msg: Bool):
        """
        Handles the incoming IO signals
        """

        if io_name in self.di_signals:
            self.di_signals[io_name] = msg.data
        elif io_name in self.ai_signals:
            self.ai_signals[io_name] = msg.data
        elif io_name in self.do_feedback:
            self.do_feedback[io_name] = msg.data
        else:
            rospy.logwarn(f"IO signal '{io_name}' not found in list of IO signals")

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 768:
            self.can_actions["ts_pressed"] = bool(frame.data[0] & 0b01000000)
            self.can_actions["r2d_pressed"] = bool(frame.data[0] & 0b10000000)

        # RES TODO

        # # LV ECU HBS
        # # PDU
        # if frame.id == 16:
        #     self.hbs["PDU"] = rospy.Time.now().to_sec()

        # # ELVIS
        # if frame.id == 2:
        #     self.air1 = bool(frame.data[0] & 0b00100000)
        #     self.air2 = bool(frame.data[0] & 0b00010000)

        #     if bool(frame.data[0] & 0b00000001):
        #         self.hbs["ELVIS"] = rospy.Time.now().to_sec()
        #     critical_fault = (frame.data[0] & 0b00001100) >> 2
        #     if critical_fault != 0:
        #         self.send_error_to_db(25 + critical_fault)

        # # DB
        # if frame.id == 3:
        #     self.hbs["DB"] = rospy.Time.now().to_sec()
        # # ASSI
        # if frame.id == 4:
        #     self.hbs["ASSI"] = rospy.Time.now().to_sec()

        # # MC CAN HB
        # if frame.id == 2147492865:
        #     self.hbs["MC"] = rospy.Time.now().to_sec()

    def send_status_over_can(self):
        #
        # Brake light (temporary)
        #

        brake_pressures_msg = self.db.get_message_by_name("Brake_pressures")
        data = brake_pressures_msg.encode(
            {"Brake_rear": max(0, self.rear_bp), "Brake_front": max(0, self.front_bp)}
        )
        message = can.Message(arbitration_id=brake_pressures_msg.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))

        #
        # 0x502 acc. to rules
        #

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

        #
        # Heartbeat & CAR state
        #

        state_bits = self.state_to_bits.get(self.car_state, 0b0000)

        # Bit 0: Heartbeat
        # Bits 1-4: State
        data = [(0b1) | ((state_bits & 0b1111) << 1)]

        canmsg = can.Message(
            arbitration_id=1,
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

    def active(self):
        # Update state machines
        self.update_car_state()
        self.update_as_state()

        # Check if driving mode should be updated
        if (
            self.driving_mode == DrivingModeStatesEnum.MANUAL
            and self.di_signals["bypass_status"]
            or self.driving_mode == DrivingModeStatesEnum.DRIVERLESS
            and not self.di_signals["bypass_status"]
        ):
            self.driving_mode = (
                DrivingModeStatesEnum.DRIVERLESS
                if self.di_signals["bypass_status"]
                else DrivingModeStatesEnum.MANUAL
            )

            # Do safety checks
            checks_ok, msg = self.initial_checkup()
            if not checks_ok:
                self.change_state(OrionStateEnum.ERROR)
                self.set_health(
                    DiagnosticStatus.ERROR,
                    f"Initial checkup failed in mode '{self.driving_mode}'. Got error '{msg}'",
                )

            self.activate_nodes(self.driving_mode, None)

        # Update car and diagnostics
        self.send_status_over_can()
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: Orion state", str(self.car_state)
            )
        )
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: AS state", str(self.as_state)
            )
        )

        # Step job scheduler
        self.job_scheduler.step()

        # Toggle watchdog as final step
        if self.wd_trigger_enable:
            self.do_publishers["wd_trigger"].publish(Bool(data=self.wd_trigger_status))
            self.wd_trigger_status = not self.wd_trigger_status

    def update_car_state(self):
        """
        Sets the car state based on current state and incoming signals
        """

        # Run boot checklist, then go to SDC_OPEN
        if self.car_state == OrionStateEnum.INIT:
            if self.boot_procedure():
                self.do_publishers["sdc_close"].publish(Bool(data=True))
                self.change_state(OrionStateEnum.SDC_OPEN)

        # Transition from SDC_OPEN to TS_READY when SDC is ok
        elif self.car_state == OrionStateEnum.SDC_OPEN:
            if self.di_signals["sdc_out"] is True:
                self.change_state(OrionStateEnum.TS_READY)

        # If TS is pressed, go to TS_ACTIVATING and then TS_ACTIVE
        elif self.car_state == OrionStateEnum.TS_READY:
            # Transition differs from manual to autonomous
            if (
                self.can_actions["ts_pressed"]
                and self.driving_mode == DrivingModeStatesEnum.MANUAL
                or self.di_signals["dv_btn"]
                and self.driving_mode == DrivingModeStatesEnum.DRIVERLESS
            ):
                self.can_actions["ts_pressed"] = False
                self.change_state(OrionStateEnum.TS_ACTIVATING)

                # Activate TS
                self.press_btn_procedure("ts_btn", 0.1, 1.0)

                # Enable state timeout for this state
                self.timeout_state_procedure(5.0)

        # Wait for AIRS to close
        elif self.car_state == OrionStateEnum.TS_ACTIVATING:
            if self.can_inputs["air1"] and self.can_inputs["air2"]:
                self.change_state(OrionStateEnum.TS_ACTIVE)

        # If both brake pressures are above 5, go to R2D_READY
        elif self.car_state == OrionStateEnum.TS_ACTIVE:
            if self.ai_signals["front_bp"] > 5 and self.ai_signals["rear_bp"] > 5:
                self.change_state(OrionStateEnum.R2D_READY)

        # If R2D is pressed, go to R2D (when ok)
        elif self.car_state == OrionStateEnum.R2D_READY:
            if self.ai_signals["front_bp"] < 5 or self.ai_signals["rear_bp"] > 5:
                self.change_state(OrionStateEnum.TS_ACTIVE)

            elif (
                self.can_actions["r2d_pressed"]
                and self.driving_mode == DrivingModeStatesEnum.MANUAL
                or self.can_inputs["res_go"]
                and self.driving_mode == DrivingModeStatesEnum.DRIVERLESS
                and not self.job_scheduler.tag_exists("r2d_dv_wait")
            ):
                self.can_actions["r2d_pressed"] = False
                self.change_state(OrionStateEnum.R2D)

        elif self.car_state == OrionStateEnum.ERROR:
            self.do_publishers["sdc_close"].publish(Bool(data=False))

    def update_as_state(self):
        # Flowchart to determine AS state
        if self.driving_mode == DrivingModeStatesEnum.MANUAL:
            self.change_as_state(AutonomousStatesEnum.ASOFF)
        elif self.ebs_activated:
            if self.mission_finished and self.vehicle_stopped:
                if self.can_inputs["res_activated"]:
                    self.change_as_state(AutonomousStatesEnum.ASEMERGENCY)
                else:
                    self.change_as_state(AutonomousStatesEnum.ASFINISHED)
            else:
                self.change_as_state(AutonomousStatesEnum.ASEMERGENCY)
        else:
            if (
                self.mission_selected
                and self.di_signals["bypass_status"]
                and self.di_signals["asms_status"]
                and self.car_state == OrionStateEnum.TS_ACTIVE
                or self.car_state == OrionStateEnum.R2D
                or self.car_state == OrionStateEnum.R2D_READY
            ):
                if self.car_state == OrionStateEnum.R2D:
                    self.change_as_state(AutonomousStatesEnum.ASDRIVE)
                elif self.car_state == OrionStateEnum.R2D_READY:
                    self.change_as_state(AutonomousStatesEnum.ASREADY)
                else:
                    self.change_as_state(AutonomousStatesEnum.ASOFF)

        # Update jobs
        if self.as_state != AutonomousStatesEnum.ASREADY:
            self.job_scheduler.remove_job_by_tag("r2d_dv_wait")
            self.as_ready_transitioned = False
        else:
            if not self.as_ready_transitioned:
                self.job_scheduler.add_job_relative(
                    lambda: None, 5.0, tag="r2d_dv_wait"
                )
                self.as_ready_transitioned = True

        # Emergency stop
        if self.as_state == AutonomousStatesEnum.ASEMERGENCY:
            self.do_publishers["arm_ebs"].publish(Bool(data=True))
            self.do_publishers["arm_dbs"].publish(Bool(data=True))
            self.wd_trigger_enable = False

    """

    PROCEDURES

    """

    def press_btn_procedure(self, btn, delay, duration, cooldown=0.5):
        # Only do this if button was already released
        if self.job_scheduler.tag_exists(f"cooldown_btn_{btn}"):
            return

        self.job_scheduler.add_job_relative(
            partial(self.do_publishers[btn].publish, Bool(data=True)),
            delay,
            tag=f"press_btn_{btn}",
        )
        self.job_scheduler.add_job_relative(
            partial(self.do_publishers[btn].publish, Bool(data=False)),
            delay + duration,
            tag=f"release_btn_{btn}",
        )
        self.job_scheduler.add_job_relative(
            lambda: None, delay + duration + cooldown, tag=f"cooldown_btn_{btn}"
        )

    def timeout_state_procedure(self, duration):
        self.job_scheduler.add_job_relative(
            partial(self.change_state, OrionStateEnum.INIT),
            duration,
            tag="timeout_state",
        )

    def boot_procedure(self):
        """
        Orion BOOT checklist. Returns True if everything is OK, False if not.
        Checklist:
        - WD status high (resets watchdog if not)
        - All healtchecks OK (hardware AND software)
        """

        # check heartbeats of low voltage systems, motorcontrollers and sensors
        # ! Skipped for now
        # for i, hb in enumerate(self.hbs):
        #     if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
        #         self.send_error_to_db(13 + i)
        #         return False

        # watchdog OK?
        if self.di_signals["wd_ok"] is False:
            self.press_btn_procedure("wd_reset", 0.1, 1.0)
            return False

        # check ipc, sensors and actuators
        if self.get_health_level() == DiagnosticStatus.ERROR:
            return False

        return True

    def initial_checkup(self):
        if self.driving_mode == DrivingModeStatesEnum.MANUAL:
            # ASMS needs to be off
            if not self.di_signals["asms_status"]:
                return False, "ASMS is ON"

            # bypass needs to be off
            if not self.di_signals["bypass_status"]:
                return False, "BYPASS is ON"

            # check air pressures
            if not (
                self.ai_signals["air_pressure1"] < 1
                and self.ai_signals["air_pressure1"] < 1
            ):
                return False, "ASB is still ENABLED"

        else:
            return False, "Not yet implemented"

        return True

    def monitor(self):
        # # check heartbeats of low voltage systems, motorcontrollers and sensors
        # for i, hb in enumerate(self.hbs):
        #     if rospy.Time.now().to_sec() - self.hbs[hb] > 0.5:
        #         self.send_error_to_db(13 + i)
        #         return False

        # # check ipc, sensors and actuators
        # if self.get_health_level() == DiagnosticStatus.ERROR:
        #     self.send_error_to_db(23)
        #     return False

        # check output signal of watchdog
        if not self.watchdog_status:
            self.send_error_to_db(6)
            return False

        # # check air pressures
        # if not (self.air_pressure1 < 1 and self.air_pressure2 < 1):
        #     self.send_error_to_db(5)
        #     return False

        # # check if bypass is closed
        # if self.bypass_status:
        #     self.send_error_to_db(24)
        #     return False

        return True


node = OrionState()  #
