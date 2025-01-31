#! /usr/bin/python3
import subprocess
import threading
import time
from collections import deque
from functools import partial
from time import sleep

import can
import cantools
import numpy as np
import rospy
from can_msgs.msg import Frame
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
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
    SLAMStatesEnum,
)
from node_fixture.node_manager import NodeManager
from scheduling import JobScheduler
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Float32, Float64, Header, Int64, UInt16
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped, State


class OrionState(NodeManager):
    """
    This the main controller for the car and the first node to "activate"
    It handles the main state machine of the car
    """

    def __init__(self) -> None:
        super().__init__("orion_state", NodeManagingStatesEnum.ACTIVE)
        self.rate = rospy.Rate(15)
        self.checkup_result = None
        self.initial_checkup_done = False

        self.debug_state = 0

        self.record_p = None

        rospy.set_param("/mission", "manual")

        # Activate nodes for driving mode
        self.activate_nodes(self.driving_mode, None)
        self.switch_controllers()
        # Wait a few seconds
        rospy.sleep(1)

        # Start default in INIT and ASOFF
        self.change_state(OrionStateEnum.INIT)
        self.change_as_state(AutonomousStatesEnum.ASOFF)
        self.switched_driving_mode = False

        # self.enable_lidar(None)
        # self.enable_lidar(None)

        # Initial checkup
        self.initial_checkup()
        checks_ok, msg = self.checkup_result
        self.checkup_result = None
        print("printing msg")
        print(msg)
        if not checks_ok:
            self.change_state(OrionStateEnum.ERROR)
            self.set_health(
                DiagnosticStatus.ERROR,
                f"Initial checkup failed in mode '{self.driving_mode}'. Got error '{msg}'",
            )

        self.spin()

    def switch_controllers(self):
        rospy.wait_for_service("/ugr/car/controller_manager/switch_controller")
        try:
            switch_controller = rospy.ServiceProxy(
                "/ugr/car/controller_manager/switch_controller", SwitchController
            )

            req = SwitchControllerRequest()

            if self.driving_mode == DrivingModeStatesEnum.MANUAL:
                req.start_controllers = [
                    "joint_state_controller",
                    "steering_position_controller",
                    "drive_effort_controller",
                ]
                req.stop_controllers = [
                    "axis0_velocity_controller",
                    "axis1_velocity_controller",
                ]
            else:
                req.start_controllers = [
                    "joint_state_controller",
                    "steering_position_controller",
                    "axis0_velocity_controller",
                    "axis1_velocity_controller",
                ]
                req.stop_controllers = ["drive_effort_controller"]

            req.strictness = SwitchControllerRequest.BEST_EFFORT

            response = switch_controller(req)

            if not response.ok:
                rospy.logerr("Could not start controllers")
                self.set_health(2, "Could not start controllers")
                self.change_state(OrionStateEnum.ERROR)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.set_health(2, "Service call failed: {e}")
            self.change_state(OrionStateEnum.ERROR)

    def doConfigure(self):
        # Job scheduler (synchronous!)
        self.job_scheduler = JobScheduler()

        # DO Signals
        self.do_publishers = {
            "ts_btn": None,
            "wd_reset": None,
            "arm_ebs": None,
            "arm_dbs": None,
            "arm_service_brake": None,
            "wd_trigger": None,
            "sdc_close": None,
        }

        self.pdu_output_status = {
            "set_TSAC_fan_PWM",
            "state_TSAL_pwr",
            "state_TSAC_pwr",
            "state_TSAC_fans_pwr",
            "state_TSAC_fan_PWM",
            "state_router_pwr",
            "state_mctrl_2_pwr",
            "state_mctrl_1_pwr",
            "state_LV_box",
            "state_lidar_bakje_pwr",
            "state_IPC_pwr",
            "state_discharge_pwr",
            "state_DB_pwr",
            "state_cooling_pump_1_pwr",
            "state_cooling_fan_1_pwr",
        }

        # CAN TX
        self.bus = self.AddPublisher("/ugr/can/lv/tx", Frame, queue_size=10)

        # States and other
        self.state_publisher = self.AddPublisher(
            "/state", State, queue_size=10, latch=True
        )
        self.orion_state_publisher = rospy.Publisher(
            "/state/car", State, queue_size=10, latch=True
        )
        self.diagnostics_publisher = self.AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.as_state_publisher = rospy.Publisher(
            "/state/as", State, queue_size=10, latch=True
        )

        self.wd_trigger_status = False
        self.wd_trigger_enable = True

        self.ebs_activated = False
        self.mission_finished = False
        self.vehicle_stopped = True
        self.mission_selected = False
        self.odom_avg = deque([], 100)

        # State machine states
        self.car_state = None
        self.as_state = None
        self.driving_mode = DrivingModeStatesEnum.MANUAL

        self.lidar_time = rospy.Time.now()
        self.cam_time = rospy.Time.now()

        # DV data
        self.dv_data = {
            "cones_count_all": 0,
            "cones_count_actual": 0,
            "lap_counter": 0,
            "speed_actual": 0,
            "steering_angle_actual": 0,
            "steering_angle_target": 0,
            "brake_target": 0,
            "left_ac_current": 0,
            "right_ac_current": 0,
        }

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
        self.debounced_signals = {
            "wd_ok": [False, rospy.Time.now().to_sec()],
            "imd_ok": [False, rospy.Time.now().to_sec()],
            "sdc_out": [False, rospy.Time.now().to_sec()],
            "ts_btn_ok": [False, rospy.Time.now().to_sec()],
            "bypass_status": [False, rospy.Time.now().to_sec()],
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
            "res_go": False,
            "res_activated": False,
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
            "gearbox_temp_left": 0,
            "gearbox_temp_right": 0,
        }

        self.ai_signals_queue = {
            "air_pressure1": deque([], 10),
            "air_pressure2": deque([], 10),
            "front_bp": deque([], 10),
            "rear_bp": deque([], 10),
            "gearbox_temp_left": deque([], 10),
            "gearbox_temp_right": deque([], 10),
        }

        self.mc_temps = {
            "mc_left_motor_temp": 0,
            "mc_right_motor_temp": 0,
            "mc_left_controller_temp": 0,
            "mc_right_controller_temp": 0,
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
            OrionStateEnum.SDC_OPEN: 7,
        }

        # Publishers
        # DO Signals
        for do_signal in self.do_publishers:
            self.do_publishers[do_signal] = self.AddPublisher(
                f"/dio/out/{do_signal}", Bool, queue_size=10
            )

        # Subscribers
        rospy.Subscriber("/state", State, self.handle_external_state_change)
        rospy.Subscriber("/ugr/car/odometry/filtered/odom", Odometry, self.handle_odom)

        rospy.Subscriber("/ugr/car/sensors/lidar", PointCloud2, self.handle_lidar)
        rospy.Subscriber("/ugr/car/sensors/cam0/image", Image, self.handle_image)

        # DI signals
        for di_signal in self.di_signals:
            self.AddSubscriber(
                f"/dio/in/{di_signal}", Bool, partial(self.handle_DI, di_signal)
            )

        # DO feedback
        for do_feedback in self.do_feedback:
            self.AddSubscriber(
                f"/dio/feedback/{do_feedback}",
                Bool,
                partial(self.handle_DO_feedback, do_feedback),
            )

        # AI signals
        for ai_signal in self.ai_signals:
            if ai_signal == "gearbox_temp_left" or ai_signal == "gearbox_temp_right":
                pass
            else:
                self.AddSubscriber(
                    f"/aio/in/{ai_signal}", Float64, partial(self.handle_AI, ai_signal)
                )

        for ai_signal in self.ai_signals:
            if ai_signal == "gearbox_temp_left" or ai_signal == "gearbox_temp_right":
                self.AddSubscriber(
                    f"/aio/in/{ai_signal}", Int64, partial(self.handle_AI, ai_signal)
                )

        # AO dbs
        self.dbs_pub = self.AddPublisher("/iologik/output1", Float64, queue_size=10)
        rospy.Subscriber("/iologik/output1", Float64, self.handle_dbs)

        # CAN
        self.AddSubscriber("/ugr/can/lv/rx", Frame, self.handle_can)
        self.AddSubscriber("/ugr/can/mc_left/rx", Frame, self.handle_can_mc)
        self.AddSubscriber("/ugr/can/mc_right/rx", Frame, self.handle_can_mc)
        self.AddSubscriber(
            "/ugr/can/mc_left/processed/actual_accurrent",
            Float64,
            self.handle_left_current,
        )
        self.AddSubscriber(
            "/ugr/can/mc_right/processed/actual_accurrent",
            Float64,
            self.handle_right_current,
        )

        # MC temps
        for side in ["left", "right"]:
            for typ in ["motor", "controller"]:
                topic = f"/ugr/can/mc_{side}/processed/actual_temp{typ}"
                self.AddSubscriber(
                    topic,
                    Float64,
                    partial(self.handle_mc_temps, f"mc_{side}_{typ}_temp"),
                )

        # Subscribers for dv data
        self.AddSubscriber(
            rospy.get_param("/observations_topic", "ugr/car/observations/lidar"),
            ObservationWithCovarianceArrayStamped,
            self.handle_observations,
        )
        self.AddSubscriber(
            "/input/map",
            ObservationWithCovarianceArrayStamped,
            self.handle_new_map,
        )
        self.AddSubscriber("/input/lapComplete", UInt16, self.handle_lap_complete)
        self.AddSubscriber(
            "/input/steering_target", Float64, self.handle_steering_target
        )
        self.AddSubscriber(
            "/input/steering_actual", Float32, self.handle_steering_actual
        )
        self.AddSubscriber("/input/odom", Odometry, self.get_odom_update)
        self.AddSubscriber("/iologik/output1", Float64, self.handle_brake_target)

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

    def handle_dbs(self, msg: Float64):
        if msg.data > 10:
            self.do_publishers["arm_service_brake"].publish(Bool(data=False))
        else:
            self.do_publishers["arm_service_brake"].publish(Bool(data=True))

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

        if new_state == AutonomousStatesEnum.ASDRIVE:
            self.dbs_pub.publish(Float64(4))

        if new_state == AutonomousStatesEnum.ASREADY:
            self.job_scheduler.add_job_relative(5.0, lambda x: None, tag="r2d_dv_wait")

        self.as_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.as_state,
                cur_state=new_state,
            )
        )
        self.state_publisher.publish(
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
    def handle_odom(self, odom: Odometry):
        """
        Just keeps track of latest odometry estimate

        Args:
            odom: the odometry message containing speed information
        """

        # If vehicle is stopped and EBS is activated, vehicle will not be able to move, so
        # latch self.vehicle_stopped
        if self.vehicle_stopped and self.ebs_activated:
            return

        self.odom_avg.append(abs(odom.twist.twist.linear.x))
        self.vehicle_stopped = sum(list(self.odom_avg)) / len(list(self.odom_avg)) < 0.1

    def handle_lidar(self, data: PointCloud2):
        self.lidar_time = rospy.Time.now()

    def handle_image(self, data: Image):
        self.cam_time = rospy.Time.now()

    def handle_external_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        if state.scope == StateMachineScopeEnum.SLAM:
            """
            When SLAM reports being in the finished mode, autonomous should perhaps
            also do something
            """

            self.mission_finished = state.cur_state == SLAMStatesEnum.FINISHED

    def handle_DI(self, io_name, msg: Bool):
        """
        Handles the incoming DI signals
        """

        if io_name in self.di_signals:
            if io_name in self.debounced_signals:
                if self.debounced_signals[io_name][0] != msg.data:
                    self.debounced_signals[io_name][0] = msg.data
                    self.debounced_signals[io_name][1] = rospy.Time.now().to_sec()

            else:
                self.di_signals[io_name] = msg.data
        else:
            rospy.logwarn(f"DI signal '{io_name}' not found in list of IO signals")

    def handle_AI(self, io_name, msg: Bool):
        """
        Handles the incoming AI signals
        """

        if io_name in self.ai_signals:
            if io_name == "air_pressure1" or io_name == "air_pressure2":
                self.ai_signals_queue[io_name].append((msg.data - 4) / 16 * 10)
            elif io_name == "front_bp" or io_name == "rear_bp":
                self.ai_signals_queue[io_name].append((msg.data - 4.3) / 16 * 250)
            elif io_name == "gearbox_temp_left" or io_name == "gearbox_temp_right":
                self.ai_signals_queue[io_name].append(
                    (msg.data * 2 / 1000) / 0.005 - 259
                )
            else:
                pass

            self.ai_signals[io_name] = np.average(self.ai_signals_queue[io_name])

        else:
            rospy.logwarn(f"AI signal '{io_name}' not found in list of IO signals")

    def handle_DO_feedback(self, io_name, msg: Bool):
        """
        Handles the incoming DO feedback signals
        """

        if io_name in self.do_feedback:
            self.do_feedback[io_name] = msg.data
        else:
            rospy.logwarn(
                f"DO feedback signal '{io_name}' not found in list of IO signals"
            )

    def handle_mc_temps(self, mc_name, msg: Float64):
        self.mc_temps[mc_name] = msg.data

    def handle_can_mc(self, frame: Frame):
        # MC CAN HB
        if frame.id == 2147484230 or frame.id == 2147484229:
            self.hbs["MC"] = rospy.Time.now().to_sec()

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 768:
            self.can_actions["ts_pressed"] = bool(frame.data[0] & 0b01000000)
            self.can_actions["r2d_pressed"] = bool(frame.data[0] & 0b10000000)
            AMI_state = int(frame.data[0] & 0b00000111)

            if AMI_state == 0:
                rospy.set_param("/mission", "manual")
            elif AMI_state == 1:
                rospy.set_param("/mission", "acceleration")
            elif AMI_state == 2:
                rospy.set_param("/mission", "skidpad")
            elif AMI_state == 3:
                rospy.set_param("/mission", "trackdrive")
            elif AMI_state == 4:
                rospy.set_param("/mission", "braketest")
            elif AMI_state == 5:
                rospy.set_param("/mission", "inspection")
            elif AMI_state == 6:
                rospy.set_param("/mission", "autocross")
            else:
                rospy.set_param("/mission", "manual")

        # PDU status
        if frame.id == 0x111:
            self.pdu_output_status = self.db.decode_message(frame.id, frame.data)
            # print(self.pdu_output_status)

        # RES
        if frame.id == 0x711:
            self.bus.publish(
                (
                    Frame(
                        id=0x000,
                        data=[0x01, 0x11, 0x0, 0, 0, 0, 0, 0],
                        dlc=8,
                        is_extended=False,
                    )
                )
            )

        if frame.id == 0x191:
            self.can_inputs["res_go"] = bool((frame.data[0] & 0b0000100) >> 2)
            self.can_inputs["res_activated"] = not bool(frame.data[0] & 0b0000001)

        # LV ECU HBS
        # PDU
        if frame.id == 16:
            self.hbs["PDU"] = rospy.Time.now().to_sec()

        # ELVIS
        if frame.id == 2:
            self.can_inputs["air1"] = bool(frame.data[0] & 0b00100000)
            self.can_inputs["air2"] = bool(frame.data[0] & 0b00010000)

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

    def handle_observations(self, msg: ObservationWithCovarianceArrayStamped):
        self.dv_data["cones_count_actual"] = len(msg.observations)

    def handle_new_map(self, msg: ObservationWithCovarianceArrayStamped):
        self.dv_data["cones_count_all"] = len(msg.observations)

    def handle_lap_complete(self, msg: UInt16):
        self.dv_data["lap_counter"] = msg.data

    def handle_steering_target(self, msg: Float64):
        self.dv_data["steering_angle_target"] = msg.data

    def handle_steering_actual(self, msg: Float32):
        self.dv_data["steering_angle_actual"] = msg.data

    def handle_brake_target(self, msg: Float64):
        self.dv_data["brake_target"] = msg.data

    def get_odom_update(self, msg: Odometry):
        self.dv_data["speed_actual"] = msg.twist.twist.linear.x

    def handle_left_current(self, msg: Float64):
        self.dv_data["left_ac_current"] = msg.data

    def handle_right_current(self, msg: Float64):
        self.dv_data["right_ac_current"] = msg.data

    def send_status_over_can(self):
        #
        # 0x500 acc. to rules
        #
        # dv1_message = self.db.get_message_by_name("DV_1")
        # brake_hydr = 100 if self.ebs_activated else 0
        # motor_moment_actual = (
        #     self.dv_data["right_ac_current"] + self.dv_data["left_ac_current"]
        # ) / (
        #     240 * 2
        # )  # 240 = max ac current
        # data = dv1_message.encode(
        #     {
        #         "Motor_moment_target": motor_moment_actual,  # percentage // target topic ros control, dit van current naar moment
        #         "Motor_moment_actual": motor_moment_actual,  # percentage
        #         "Brake_hydr_actual": abs(brake_hydr),  # percentage
        #         "Brake_hydr_target": abs(brake_hydr),  # percentage
        #         "Steering_angle_target": self.dv_data["steering_angle_target"]
        #         * 2
        #         * 180
        #         / math.pi,  # degrees, scale 0.5
        #         "Steering_angle_actual": self.dv_data["steering_angle_actual"]
        #         * 2
        #         * 180
        #         / math.pi,  # degrees, scale 0.5
        #         "Speed_target": abs(rospy.get_param("/speed/target", 0.0))
        #         * 3.6,  # km/h
        #         "Speed_actual": abs(self.dv_data["speed_actual"]) * 3.6,  # km/h
        #     }
        # )

        # message = can.Message(
        #     arbitration_id=dv1_message.frame_id, data=data, is_extended_id=False
        # )

        # self.bus.publish(serialcan_to_roscan(message))

        #
        # 0x502 acc. to rules
        #
        as_state_bits = 0
        if self.as_state == AutonomousStatesEnum.ASOFF:
            as_state_bits = 1
        elif self.as_state == AutonomousStatesEnum.ASREADY:
            as_state_bits = 2
        elif self.as_state == AutonomousStatesEnum.ASDRIVE:
            as_state_bits = 3
        elif self.as_state == AutonomousStatesEnum.ASEMERGENCY:
            as_state_bits = 4
        elif self.as_state == AutonomousStatesEnum.ASFINISHED:
            as_state_bits = 5
        dv_message = self.db.get_message_by_name("DV_status")

        ami_state_bits = 0
        if self.driving_mode == DrivingModeStatesEnum.MANUAL:
            ami_state_bits = 0
        else:
            mission = rospy.get_param("/mission")
            if mission == "acceleration":
                ami_state_bits = 1
            elif mission == "skidpad":
                ami_state_bits = 2
            elif mission == "trackdrive":
                ami_state_bits = 3
            elif mission == "braketest":
                ami_state_bits = 4
            elif mission == "inspection":
                ami_state_bits = 5
            elif mission == "autocross":
                ami_state_bits = 6

        if not self.di_signals["bypass_status"]:
            service_brake_state = 1  # disengaged
        elif self.as_state in {
            AutonomousStatesEnum.ASEMERGENCY,
            AutonomousStatesEnum.ASFINISHED,
            AutonomousStatesEnum.ASDRIVE,
            AutonomousStatesEnum.ASREADY,
        }:
            service_brake_state = 3  # available
        else:
            service_brake_state = 2  # engaged

        data = dv_message.encode(
            {
                "Cones_count_all": abs(self.dv_data["cones_count_all"]),
                "Cones_count_actual": abs(self.dv_data["cones_count_actual"]),
                "Lap_counter": 0,
                "Service_brake_state": service_brake_state,
                "Steering_state": 0,
                "AMI_state": ami_state_bits,
                "EBS_state": self.ebs_activated + 1,
                "AS_state": as_state_bits,
            }
        )

        message = can.Message(
            arbitration_id=dv_message.frame_id, data=data, is_extended_id=False
        )
        self.bus.publish(serialcan_to_roscan(message))

        #
        # Heartbeat & CAR state
        #

        state_bits = self.state_to_bits.get(self.car_state, 0b0000)

        # Bit 0: Heartbeat
        # Bits 1-4: State
        data = [
            (0b1) | ((state_bits & 0b111) << 1) | ((self.debug_state & 0x1111) << 4)
        ]

        canmsg = can.Message(
            arbitration_id=1,
            data=data,
            is_extended_id=False,
        )
        self.bus.publish(serialcan_to_roscan(canmsg))

        # MC
        mc_temp_message = self.db.get_message_by_name("MCU_temp")
        data = mc_temp_message.encode(
            {
                "Temp_motor_R": max(0, self.mc_temps["mc_right_motor_temp"]),
                "Temp_motor_L": max(0, self.mc_temps["mc_left_motor_temp"]),
                "Temp_invertor_R": max(0, self.mc_temps["mc_right_controller_temp"]),
                "Temp_invertor_L": max(0, self.mc_temps["mc_left_controller_temp"]),
            }
        )
        message = can.Message(arbitration_id=mc_temp_message.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))

    def start_initial_checkup(self):
        thread = threading.Thread(target=self.initial_checkup, args=())
        thread.start()

    def active(self):
        rospy.loginfo_throttle(5, self.as_state)
        self.send_status_over_can()
        if self.switched_driving_mode and self.checkup_result is not None:
            checks_ok, msg = self.checkup_result
            self.checkup_result = None
            self.switched_driving_mode = False
            self.do_publishers["sdc_close"].publish(Bool(data=True))

            if not checks_ok:
                self.change_state(OrionStateEnum.ERROR)
                self.set_health(
                    DiagnosticStatus.ERROR,
                    f"Initial checkup failed in mode '{self.driving_mode}'. Got error '{msg}'",
                )
            # self.activate_nodes(self.driving_mode, None)
            print("SWITCHNG controllers \n\n\n")
            self.switch_controllers()

        # Update state machines
        self.update_car_state()
        self.update_as_state()

        # Update deboucners
        for io_name in self.debounced_signals:
            if (
                self.debounced_signals[io_name][0] != self.di_signals[io_name]
                and rospy.Time.now().to_sec() - self.debounced_signals[io_name][1] > 0.5
            ):
                self.di_signals[io_name] = self.debounced_signals[io_name][0]

        # Check if driving mode should be updated
        if (
            (
                not self.switched_driving_mode
                and self.driving_mode == DrivingModeStatesEnum.MANUAL
                and self.di_signals["bypass_status"]
                or self.driving_mode == DrivingModeStatesEnum.DRIVERLESS
                and not self.di_signals["bypass_status"]
            )
            and self.car_state != OrionStateEnum.R2D
            and self.car_state != OrionStateEnum.ERROR
        ):
            self.driving_mode = (
                DrivingModeStatesEnum.DRIVERLESS
                if self.di_signals["bypass_status"]
                else DrivingModeStatesEnum.MANUAL
            )
            # Do safety checks
            self.initial_checkup_done = False
            self.start_initial_checkup()
            self.switched_driving_mode = True

        # Update car and diagnostics
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

        if self.initial_checkup_done:
            checks_ok, msg = self.monitor()
            if not checks_ok:
                self.debug_state = 5
                self.change_state(OrionStateEnum.ERROR)
                self.set_health(
                    DiagnosticStatus.ERROR,
                    f"Monitoring failed in mode '{self.driving_mode}'. Got error '{msg}'",
                )

        # Toggle watchdog
        if self.wd_trigger_enable:
            self.do_publishers["wd_trigger"].publish(Bool(data=self.wd_trigger_status))
            self.wd_trigger_status = not self.wd_trigger_status

    def enable_lidar(self, _):
        msg = self.db.get_message_by_name("set_outputs")
        data = [0, 0x41, 0, 0, 0, 0, 0, 0]
        message = can.Message(arbitration_id=msg.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))
        print("Lidar enabled")

    def enable_tsac_fans(self, _):
        msg = self.db.get_message_by_name("set_outputs")
        data = [1, 1, 0, 0, 0, 0, 0, 0]
        if self.pdu_output_status["state_lidar_bakje_pwr"]:
            data[1] = 0x41
        message = can.Message(arbitration_id=msg.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))

    def enable_cooling(self, _):
        msg = self.db.get_message_by_name("set_outputs")
        data = [7, 1, 0, 0, 0, 0, 0, 0]
        if self.pdu_output_status["state_lidar_bakje_pwr"]:
            data[1] = 0x41
        message = can.Message(arbitration_id=msg.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))

    def disable_cooling(self, _):
        msg = self.db.get_message_by_name("set_outputs")
        data = [0, 1, 0, 0, 0, 0, 0, 0]
        if self.pdu_output_status["state_lidar_bakje_pwr"]:
            data[1] = 0x41
        message = can.Message(arbitration_id=msg.frame_id, data=data)
        self.bus.publish(serialcan_to_roscan(message))

    def update_car_state(self):
        """
        Sets the car state based on current state and incoming signals
        """

        if self.di_signals["sdc_out"]:
            self.job_scheduler.remove_job_by_tag("timeout_state_sdc")

        # Run boot checklist, then go to SDC_OPEN
        if self.car_state == OrionStateEnum.INIT:
            if self.boot_procedure():
                self.do_publishers["sdc_close"].publish(Bool(data=True))
                self.change_state(OrionStateEnum.SDC_OPEN)

        # Transition from SDC_OPEN to TS_READY when SDC is ok
        elif self.car_state == OrionStateEnum.SDC_OPEN:
            if self.di_signals["sdc_out"] is True:
                self.change_state(OrionStateEnum.TS_READY)

        elif self.car_state == OrionStateEnum.ERROR:
            self.do_publishers["sdc_close"].publish(Bool(data=False))

        else:
            if self.di_signals["sdc_out"] is False:
                self.timeout_sdc_open(0.5)

            # If TS is pressed, go to TS_ACTIVATING and then TS_ACTIVE
            elif self.car_state == OrionStateEnum.TS_READY:
                # Transition differs from manual to autonomous
                if (
                    self.can_actions["ts_pressed"]
                    and self.driving_mode == DrivingModeStatesEnum.MANUAL
                ) or (
                    self.di_signals["dv_btn"]
                    and self.driving_mode == DrivingModeStatesEnum.DRIVERLESS
                ):
                    print("here")
                    self.can_actions["ts_pressed"] = False
                    self.change_state(OrionStateEnum.TS_ACTIVATING)

                    # Activate TS
                    self.press_btn_procedure("ts_btn", 1.0, 1.0)

                    # Enable state timeout for this state
                    self.timeout_state_procedure(10.0)

            # Wait for AIRS to close
            elif self.car_state == OrionStateEnum.TS_ACTIVATING:
                if self.can_inputs["air1"] and self.can_inputs["air2"]:
                    self.change_state(OrionStateEnum.TS_ACTIVE)

                    # Record rosbag
                    if self.record_p is not None:
                        self.record_p.terminate()

                    self.record_p = subprocess.Popen(
                        'cd /home/ugr/rosbags && rosbag record -b 0 -a --exclude "/ugr/car/sensors/lidar|/ugr/car/sensors/cam0/image|/ouster/lidar_packets"',
                        stdin=subprocess.PIPE,
                        shell=True,
                        cwd="/",
                        executable="/bin/bash",
                    )

            # If both brake pressures are above 5, go to R2D_READY
            elif self.car_state == OrionStateEnum.TS_ACTIVE:
                if self.ai_signals["front_bp"] > 2 and self.ai_signals["rear_bp"] > 2:
                    self.change_state(OrionStateEnum.R2D_READY)

                # Enable TSAC Fans
                self.job_scheduler.remove_job_by_tag("disable_cooling")
                self.job_scheduler.add_job_relative(
                    3, self.enable_tsac_fans, tag="enable_tsac_fans"
                )

            # If R2D is pressed, go to R2D (when ok)
            elif self.car_state == OrionStateEnum.R2D_READY:
                if self.ai_signals["front_bp"] < 2 or self.ai_signals["rear_bp"] < 2:
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

                    # Enable cooling
                    self.job_scheduler.remove_job_by_tag("disable_cooling")
                    self.job_scheduler.add_job_relative(
                        1, self.enable_cooling, tag="enable_cooling"
                    )

                    if self.driving_mode == DrivingModeStatesEnum.DRIVERLESS:
                        self.job_scheduler.add_job_relative(
                            3, lambda x: None, tag="r2d_sound_delay"
                        )

                    # Enable drive
                    arbitration_id = 0x24FF
                    data = [1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
                    message = can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=True,
                        dlc=8,
                    )
                    self.bus.publish(serialcan_to_roscan(message))

        # Stop record script
        if self.car_state not in [
            OrionStateEnum.TS_ACTIVE,
            OrionStateEnum.TS_ACTIVATING,
            OrionStateEnum.R2D_READY,
            OrionStateEnum.R2D,
        ]:
            if self.record_p is not None:
                self.record_p.terminate()
                self.record_p = None

            # Stop cooling
            self.job_scheduler.remove_job_by_tag("enable_cooling")
            self.job_scheduler.remove_job_by_tag("enable_tsac_fans")
            if not self.job_scheduler.tag_exists("disable_cooling"):
                self.job_scheduler.add_job_relative(
                    1, self.disable_cooling, tag="disable_cooling"
                )

    def update_as_state(self):
        if self.initial_checkup_done:
            # Signals
            # arm_dbs signal is not used to check if EBS is activated because
            # arm_dbs is actually the service brake and in case of an
            # emergecny, arm_ebs and arm_dbs are always triggered together
            self.ebs_activated = (
                # TODO change sdc_out to check state instead
                self.car_state == OrionStateEnum.SDC_OPEN
                or self.car_state == OrionStateEnum.ERROR
                or self.do_feedback["arm_ebs"] is False
                or self.can_inputs["res_activated"] is True
            )
            self.mission_selected = (
                rospy.has_param("/mission") and rospy.get_param("/mission") != ""
            )

            # Flowchart to determine AS state
            if self.driving_mode == DrivingModeStatesEnum.MANUAL:
                self.change_as_state(AutonomousStatesEnum.ASOFF)
            elif self.ebs_activated:
                if self.mission_finished and self.vehicle_stopped:
                    if self.can_inputs["res_activated"]:
                        print("EMERGECNY 1")
                        print(self.do_feedback)
                        self.change_as_state(AutonomousStatesEnum.ASEMERGENCY)
                    else:
                        self.change_as_state(AutonomousStatesEnum.ASFINISHED)
                        self.do_publishers["sdc_close"].publish(Bool(data=False))
                else:
                    print("EMERGECNY 2")
                    print(self.do_feedback)
                    self.change_as_state(AutonomousStatesEnum.ASEMERGENCY)
            else:
                if self.mission_finished and self.vehicle_stopped:
                    self.do_publishers["arm_ebs"].publish(Bool(data=False))
                    self.do_publishers["arm_dbs"].publish(Bool(data=False))
                    # self.do_publishers["sdc_close"].publish(Bool(data=False))

                if (
                    self.mission_selected
                    and self.di_signals["bypass_status"]
                    and self.car_state
                    in [
                        OrionStateEnum.TS_ACTIVE,
                        OrionStateEnum.R2D,
                        OrionStateEnum.R2D_READY,
                    ]
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

            # Emergency stop
            if self.as_state == AutonomousStatesEnum.ASEMERGENCY:
                self.do_publishers["arm_ebs"].publish(Bool(data=False))
                self.do_publishers["arm_dbs"].publish(Bool(data=False))
                # self.do_publishers["sdc_close"].publish(Bool(data=False))
                self.wd_trigger_enable = False
        else:
            self.as_state = AutonomousStatesEnum.ASOFF

    """

    PROCEDURES

    """

    def press_btn_procedure(self, btn, delay, duration, cooldown=0.5):
        # Only do this if button was already released
        if self.job_scheduler.tag_exists(f"cooldown_btn_{btn}"):
            return

        self.job_scheduler.add_job_relative(
            delay,
            self.do_publishers[btn].publish,
            Bool(data=True),
            tag=f"press_btn_{btn}",
        )
        self.job_scheduler.add_job_relative(
            delay + duration,
            self.do_publishers[btn].publish,
            Bool(data=False),
            tag=f"release_btn_{btn}",
        )
        self.job_scheduler.add_job_relative(
            delay + duration + cooldown, lambda x: None, tag=f"cooldown_btn_{btn}"
        )

    def timeout_sdc_open(self, duration):
        def interfere(state):
            print("Interfering...")
            self.change_state(state)

        self.job_scheduler.add_job_relative(
            duration,
            interfere,
            OrionStateEnum.SDC_OPEN,
            tag="timeout_state_sdc",
        )

    def timeout_state_procedure(self, duration):
        def interfere(state):
            print("Interfering...")
            self.change_state(state)

        self.job_scheduler.add_job_relative(
            duration,
            interfere,
            OrionStateEnum.INIT,
            tag="timeout_state",
        )

    def boot_procedure(self):
        """
        Orion BOOT checklist. Returns True if everything is OK, False if not.
        Checklist:
        - WD status high (resets watchdog if not)
        - All healtchecks OK (hardware AND software)
        """
        # watchdog OK?
        if self.di_signals["wd_ok"] is False:
            self.press_btn_procedure("wd_reset", 0.1, 1.0)
            return False

        # check ipc, sensors and actuators
        if self.get_health_level() == DiagnosticStatus.ERROR:
            return False

        return True

    def initial_checkup(self):
        self.debug_state = 0

        if self.driving_mode == DrivingModeStatesEnum.MANUAL:
            # bypass needs to be off
            if self.di_signals["bypass_status"]:
                self.checkup_result = (False, "BYPASS is ON")
                return

            # check air pressures
            # if (
            #     self.ai_signals["air_pressure1"] > 1
            #     or self.ai_signals["air_pressure2"] > 1
            # ):
            #     self.checkup_result = (
            #         False,
            #         f"ASB is still ENABLED: Reading AP1: {self.ai_signals['air_pressure1']}, AP2: {self.ai_signals['air_pressure2']}",
            #     )
            #     return

        else:
            self.debug_state = 1

            # time.sleep(4)
            # print("LOLOLO\n\n\n\n\n")

            # wait (asms still registering)

            self.do_publishers["arm_ebs"].publish(Bool(data=False))
            self.do_publishers["arm_dbs"].publish(Bool(data=False))

            time.sleep(2)
            print("start")

            # mission needs to be selected
            if rospy.get_param("/mission") == "manual" or (
                not (rospy.has_param("/mission") and rospy.get_param("/mission") != "")
            ):
                self.checkup_result = (False, "No mission selected")
                return (False, "NO MISSION SELECTED")

            print("mission")

            # bypass needs to be on
            if not self.di_signals["bypass_status"]:
                self.checkup_result = (False, "BYPASS is OFF")
                return (False, "BYPASS IS OFF")

            print("bypass ok")

            # check air pressures
            if not (
                self.ai_signals["air_pressure1"] > 5
                and self.ai_signals["air_pressure2"] > 5
                and self.ai_signals["air_pressure1"] < 11
                and self.ai_signals["air_pressure2"] < 11
            ):
                self.checkup_result = (
                    False,
                    f"Air pressures out of range: Reading AP1: {self.ai_signals['air_pressure1']}, AP2: {self.ai_signals['air_pressure2']}",
                )
                return

            print("air ok")

            self.debug_state = 2

            # watchdog OK?
            if self.di_signals["wd_ok"] is False:
                self.checkup_result = (False, "Watchdog indicating error")
                return

            print("wd ok")

            # stop toggling watchdog
            self.wd_trigger_enable = False

            time.sleep(3)

            # check whether watchdog indicating error
            if self.di_signals["wd_ok"] is True:
                self.checkup_result = (
                    False,
                    "Watchdog not indicating error after we stopped toggling",
                )
                return

            print("wd error ok")

            # start toggling watchdog
            self.wd_trigger_enable = True

            # reset watchdog
            self.do_publishers["wd_reset"].publish(Bool(data=True))
            time.sleep(0.5)
            self.do_publishers["wd_reset"].publish(Bool(data=False))
            time.sleep(2)

            print("reset ok")

            # watchdog OK?
            if self.di_signals["wd_ok"] is False:
                self.checkup_result = (False, "Watchdog indicating error")
                return (False, "WD indicates error")

            self.debug_state = 3

            self.activate_nodes(self.driving_mode, None)
            print("okok ASRRRR")

            # Wait until we are in TS_ACTIVE
            ts_active = False
            t = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - t < 100:
                if self.car_state == OrionStateEnum.TS_ACTIVE:
                    ts_active = True
                    time.sleep(0.200)
                    break

            if ts_active is False:
                return False, "Timed out waiting for TS_ACTIVE"

            rospy.sleep(1)

            # ARM ebs/dbs
            self.do_publishers["arm_ebs"].publish(Bool(data=True))
            self.do_publishers["arm_dbs"].publish(Bool(data=True))
            rospy.sleep(1)

            # check whether pressure is being released as expected
            if (
                self.ai_signals["front_bp"] < 10 and self.ai_signals["rear_bp"] < 10
            ) is False:
                return False, "Brake pressure not released as expected"

            # trigger ebs
            self.do_publishers["arm_ebs"].publish(Bool(data=False))
            time.sleep(1)

            # check whether pressure is being built up as expected
            if (
                self.ai_signals["front_bp"] > 10 and self.ai_signals["rear_bp"] > 10
            ) is False:
                return False, "EBS brake pressures not built up as expected"

            # release ebs
            self.do_publishers["arm_ebs"].publish(Bool(data=True))
            time.sleep(1)

            # check whether pressure is being released as expected
            if (
                self.ai_signals["front_bp"] < 10 and self.ai_signals["rear_bp"] < 10
            ) is False:
                return False, "EBS brake pressures not released as expected"

            # trigger dbs
            self.do_publishers["arm_dbs"].publish(Bool(data=False))
            time.sleep(1)

            # check whether pressure is being built up as expected
            if (
                self.ai_signals["front_bp"] > 10 and self.ai_signals["rear_bp"] > 10
            ) is False:
                return False, "DBS brake pressures not built up as expected"

            # release dbs
            self.do_publishers["arm_dbs"].publish(Bool(data=True))
            time.sleep(1)

            # check whether pressure is being released as expected
            if (
                self.ai_signals["front_bp"] < 10 and self.ai_signals["rear_bp"] < 10
            ) is False:
                return False, "DBS brake pressures not released as expected"

            # set PPR setpoint, actuate brake with DBS
            self.dbs_pub.publish(Float64(12))
            time.sleep(1)

            # check whether pressure is being built up as expected
            if (
                self.ai_signals["front_bp"] > 10 and self.ai_signals["rear_bp"] > 10
            ) is False:
                return False, "Missed PPR setpoint"

            self.debug_state = 4

        self.initial_checkup_done = True
        self.checkup_result = True, "OK"
        return

    def monitor(self):
        if self.car_state == OrionStateEnum.R2D:
            # check heartbeats of low voltage systems, motorcontrollers and sensors TODO
            if self.driving_mode == DrivingModeStatesEnum.MANUAL:
                if self.di_signals["bypass_status"]:
                    return False, "BYPASS is ON"
            else:
                print(self.di_signals)
                if not self.di_signals["bypass_status"]:
                    return False, "BYPASS is OFF"

            # Check brake pressures
            if self.ai_signals["front_bp"] < -10 or self.ai_signals["rear_bp"] < -10:
                return False, "Lost brake pressure sensors"

            # check air pressures
            if self.driving_mode == DrivingModeStatesEnum.MANUAL:
                # if not (
                #     self.ai_signals["air_pressure1"] < 1
                #     and self.ai_signals["air_pressure2"] < 1
                # ):
                #     return False, "Air pressures not released"
                pass
            else:
                print(self.ai_signals)
                if not (
                    self.ai_signals["air_pressure1"] > 5
                    and self.ai_signals["air_pressure2"] > 5
                    and self.ai_signals["air_pressure1"] < 11
                    and self.ai_signals["air_pressure2"] < 11
                ):
                    return False, "Air pressures out of range"

            # Check if lidar and camera are working (for autonomous)
            if self.driving_mode == DrivingModeStatesEnum.DRIVERLESS:
                if self.cam_time - rospy.Time.now() > rospy.Duration(
                    0.5
                ) and self.lidar_time - rospy.Time.now() > rospy.Duration(0.5):
                    # 300ms
                    return False, "No Lidar PointClouds or Camera Image received"

        return True, "OK"


node = OrionState()
