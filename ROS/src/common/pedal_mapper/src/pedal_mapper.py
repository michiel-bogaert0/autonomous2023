#! /usr/bin/python3
import rospy
from can_msgs.msg import Frame
from node_fixture.fixture import AutonomousStatesEnum, OrionStateEnum
from node_fixture.node_manager import ManagedNode
from std_msgs.msg import Float64, Float64MultiArray
from ugr_msgs.msg import State


class PedalMapper(ManagedNode):
    """
    This the main controller for the car and the first node to "activate"
    It handles the main state machine of the car
    """

    def __init__(self) -> None:
        super().__init__("pedal_mapper")

        self.apps = 0
        self.last_received_hb = rospy.Time.now().to_sec()

        self.spin()

    def doConfigure(self):
        # ROS parameters
        self.max_deviation = rospy.get_param("~max_deviation", 10)
        self.deadzone = rospy.get_param("~deadzone", 5)
        self.max_effort = rospy.get_param("~max_effort", 2)

        self.as_state = None
        self.car_state = None

        self.front_bp = 0
        self.rear_bp = 0
        self.bpsd_triggered = False

        # Subscribers
        # CAN
        self.AddSubscriber("/ugr/can/lv/301", Frame, self.handle_can)
        self.AddSubscriber(
            "/ugr/can/lv/processed/state_bpri3", Float64, self.handle_front_bp
        )
        self.AddSubscriber(
            "/ugr/can/lv/processed/state_bpri4", Float64, self.handle_rear_bp
        )

        # Machine States
        self.AddSubscriber("/state/as", State, self.handle_as_state)
        self.AddSubscriber("/state/car", State, self.handle_car_state)

        # Publishers
        self.effort_publisher = self.AddPublisher(
            "/ugr/car/drive_effort_controller/command", Float64MultiArray, queue_size=0
        )

    def handle_front_bp(self, msg):
        self.front_bp = (msg.data - 4) / 16 * 250

    def handle_rear_bp(self, msg):
        self.rear_bp = (msg.data - 4) / 16 * 250

    def handle_as_state(self, msg):
        self.as_state = msg.cur_state

    def handle_car_state(self, msg):
        self.car_state = msg.cur_state

    def doActivate(self):
        self.apps = 0

    def handle_can(self, frame: Frame):
        # DB_Commands
        if frame.id == 0x301:
            self.set_health(0, "Received APPS signal")
            self.last_received_hb = rospy.Time.now().to_sec()

            apps1 = max(min(frame.data[0] * 1.5, 100), 0)
            apps2 = max(min(frame.data[1] * 1.5, 100), 0)

            average_apps = max(min((apps1 + apps2) / 2, 100), 0)

            # Check if in range
            if (
                abs(apps1 - apps2) <= self.max_deviation
                and apps1 > self.deadzone
                and apps2 > self.deadzone
            ):
                # Apply deadzone and rescale
                apps_deadzoned = (
                    0 if average_apps <= self.deadzone else average_apps - self.deadzone
                )
                apps = (
                    apps_deadzoned
                    / ((100 - self.deadzone) / 100)
                    * self.max_effort
                    / 100
                )

                # Software BSPD
                if not self.bpsd_triggered and (
                    (self.front_bp > 5 or self.rear_bp > 5) and apps > 5
                ):
                    apps = 0
                    self.bpsd_triggered = True

                elif self.bpsd_triggered:
                    apps = 0

                    if average_apps < 5:
                        self.bpsd_triggered = False

                self.apps = apps

            else:
                self.apps = 0

    def active(self):
        self.max_effort = rospy.get_param("~max_effort", 2)

        zero_torque = Float64MultiArray()
        zero_torque.data = [0, 0]

        # Check states
        if (
            self.as_state == AutonomousStatesEnum.ASOFF
            and self.car_state == OrionStateEnum.R2D
        ):
            if self.last_received_hb != 0:
                if rospy.Time.now().to_sec() - self.last_received_hb > 0.5:
                    self.set_health(2, "Lost APPS signal")
                    self.effort_publisher.publish(zero_torque)
                else:
                    torque = Float64MultiArray()
                    torque.data = [self.apps, self.apps]

                    self.effort_publisher.publish(torque)
            else:
                self.effort_publisher.publish(zero_torque)

        else:
            self.effort_publisher.publish(zero_torque)


node = PedalMapper()
