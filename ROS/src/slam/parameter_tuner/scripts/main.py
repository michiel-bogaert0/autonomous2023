#! /usr/bin/python3
from time import sleep

import rospy
import yaml
from data import Data, SimulationData, SimulationStopEnum
from launcher import Launcher
from node_fixture.node_fixture import AutonomousStatesEnum, StateMachineScopeEnum
from std_msgs.msg import UInt16
from tuner import Tuner
from ugr_msgs.msg import State


class Main:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param("~map", "circle_R15") + ".yaml"
        self.mission = rospy.get_param("~mission", "trackdrive")
        self.max_time = rospy.Duration.from_sec(rospy.get_param("~max_time", 60))
        self.number_of_simulations = rospy.get_param("~number_of_simulations", 2)
        self.yaml_file_path = rospy.get_param("~yaml_file_path")
        self.parameter_name = rospy.get_param("~parameter")

        # Parses the yaml file into the self.data
        with open(self.yaml_file_path, "r") as f:
            self.data = list(yaml.safe_load_all(f))

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = Launcher(rospy.get_param("~logging", True))
        self.tuner = Tuner()
        self.saveData = Data()

        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/loopclosure", UInt16, self.loopclosure_callback)

        self.simulation = 0
        self.start_simulation()

        while not rospy.is_shutdown():
            # Stops the simulation if the car of the car is finished
            if self.state == AutonomousStatesEnum.ASFINISHED:
                rospy.loginfo("Simulation is finisched")
                self.new_simulation(SimulationStopEnum.ASFINISHED)

            # Stops the simulation if the duration of the simulation is longer than the max time
            if rospy.Time.now() - self.start_time > self.max_time:
                rospy.loginfo("Simulation out of time")
                self.new_simulation(SimulationStopEnum.TimeLimit)

            sleep(0.1)

        self.launcher.shutdown()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if data.scope == StateMachineScopeEnum.AUTONOMOUS:
            self.state = data.cur_state

    def loopclosure_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)

    def new_simulation(self, stop_reason: str) -> None:
        self.launcher.shutdown()

        # Save simulation data
        self.simulation_data.duration = rospy.Time.now() - self.start_time
        self.simulation_data.stop_reason = stop_reason

        self.saveData.add(self.simulation_data)

        if self.simulation >= self.number_of_simulations:
            rospy.loginfo("Finished all simulations")

            rospy.loginfo(f"Data:\n{self.saveData.to_str()}")

            rospy.signal_shutdown("Finished all simulations")
            return

        self.start_simulation()

    def start_simulation(self):
        self.state = AutonomousStatesEnum.ASOFF
        parameter = self.set_parameter()

        sleep(1)

        self.simulation += 1
        rospy.loginfo("Start simulation %s", self.simulation)
        self.simulation_data = SimulationData(self.simulation, parameter)

        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)
        self.start_time = rospy.Time.now()

    def set_parameter(self):
        parameter = self.tuner.change(self.data[0][self.parameter_name])
        self.data[0][self.parameter_name] = parameter
        with open(self.yaml_file_path, "w") as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(
            f"Change parameter({self.parameter_name}) to {parameter} in yaml file"
        )

        return parameter


node = Main()
