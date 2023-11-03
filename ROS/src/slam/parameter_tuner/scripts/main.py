#! /usr/bin/python3
from time import sleep

import rospy
from data import Data, SimulationData, SimulationStopEnum
from launcher import Launcher
from node_fixture.node_fixture import AutonomousStatesEnum, StateMachineScopeEnum
from param import getTuner
from std_msgs.msg import UInt16
from ugr_msgs.msg import State


class Main:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param("~map", "circle_R15") + ".yaml"
        self.mission = rospy.get_param("~mission", "trackdrive")
        self.tuner_mode = rospy.get_param("~tuner_mode", "none")
        self.max_time = rospy.Duration.from_sec(rospy.get_param("~max_time", 60))
        self.number_of_simulations = rospy.get_param("~number_of_simulations", 2)

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = Launcher(rospy.get_param("~logging", True))
        self.param = getTuner(self.tuner_mode)()
        self.saveData = Data()

        # If the tuner mode is list set the number simulations to the lenght of the list
        if self.tuner_mode == "list":
            self.number_of_simulations = self.param.get_number_of_simulations()

        # Subscribers
        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/loopclosure", UInt16, self.loopclosure_callback)

        self.simulation = 0
        self.start_simulation()

        while not rospy.is_shutdown():
            # Stops the simulation if the car of the car is finished
            if self.state == AutonomousStatesEnum.ASFINISHED:
                rospy.loginfo("Simulation is finisched")
                self.stop_and_start_simulation(SimulationStopEnum.ASFINISHED)

            # Stops the simulation if the duration of the simulation is longer than the max time
            if rospy.Time.now() - self.start_time > self.max_time:
                rospy.loginfo("Simulation out of time")
                self.stop_and_start_simulation(SimulationStopEnum.TimeLimit)

            sleep(0.1)

        self.launcher.shutdown()

    def stop_and_start_simulation(self, stop_reason: str) -> None:
        self.launcher.shutdown()

        # Save simulation data
        self.simulation_data.duration = rospy.Time.now() - self.start_time
        self.simulation_data.stop_reason = stop_reason
        self.saveData.add(self.simulation_data)

        # Did all the simulations
        if self.simulation >= self.number_of_simulations:
            rospy.loginfo("Finished all simulations")

            rospy.loginfo(f"Data:\n{self.saveData.to_str()}")

            rospy.signal_shutdown("Finished all simulations")
            return

        self.start_simulation()

    def start_simulation(self):
        self.state = AutonomousStatesEnum.ASOFF
        parameter = self.param.change()
        sleep(1)
        self.simulation += 1
        rospy.loginfo("Start simulation %s", self.simulation)
        self.simulation_data = SimulationData(self.simulation, parameter)
        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)
        self.start_time = rospy.Time.now()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if data.scope == StateMachineScopeEnum.AUTONOMOUS:
            self.state = data.cur_state

    def loopclosure_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)


node = Main()
