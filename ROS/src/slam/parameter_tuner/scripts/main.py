#! /usr/bin/python3
from time import sleep

import rospy
from data import SimulationData, SimulationStopEnum, getDataHandler
from evaluation import Evaluation
from launcher import getLauncher
from node_fixture.fixture import AutonomousStatesEnum, StateMachineScopeEnum
from std_msgs.msg import UInt16
from tuner import getTuner
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
        self.iterations = rospy.get_param("~iterations", 1)
        self.state = AutonomousStatesEnum.ASOFF
        self.launcher = getLauncher(rospy.get_param("~logging", True))
        self.saveData = getDataHandler(rospy.get_param("~data", "print"))
        self.tuner = getTuner(rospy.get_param("~params_config_file"))
        self.evalution = Evaluation(
            rospy.get_param("~evaluation", False), self.handle_end_evaluation
        )

        # Subscribers
        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/lapComplete", UInt16, self.lap_counter_callback)

        self.simulation = 0
        self.iterations_counter = 0
        self.evalution_finished = False
        self.simulation_finished = False
        self.start_simulation()

        while not rospy.is_shutdown():
            if not self.simulation_finished:
                # Stops the simulation if the car of the car is finished
                if self.state == AutonomousStatesEnum.ASFINISHED:
                    rospy.loginfo("Simulation is finisched")
                    self.handle_end_simulation(SimulationStopEnum.ASFINISHED)

                # Stops the simulation if the duration of the simulation is longer than the max time
                if rospy.Time.now() - self.start_time > self.max_time:
                    rospy.loginfo("Simulation out of time")
                    self.handle_end_simulation(SimulationStopEnum.TimeLimit)

            sleep(0.1)

        self.launcher.shutdown()

    def handle_end_simulation(self, stop_reason: str) -> None:
        """
        Handles the end of the simulation by updating simulation data and completing the simulation.

        Args:
            stop_reason (str): The reason for stopping the simulation.
        """
        self.launcher.shutdown()

        # Save simulation data
        self.simulation_data.duration = rospy.Time.now() - self.start_time
        self.simulation_data.stop_reason = stop_reason

        self.simulation_finished = True
        self.complete_simulation()

    def handle_end_evaluation(self, data) -> None:
        """
        Handles the end of evaluation by updating simulation data and completing the simulation.

        Args:
            data: The evaluation data containing average distance to cone SLAM and labels of cones detected by SLAM.
        """
        self.simulation_data.avgDistanceToConeSLAM = data.avgDistanceToConeSLAM
        self.simulation_data.labelsConesSlam = data.labelsConesSlam
        self.evalution_finished = True
        self.complete_simulation()

    def complete_simulation(self):
        """
        Completes the simulation process.

        If the simulation is not finished, it waits for the simulation to complete.
        If the evaluation is not finished and not turned on, it waits for the evaluation to complete.
        After the simulation and evaluation are finished, it saves the simulation data.
        If all simulations are completed, it resets the parameters and starts a new simulation.
        If all iterations are completed, it finishes all simulations and shuts down.
        """
        if not self.simulation_finished:
            rospy.loginfo("Waiting for simulation")
            return
        if not self.evalution_finished and not self.evalution.on:
            rospy.loginfo("Waiting for evaluation")
            return

        self.evalution_finished = False
        self.simulation_finished = False

        self.saveData.add(self.simulation_data)

        # Did all the simulations
        if self.tuner.simulation_finished():
            self.iterations_counter += 1
            if self.iterations_counter < self.iterations:
                rospy.loginfo("resetting parameters")
                self.tuner.reset()
                self.start_simulation()
                return
            rospy.loginfo("Finished all simulations")

            self.saveData.end()

            rospy.signal_shutdown("Finished all simulations")
            return

        self.start_simulation()

    def start_simulation(self):
        """
        Starts the simulation by changing parameters, launching the simulation, launching the evaluation,
        launching the car, and recording the start time.
        """
        self.state = AutonomousStatesEnum.ASOFF
        parameters = self.tuner.change()
        sleep(0.1)
        self.simulation += 1
        rospy.loginfo("Start simulation %s", self.simulation)
        self.simulation_data = SimulationData(self.simulation, parameters)
        self.launcher.launch_simulation(self.map_filename)
        sleep(0.1)
        self.evalution.launch()
        self.launcher.launch_car(self.mission)
        self.start_time = rospy.Time.now()

    def state_callback(self, data: State):
        """
        Callback function for handling state updates.

        Args:
            data (State): The state data received.
        """
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if data.scope == StateMachineScopeEnum.AUTONOMOUS:
            self.state = data.cur_state

    def lap_counter_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)


node = Main()
