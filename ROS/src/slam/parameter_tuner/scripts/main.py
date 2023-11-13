#! /usr/bin/python3
from time import sleep

import rospy
from data import Data, SimulationData, SimulationStopEnum
from launcher import getLauncher
from node_fixture.fixture import AutonomousStatesEnum, StateMachineScopeEnum
from node_launcher.node_launcher import NodeLauncher
from std_msgs.msg import UInt16
from tuner import getTuner
from ugr_msgs.msg import State, TrajectoryMapInfo


class Main:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param("~map", "circle_R15") + ".yaml"
        self.mission = rospy.get_param("~mission", "trackdrive")
        self.max_time = rospy.Duration.from_sec(rospy.get_param("~max_time", 60))

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = getLauncher(rospy.get_param("~logging", True))
        self.saveData = Data()
        self.tuner = getTuner(rospy.get_param("~params_config_file"))

        # Subscribers
        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/loopclosure", UInt16, self.loopclosure_callback)
        rospy.Subscriber(
            "/ugr/car/evaluation/infoMap", TrajectoryMapInfo, self.evaluation_callback
        )

        self.evaluationLauncher = NodeLauncher()

        self.simulation = 0
        self.got_evaluation = False
        self.simulation_finished = False
        self.start_simulation()

        while not rospy.is_shutdown():
            if not self.simulation_finished:
                # Stops the simulation if the car of the car is finished
                if self.state == AutonomousStatesEnum.ASFINISHED:
                    rospy.loginfo("Simulation is finisched")
                    self.stop_simulation(SimulationStopEnum.ASFINISHED)

                # Stops the simulation if the duration of the simulation is longer than the max time
                if rospy.Time.now() - self.start_time > self.max_time:
                    rospy.loginfo("Simulation out of time")
                    self.stop_simulation(SimulationStopEnum.TimeLimit)

            sleep(0.1)

        self.launcher.shutdown()

    def stop_simulation(self, stop_reason: str) -> None:
        self.launcher.shutdown()

        # Save simulation data
        self.simulation_data.duration = rospy.Time.now() - self.start_time
        self.simulation_data.stop_reason = stop_reason

        self.simulation_finished = True
        if self.got_evaluation:
            self.handle_end_simulation()
        else:
            rospy.loginfo("Waiting for evaluation")

    def handle_end_simulation(self):
        rospy.loginfo("Simulation finished")

        self.got_evaluation = False
        self.simulation_finished = False

        self.saveData.add(self.simulation_data)

        # Did all the simulations
        if self.tuner.simulation_finished():
            rospy.loginfo("Finished all simulations")

            rospy.loginfo(f"Data:\n{self.saveData.to_str()}")

            rospy.signal_shutdown("Finished all simulations")
            return

        self.start_simulation()

    def start_simulation(self):
        self.state = AutonomousStatesEnum.ASOFF
        parameters = self.tuner.change()
        sleep(1)
        self.simulation += 1
        rospy.loginfo("Start simulation %s", self.simulation)
        self.simulation_data = SimulationData(self.simulation, parameters)
        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.evaluationLauncher.launch_node(
            "slam_trajectory_evaluation",
            "launch/trajectory_evaluation.launch",
            [str("printFiles:=Fase")],
        )
        self.evaluationLauncher.run()
        self.launcher.launch_car(self.mission)
        self.start_time = rospy.Time.now()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if data.scope == StateMachineScopeEnum.AUTONOMOUS:
            self.state = data.cur_state

    def loopclosure_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)

    def evaluation_callback(self, data):
        rospy.loginfo("Evaluation: %s", data.avgDistanceToConeSLAM)
        self.simulation_data.avgDistanceToConeSLAM = data.avgDistanceToConeSLAM
        self.got_evaluation = True
        self.evaluationLauncher.shutdown()
        if self.simulation_finished:
            self.handle_end_simulation()
        else:
            rospy.loginfo("Waiting for simulation to finish")


node = Main()
