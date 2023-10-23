#! /usr/bin/python3
import rospy

from ugr_msgs.msg import State
from std_msgs.msg import UInt16

from node_fixture.node_fixture import AutonomousStatesEnum, StateMachineScopeEnum
from launcher import Launcher
from tuner import Tuner

from time import sleep
import yaml

class Main:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param('~map', "circle_R15") + ".yaml"
        self.mission = rospy.get_param('~mission', "trackdrive")
        self.max_time = rospy.Duration.from_sec(rospy.get_param('~max_time', 60))
        self.number_of_simulations = rospy.get_param('~number_of_simulations', 2)
        self.yaml_file_path = rospy.get_param('~yaml_file_path')
        self.parameter = rospy.get_param('~parameter')

        # Parses the yaml file into the self.data
        with open(self.yaml_file_path,'r') as f:
            self.data = list(yaml.safe_load_all(f))
        

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = Launcher(rospy.get_param("~logging", True))
        self.tuner = Tuner()
        
        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/loopclosure", UInt16, self.loopclosure_callback)

        self.simulation = 0
        self.new_simulation()

        while not rospy.is_shutdown():
            #Stops the simulation if the car of the car is finished
            if(self.state == AutonomousStatesEnum.ASFINISHED):
                rospy.loginfo("Simulation is finisched")
                self.new_simulation()

            #Stops the simulation if the duration of the simulation is longer than the max time
            if(rospy.Time.now() - self.start > self.max_time):
                rospy.loginfo("Simulation out of time")
                self.new_simulation()

            sleep(0.1)

        self.launcher.shutdown()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if(data.scope == StateMachineScopeEnum.AUTONOMOUS):
            self.state = data.cur_state

    def loopclosure_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)
        
    def new_simulation(self):
        self.launcher.shutdown()

        if(self.simulation >= self.number_of_simulations):
            rospy.loginfo("Finished all simulations")
            rospy.signal_shutdown("Finished all simulations")
            return

        self.state = AutonomousStatesEnum.ASOFF
        self.set_parameter()

        sleep(1)

        self.simulation += 1
        rospy.loginfo("Start simulation %s", self.simulation)

        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)
        self.start = rospy.Time.now()

    def set_parameter(self):
        self.data[0][self.parameter] = self.tuner.change(self.data[0][self.parameter])
        with open(self.yaml_file_path, 'w') as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(f"Change parameter({self.parameter}) to {self.data[0][self.parameter]} in yaml file")

    def init_data_file():
        
        return
    
    def save_simulation_data():

        return

node = Main()
