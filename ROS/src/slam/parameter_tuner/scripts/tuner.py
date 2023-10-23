#! /usr/bin/python3
import rospy

from ugr_msgs.msg import State
from std_msgs.msg import UInt16

from node_fixture.node_fixture import AutonomousStatesEnum, StateMachineScopeEnum
from launcher import Launcher

from time import sleep
import yaml

class Tuner:
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

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = Launcher(rospy.get_param("~logging", True))
        
        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/ugr/car/loopclosure", UInt16, self.loopclosure_callback)

        self.simulation = 0
        self.new_simulation()


        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if(self.state == AutonomousStatesEnum.ASFINISHED):
                rospy.loginfo("Simulation is finisched")
                self.new_simulation()

            if(current_time - self.start > self.max_time):
                rospy.loginfo("Simulation out of time")
                self.new_simulation()

            sleep(0.1)

        self.launcher.shutdown_car()
        self.launcher.shutdown_simulation()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        if(data.scope == StateMachineScopeEnum.AUTONOMOUS):
            self.state = data.cur_state

    def loopclosure_callback(self, data):
        rospy.loginfo("Laps finished: %s", data.data)


    def new_simulation(self):
        self.launcher.shutdown_car()
        self.launcher.shutdown_simulation()

        if(self.simulation >= self.number_of_simulations):
            rospy.loginfo("Finished all simulations")
            rospy.signal_shutdown("Finished all simulations")
            return
        
        self.simulation = self.simulation + 1
        rospy.loginfo("Start simulation %s", self.simulation)

        self.state = AutonomousStatesEnum.ASOFF
        self.change_parameter()
        sleep(2)
        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)
        self.start = rospy.Time.now()

    def change_parameter(self):
        with open(self.yaml_file_path,'r') as f:
            data = list(yaml.safe_load_all(f))

        data[0][self.parameter] -= 200
        with open(self.yaml_file_path, 'w') as file:
            yaml.dump_all(data, file, sort_keys=False)

        rospy.loginfo(f"Change parameter({self.parameter}) to {data[0][self.parameter]} in yaml file")

    def init_data_file():
        
        return
    
    def save_simulation_data():

        return

node = Tuner()
