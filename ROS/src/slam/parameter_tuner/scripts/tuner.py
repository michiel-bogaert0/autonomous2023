#! /usr/bin/python3
import rospy
from ugr_msgs.msg import State

from node_fixture.node_fixture import AutonomousStatesEnum, StateMachineScopeEnum
from launcher import Launcher

from time import sleep

class Tuner:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param('~map', "circle_R15") + ".yaml"
        self.mission = rospy.get_param('~mission', "trackdrive")
        self.max_time = rospy.Duration.from_sec(rospy.get_param('~max_time', 60))

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = Launcher(rospy.get_param("~logging", True))
        
        rospy.Subscriber("/state", State, self.state_callback)

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

    def new_simulation(self):
        self.launcher.shutdown_car()
        self.launcher.shutdown_simulation()

        self.state = AutonomousStatesEnum.ASOFF
        sleep(2)
        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)
        self.start = rospy.Time.now()

node = Tuner()
