import rospy
from node_fixture.fixture import NodeManagingStatesEnum, OrionStatesEnum
from node_fixture.node_manager import NodeManager
from std_msgs.msg import Bool


class OrionState(NodeManager):
    def __init__(self):
        super().__init__("orion_state", NodeManagingStatesEnum.ACTIVE)
        rospy.Subscriber("/dio_driver_1/DI3", Bool, self.handle_asms)
        self.state = OrionStatesEnum.IDLE
        self.asms_on = False
        self.spin()

    def handle_asms(self, msg: Bool):
        self.asms_on = msg.data
        if self.asms_on:
            self.activate_nodes(OrionStatesEnum.DRIVERLESS, self.state)
            self.state = OrionStatesEnum.DRIVERLESS
        else:
            self.activate_nodes(OrionStatesEnum.MANUAL, self.state)
            self.state = OrionStatesEnum.MANUAL
