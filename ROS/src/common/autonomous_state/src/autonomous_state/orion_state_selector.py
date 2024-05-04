import rospy
from node_fixture.fixture import NodeManagingStatesEnum, OrionStatesEnum
from node_fixture.node_manager import NodeManager
from std_msgs.msg import Bool


class OrionStateSelector(NodeManager):
    def __init__(self):
        super().__init__("orion_state", NodeManagingStatesEnum.ACTIVE)
        rospy.Subscriber("/dio_driver_1/DI3", Bool, self.handle_asms)
        self.state = OrionStatesEnum.IDLE
        self.bypass_status = None
        self.spin()

    def handle_bypass(self, msg: Bool):
        if msg.data != self.bypass_status:
            self.bypass_status = msg.data
            if self.bypass_status:
                self.activate_nodes(OrionStatesEnum.DRIVERLESS, self.state)
                self.state = OrionStatesEnum.DRIVERLESS
            else:
                self.activate_nodes(OrionStatesEnum.MANUAL, self.state)
                self.state = OrionStatesEnum.MANUAL
