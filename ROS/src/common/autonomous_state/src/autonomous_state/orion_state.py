import rospy
from node_fixture.node_manager import NodeManager
from std_msgs.msg import Bool


class OrionState(NodeManager):
    def __init__(self):
        super().__init__("orion_state")
        rospy.Subscriber("/dio_driver_1/DI3", Bool, self.handle_asms)

        self.asms_on = False
        self.spin()

    def handle_asms(self, msg: Bool):
        self.asms_on = msg.data
