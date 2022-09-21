import rospy
from std_msgs.msg import Empty

class Controller: 

    def __init__(self) -> None:
        rospy.init_node("slam_controller")

        rospy.Subscriber("/A", Empty, self.handle_A)
        rospy.Subscriber("/B", Empty, self.handle_B)

        self.count = 0

        rospy.spin()
      
    def handle_A(self, msg):
      print("Received A")
      self.count += 1

    def handle_B(self, msg):
      print("Received B")
      while True:
        print(self.count)
        
    

node = Controller()
