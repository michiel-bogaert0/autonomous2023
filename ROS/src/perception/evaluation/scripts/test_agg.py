#! /usr/bin/python3


import rospy
from std_msgs.msg import Float32
class plotPublisher:
    """
    ROSNode to test the aggregation publisher
    consists of two publishers
    """
    def __init__(self):
        rospy.init_node("plotpub")
        self.val = 0
        self.x_pub = rospy.Publisher("/output/x",Float32,queue_size=10)
        self.y_pub = rospy.Publisher("/output/y",Float32,queue_size=10)
        self.spin()

    def spin(self)-> None:
        """
        Update function
        """
        while not rospy.is_shutdown():
            #continuously publish increasing values on the two topics
            self.val +=1
            self.x_pub.publish(Float32(self.val))
            rospy.sleep(0.1)
            self.y_pub.publish(Float32(self.val))
            rospy.sleep(2)



if __name__ == "__main__":
    try:
        pn = plotPublisher()
    except rospy.ROSInterruptException:
        pass
