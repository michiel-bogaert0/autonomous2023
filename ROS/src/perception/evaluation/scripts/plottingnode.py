#! /usr/bin/python3


import rospy
from std_msgs.msg import Float32
from aggregation import Aggregation
class plotListender:
    def __init__(self):
        rospy.init_node("plotlistener")
        self.agg_function = rospy.get_param("~aggregation_function","")
        rospy.logwarn(self.agg_function)
        self.aggregator = Aggregation()
        args = self.aggregator.get_attrs(self.agg_function).args[1:]
        self.arg_values  = {arg: None for arg in args}
        self.nr_updates = 0
        for arg in args:
            sub = rospy.Subscriber(f"/input/{arg}",Float32,self.update,arg)
        rospy.logwarn(args)
        rospy.spin()

    def update(self,value,input):
        rospy.loginfo(input)
        self.arg_values[input] = value.data
        if self.nr_updates == 0 and list(self.arg_values.values())[0] is None:
            return
        self.nr_updates +=1
        if self.nr_updates == len(self.arg_values.keys()):
            val= self.aggregator.apply(self.agg_function,self.arg_values)
            rospy.logwarn(val)
            self.nr_updates = 0



if __name__ == "__main__":
    try:
        pn = plotListender()
    except rospy.ROSInterruptException:
        pass
