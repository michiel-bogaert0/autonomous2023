#! /usr/bin/python3


from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import rospy
from aggregation.aggr_plus import plus_aggr
from aggregation.aggregation import Aggregation
from std_msgs.msg import Float32


class plotListener:
    """
    ROSNode for plotting purposes
    subscribes to the right topic and calls the right
    aggregation function
    result is published on /output

    """

    def __init__(self):
        rospy.init_node("plotlistener")

        # get the right aggregation function
        self.agg_function = rospy.get_param("~aggregation_function", "")
        self.aggregators = {"plus": plus_aggr()}

        # initialize aggregator class
        self.aggregator = self.aggregators[self.agg_function]

        # get names of the aggration function arguments
        args = self.aggregator.get_attrs(self.agg_function)

        # initialize dict and nr of updates
        self.arg_values = {arg: None for arg in args.keys()}
        self.nr_updates = 0

        # initialize subscribers
        for arg in args.keys():
            rospy.Subscriber(f"/input/{arg}", args[arg], self.update, arg)

        # initialize output publisher
        self.output_publisher = rospy.Publisher(
            "output/", self.aggregator.get_output_type(), queue_size=10
        )

        # if plot parameter is True, set the plotting function to be executed at the end
        self.plot = rospy.get_param("~plot", False)
        if self.plot:
            self.output_values = []
            self.output_dir = rospy.get_param("~savedir", "")
            rospy.on_shutdown(self.plot_values)

        rospy.spin()

    def plot_values(self):
        if self.plot:
            # call your custom plot function
            self.aggregator.plot(self.output_values, self.output_dir)

    def update(self, value: Any, input: str) -> None:
        """
        Internally updates the input values.
        If all values are updated, the aggregation function is called.
        Args:
            value: the input value of the agg function corresponding with argument "arg"
            input: the argument name of the aggregation function

        """
        # update values dict
        self.arg_values[input] = value

        # check: first updated value should correspond with the first input of the agg function
        if self.nr_updates == 0 and list(self.arg_values.values())[0] is None:
            return

        self.nr_updates += 1

        # if all values are updated, apply the aggregation function
        if self.nr_updates == len(self.arg_values.keys()):
            val = self.aggregator.apply(self.agg_function, self.arg_values)
            if self.plot:
                self.output_values.append(val)
            # reset nr of updates
            self.nr_updates = 0
            self.output_publisher.publish(val)


if __name__ == "__main__":
    try:
        pn = plotListener()
    except rospy.ROSInterruptException:
        pass
