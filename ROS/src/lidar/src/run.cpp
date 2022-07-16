#include <ros/ros.h>
#include "lidar.hpp"

int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    int node_rate;
    n.param<int>("~rate", node_rate, 10);
    ros::Rate loop_rate(node_rate);

    // Create a LIDAR class object
    Lidar lidar(n);

    // Spin the node
    while (ros::ok())
    {
        // Keep the node alive
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
