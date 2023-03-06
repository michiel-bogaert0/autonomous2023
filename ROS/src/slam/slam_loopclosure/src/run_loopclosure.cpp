#include <ros/ros.h>
#include "loopclosure.hpp"
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loopclosure");
    ros::NodeHandle n("~");

    float targetRate = n.param<float>("target_rate", 50.0);
    ros::Rate loop_rate(targetRate);

    slam::LoopClosure loopClosure(n);

    while (ros::ok())
    {
        // Keep the node alive
        ros::spinOnce();
        loopClosure.CheckWhenLoopIsClosed();
        loop_rate.sleep();
    }

    return 0;
}
