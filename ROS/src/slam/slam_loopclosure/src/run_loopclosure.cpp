#include <ros/ros.h>
#include "loopclosure.hpp"
#include <std_srvs/Empty.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "loopClosure_1");
    ros::NodeHandle n ("~");
    slam::LoopClosure loopClosure(n);

    while (ros::ok()) {
         // Keep the node alive
        ros::spinOnce();
    }

    return 0;

}
