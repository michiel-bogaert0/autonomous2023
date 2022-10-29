#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "loopclosure.hpp"
#include <std_srvs/SetBool.h>
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
