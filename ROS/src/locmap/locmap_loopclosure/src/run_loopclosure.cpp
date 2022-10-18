#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "loopclosure.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "loopClosure_1");
    ros::NodeHandle n ("~");
    slam::LoopClosure loopClosure(n);
    
    // Spin the node
    while (ros::ok()) {
         // Keep the node alive
        ros::spinOnce();
    }

    return 0;
}