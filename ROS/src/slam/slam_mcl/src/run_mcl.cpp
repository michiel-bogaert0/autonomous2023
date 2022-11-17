#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mcl.hpp"

/**
 * FastSLAM 1.0 wrapper
 * 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "slam_mcl");

  ros::NodeHandle n("~");

  slam::MCL mcl(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}