#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "fastslam1.hpp"

/**
 * FastSLAM 1.0 wrapper
 * 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "fastslam1_0");

  ros::NodeHandle n("~");

  slam::FastSLAM1 fastslam(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}