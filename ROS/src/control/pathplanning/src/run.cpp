#include "pathplanning.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n("~");

  // Create a LIDAR class object
  pathplanning::Pathplanning pathplanning(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}
