#include "pathplanning.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n("~");

  // Create a pathplanning class object
  pathplanning::Pathplanning pathplanning(n);
  pathplanning.initialize();
  pathplanning.spin();

  return 0;
}
