#include "boundary_estimation.hpp"
#include "pathplanning.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n("~");

  // Create a pathplanning class object
  pathplanning::Pathplanning pathplanning(n);
  pathplanning::BoundaryEstimation boundary_estimation(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}
