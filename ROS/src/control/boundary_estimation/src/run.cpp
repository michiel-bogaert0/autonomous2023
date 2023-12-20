#include "boundary_estimation.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "boundary_estimation");
  ros::NodeHandle n("~");

  // Create a pathplanning class object
  boundaryestimation::BoundaryEstimation boundary_estimation(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spin();
  }

  return 0;
}
