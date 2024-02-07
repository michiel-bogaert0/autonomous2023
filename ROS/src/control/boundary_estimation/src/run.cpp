#include "boundary_estimation.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "boundary_estimation");
  ros::NodeHandle n("~");

  // Create a boundary estimation class object
  boundaryestimation::BoundaryEstimation boundary_estimation(n);

  boundary_estimation.spin();

  return 0;
}
