#include "lidar.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "lidar");
  ros::NodeHandle n("~");

  // Create a LIDAR class object
  ns_lidar::Lidar lidar(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}
