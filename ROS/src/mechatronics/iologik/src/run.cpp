#include "iologik_driver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "iologik_aio");
  ros::NodeHandle n("~");

  iologik iologik(n);
  iologik.spin();

  return 0;
}