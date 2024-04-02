#include "iologik_driver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "iologik_driver");
  ros::NodeHandle n("~");

  // Create an InputReader object
  // InputReader reader("192.168.127.254", 502);
  iologik iologik(n);
  iologik.spin();

  return 0;
}