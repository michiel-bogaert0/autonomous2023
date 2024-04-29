#include "ethercat_driver.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "ethercat_driver");
  ros::NodeHandle n("~");

  auto driver = ECatDriver(n);
  driver.spin();
}