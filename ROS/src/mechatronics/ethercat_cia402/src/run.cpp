#include "ethercat_driver.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "ethercat_cia402");
  ros::NodeHandle n("~");

  auto driver = ECatDriver(n);
  driver.spin();
}