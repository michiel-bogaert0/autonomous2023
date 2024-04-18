#include "driver.hpp"
#include <filesystem>
#include <iostream>
namespace fs = std::filesystem;

int main(int argc, char **argv) {

  ros::init(argc, argv, "dio_driver");
  ros::NodeHandle n("~");

  auto driver = DIODriver(n);
  driver.spin();
}