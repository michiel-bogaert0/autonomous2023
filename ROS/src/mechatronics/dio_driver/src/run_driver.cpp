#include "driver.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "dio_driver");
  ros::NodeHandle n("~");

  // Fetch the bank param
  int bank_id = n.param<int>("bank_id", 0);

  if (bank_id < 1 || bank_id > 2) {
    ROS_ERROR("Invalid bank_id: %d, or bank_id not set", bank_id);
    return -1;
  }

  auto driver = DIODriver(n, bank_id);
  driver.spin();
}