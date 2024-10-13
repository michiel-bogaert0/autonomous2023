#include "camera.hpp"

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "perception");
  ros::NodeHandle n("~");

  // Create a CameraNode class object
  CameraNode perception(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spin();
  }

  return 0;
}