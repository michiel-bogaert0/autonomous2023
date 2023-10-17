#include "pathplanning.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "pathplanning");
  ros::NodeHandle n("~");

  std::string vis_namespace = n.param("vis_namespace", std::string("pathplanning_vis"));
  double vis_lifetime = n.param("vis_lifetime", 0.2);

  ros::Publisher vis_points = ros::Publisher();
  ros::Publisher vis_lines = ros::Publisher();


  // Create a LIDAR class object
  pathplanning::Pathplanning pathplanning(n);

  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();
  }

  return 0;
}
