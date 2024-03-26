#include "graphslam.hpp"
#include "node_fixture/node_fixture.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "graphslam");

  ros::NodeHandle n("~");

  bool doSynchronous = n.param<bool>("synchronous", true);

  slam::GraphSLAM graphslam(n);

  ROS_INFO("Graphslam running in %s mode!",
           doSynchronous ? "synchronous" : "asynchronous");
  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM GraphSLAM");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status", "running");

  graphslam.spin();

  return 0;
}