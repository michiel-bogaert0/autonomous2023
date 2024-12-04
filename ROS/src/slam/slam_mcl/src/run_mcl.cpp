#include "mcl.hpp"
#include <ros/ros.h>

/**
 * MCL wrapper
 *
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "slam_mcl");

  ros::NodeHandle n("~");

  float targetRate = n.param<float>("target_rate", 50.0);
  ros::Rate loop_rate(targetRate);

  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM MCL");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status", "running");

  slam::MCL mcl(n);
  mcl.initialize();
  mcl.spin();

  return 0;
}