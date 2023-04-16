#include <ros/ros.h>
#include "mcl.hpp"

/**
 * MCL wrapper
 *
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "slam_mcl");

  ros::NodeHandle n("~");

  slam::MCL mcl(n);

  float targetRate = n.param<float>("target_rate", 50.0);
  ros::Rate loop_rate(targetRate);

  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM MCL");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status",
                                  "running");

  while (ros::ok())
  {
    ros::spinOnce();
    mcl.step();
    loop_rate.sleep();
  }

  return 0;
}