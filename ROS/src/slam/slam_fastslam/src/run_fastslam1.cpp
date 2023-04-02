#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "fastslam1.hpp"
#include "node_fixture/node_fixture.hpp"

/**
 * FastSLAM 1.0 wrapper
 * 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "fastslam1_0");

  ros::NodeHandle n("~");

  float targetRate = n.param<float>("target_rate", 50.0);
  ros::Rate loop_rate(targetRate);

  bool doSynchronous = n.param<bool>("synchronous", true);

  slam::FastSLAM1 fastslam(n);

  ROS_INFO("FastSLAM1.0 running in %s mode!", doSynchronous ? "synchronous" : "asynchronous");
  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM FastSLAM1.0");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status",
                                  "running");


  // Spin the node
  while (ros::ok()) {
    // Keep the node alive
    ros::spinOnce();

    if (!doSynchronous) {
      fastslam.step();
    }
    
    loop_rate.sleep();
  }

  return 0;
}