#include "loopclosure.hpp"
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "loopclosure");
  ros::NodeHandle n("~");

  slam::LoopClosure loopClosure(n);

  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM LC");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status", "running");

  loopClosure.spin();

  return 0;
}
