#include "lap_counter.hpp"
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lap_counter");
  ros::NodeHandle n("~");

  slam::LapCounter lapCounter(n);

  node_fixture::DiagnosticPublisher diagPublisher(n, "SLAM LC");
  diagPublisher.publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                  "Status", "running");

  lapCounter.spin();

  return 0;
}
