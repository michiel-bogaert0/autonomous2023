#include "node_fixture/node_fixture.hpp"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <ros/ros.h>

namespace node_fixture {

const char *ManagedNodeStateStrings[4] = {"unconfigured", "inactive", "active",
                                          "finalized"};

DiagnosticPublisher::DiagnosticPublisher(ros::NodeHandle &n)
    : DiagnosticPublisher(n, "default") {}

DiagnosticPublisher::DiagnosticPublisher(ros::NodeHandle &n, std::string name)
    : n(n), name(name) {
  diagnosticPublisher =
      n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 5);
}

void DiagnosticPublisher::publishDiagnostic(DiagnosticStatusEnum status,
                                            std::string name,
                                            std::string message) {
  diagnostic_msgs::DiagnosticArray diag_array;
  diagnostic_msgs::DiagnosticStatus diag_status;
  diag_status.level = status;
  diag_status.name = "[" + this->name + "] " + name;
  diag_status.message = message;
  diag_array.status.push_back(diag_status);

  diagnosticPublisher.publish(diag_array);
}
} // namespace node_fixture