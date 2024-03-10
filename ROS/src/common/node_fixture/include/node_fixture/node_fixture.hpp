#ifndef NODE_FIXTURE_H
#define NODE_FIXTURE_H
#include <ros/ros.h>

namespace node_fixture {

/**
 * \brief Enumerator for the managed node state machine
 */
enum ManagedNodeState { Unconfigured, Inactive, Active, Finalized };

/**
 * \brief Strings corresponding to the state machine enumerator
 */
extern const char *ManagedNodeStateStrings[4];

enum DiagnosticStatusEnum { OK, WARN, ERROR, STALE };

class DiagnosticPublisher {
public:
  explicit DiagnosticPublisher(ros::NodeHandle &n);
  DiagnosticPublisher(ros::NodeHandle &n, std::string name);

  void publishDiagnostic(DiagnosticStatusEnum status, std::string name,
                         std::string message);

private:
  std::string name;
  ros::NodeHandle &n;

  ros::Publisher diagnosticPublisher;
};
} // namespace node_fixture
#endif