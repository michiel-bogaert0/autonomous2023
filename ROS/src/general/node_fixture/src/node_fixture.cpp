#include "node_fixture.hpp"
#include <ros/ros.h>
#include "diagnostic_msgs/DiagnosticArray.h"

namespace node_fixture {

    DiagnosticPublisher::DiagnosticPublisher(std::string name) {
        this->name = name;
    }

    void DiagnosticPublisher::publishDiagnostic(DiagnosticStatusEnum status, std::string name, std::string message) {
        diagnostic_msgs::DiagnosticArray diag_array;
        diagnostic_msgs::DiagnosticStatus diag_status;
        diag_status.level = status;
        diag_status.name = name;
        diag_status.message = message;
        diag_array.status.push_back(diag_status);

        diagnosticPublisher.publish(diag_array);
    }
}