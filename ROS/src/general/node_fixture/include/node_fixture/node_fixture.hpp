#ifndef NODE_FIXTURE_H
#define NODE_FIXTURE_H
#include <ros/ros.h>

namespace node_fixture {
    enum DiagnosticStatusEnum { OK, WARN, ERROR, STALE };

    class DiagnosticPublisher {
        public:
            DiagnosticPublisher(ros::NodeHandle &n);
            DiagnosticPublisher(ros::NodeHandle &n, std::string name);

            void publishDiagnostic(DiagnosticStatusEnum status, std::string name, std::string message);

        private:
            std::string name;
            ros::NodeHandle &n;

            ros::Publisher diagnosticPublisher;
    };
}
#endif