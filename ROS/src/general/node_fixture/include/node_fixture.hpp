#include <ros/ros.h>

namespace node_fixture {
    enum DiagnosticStatusEnum { OK, WARN, ERROR, STALE };

    class DiagnosticPublisher {
        public:
            DiagnosticPublisher(std::string name);

            void publishDiagnostic(DiagnosticStatusEnum status, std::string name, std::string message);

        private:
            ros::Publisher diagnosticPublisher;

            std::string name;

    };
}