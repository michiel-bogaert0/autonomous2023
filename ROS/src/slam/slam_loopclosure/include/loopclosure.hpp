#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>

namespace slam
{
    enum DiagnosticStatusEnum { OK, WARN, ERROR, STALE };

    struct Point
    {
        float x = FLT_MAX;
        float y = FLT_MAX;
    };
    class LoopClosure
    {
    public:
        LoopClosure(ros::NodeHandle &n);

        bool handleResetClosureService(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void CheckWhenLoopIsClosed();

    private:
        void ResetClosure();
        float DotProduct(const Point &a, const Point &b);
        void ResetLoop();
        void publish();
        void publishDiagnostic(DiagnosticStatusEnum status, std::string name, std::string message);

        ros::NodeHandle &n;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        ros::Publisher diagnosticPublisher;
        ros::Publisher loopClosedPublisher;
        ros::ServiceServer reset_service;

        std::string base_link_frame;
        std::string world_frame;

        Point startPosition = Point();
        float maxDistanceSquare = 1.f;
        double latestTime;
        bool doNotCheckDistance = false;
        bool checkDistanceGoingUpWhenInRange = false;
        Point directionWhenGoingInRange = Point();
        float minDistanceForClose = 0.5f;
        int amountOfLaps = 0;
    };
}