#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "node_fixture/node_fixture.hpp"

namespace slam
{
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

        ros::NodeHandle &n;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        ros::Publisher loopClosedPublisher;
        ros::ServiceServer reset_service;

        std::unique_ptr<node_fixture::DiagnosticPublisher> diagnosticPublisher;

        std::string base_link_frame;
        std::string world_frame;

        Point startPosition = Point();
        float maxDistanceSquare = 4.f;
        double latestTime;
        bool doNotCheckDistance = false;
        bool checkDistanceGoingUpWhenInRange = false;
        Point directionWhenGoingInRange = Point();
        float minDistanceForClose = 2.0f;
        int amountOfLaps = 0;
    };
}