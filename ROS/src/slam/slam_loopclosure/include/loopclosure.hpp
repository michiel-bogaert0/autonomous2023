#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_srvs/Empty.h>

namespace slam
{
    struct Point{
        float x = FLT_MAX;
        float y = FLT_MAX;
    };
    class LoopClosure
    {
        public:

            LoopClosure(ros::NodeHandle &n);

            bool ResetClosure(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        private:
            void CheckWhenLoopIsClosed(const nav_msgs::Odometry::ConstPtr& msg);
            float DotProduct(const Point& a, const Point& b);
            void ResetLoop();

            ros::Subscriber observeEndPoint;
            ros::Publisher loopClosedPublisher;
            ros::ServiceServer reset_service;

            Point startPosition = Point();
            float maxDistanceSquare = 1.f;
            bool doNotCheckDistance = false;
            bool checkDistanceGoingUpWhenInRange = false;
            Point directionWhenGoingInRange = Point();
            float minDistanceForClose = 0.5f;
            int amountOfLaps = 0;
    };   
}