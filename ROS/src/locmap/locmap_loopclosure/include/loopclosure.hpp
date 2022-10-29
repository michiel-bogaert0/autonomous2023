
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <std_msgs/Int64.h>
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