#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>


#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"

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
        private:
            void CheckWhenLoopIsClosed(const nav_msgs::Odometry::ConstPtr& msg);
            float DotProduct(const Point& a, const Point& b);
            ros::Subscriber observeEndPoint;
            ros::Publisher loopClosedPublisher;

            Point startPosition = Point();
            float maxDistanceSquare = 1.f;
            bool doNotCheckDistance = false;
            bool checkDistanceGoingUpWhenInRange = false;
            Point directionWhenGoingInRange = Point();
            float distanceWhenLooking = 2.f;
            float minDistanceForClose = 0.5f;
    };   
}