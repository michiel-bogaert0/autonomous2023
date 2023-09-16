#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include "node_fixture/node_fixture.hpp"
#include "slam_loopclosure/FinishPoint.h"

namespace slam
{
    class LoopClosure
    {
    public:
        LoopClosure(ros::NodeHandle &n);

        bool handleResetClosureService(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        bool handleAdjustFinishLine(slam_loopclosure::FinishPoint::Request &request, slam_loopclosure::FinishPoint::Response &response);
        bool handleAdjustTargetPoint(slam_loopclosure::FinishPoint::Request &request, slam_loopclosure::FinishPoint::Response &response);
        
        void CheckWhenLoopIsClosed();

    private:
        void ResetClosure();
        float DotProduct(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
        void ResetLoop();
        void publish();


        ros::NodeHandle &n;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        ros::Publisher loopClosedPublisher;
        ros::ServiceServer reset_service;
        ros::ServiceServer adjust_finish;
        ros::ServiceServer adjust_targetpoint;

        std::unique_ptr<node_fixture::DiagnosticPublisher> diagnosticPublisher;

        std::string base_link_frame;
        std::string world_frame;

        geometry_msgs::Point startPosition = geometry_msgs::Point();
        geometry_msgs::Point finishPoint = geometry_msgs::Point();
        geometry_msgs::Point targetPoint = geometry_msgs::Point();
        bool isLoopingTarget = false;
        float maxDistanceSquare = 9.f;
        double latestTime;
        bool doNotCheckDistance = false;
        bool checkDistanceGoingUpWhenInRange = false;
        bool runLoopClosureDetection = true;
        geometry_msgs::Point directionWhenGoingInRange = geometry_msgs::Point();
        float minDistanceForClose = 2.0f;
        int amountOfLaps = 0;
    };
}