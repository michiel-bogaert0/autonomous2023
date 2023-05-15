#include "loopclosure.hpp"
#include "std_msgs/UInt16.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "node_fixture/node_fixture.hpp"

namespace slam
{
    LoopClosure::LoopClosure(ros::NodeHandle &n) : tfListener(tfBuffer),
                                                   n(n),
                                                   base_link_frame(n.param<std::string>("base_link_frame", "ugr/car_base_link")),
                                                   world_frame(n.param<std::string>("world_frame", "ugr/slam_map"))
    {
        diagnosticPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(new node_fixture::DiagnosticPublisher(n, "SLAM LC"));
        loopClosedPublisher = n.advertise<std_msgs::UInt16>("/output/loopClosure", 5, true);
        reset_service = n.advertiseService("/reset_closure", &slam::LoopClosure::handleResetClosureService, this);

        ResetClosure();

        ROS_INFO("Loop closure detection started!");
    }

    void LoopClosure::CheckWhenLoopIsClosed()
    {
        if (this->latestTime - ros::Time::now().toSec() > 0.5 && this->latestTime > 0.0)
        {
            ROS_WARN("Time went backwards! Resetting...");
            ResetClosure();
            return;
        }
        this->latestTime = ros::Time::now().toSec();

        geometry_msgs::TransformStamped car_pose;
        try
        {
            car_pose = tfBuffer.lookupTransform(this->world_frame, this->base_link_frame, ros::Time(0), ros::Duration(0.1));
        }
        catch (const std::exception e)

        {
            ROS_ERROR("Car pose lookup failed: %s", e.what());
            return;
        }

        // calculate the vector from startpos to current pos
        geometry_msgs::Point distanceVectorToStart; 
        distanceVectorToStart.x =(float)car_pose.transform.translation.x - startPosition.x;
        distanceVectorToStart.y =(float)car_pose.transform.translation.y - startPosition.y;
        distanceVectorToStart.z =(float)car_pose.transform.translation.z - startPosition.z;

        geometry_msgs::Point distanceVectorToTarget;
        distanceVectorToStart.x =(float)car_pose.transform.translation.x - targetPoint.x; 
        distanceVectorToStart.y =(float)car_pose.transform.translation.y - targetPoint.y; 
        distanceVectorToStart.z =(float)car_pose.transform.translation.z - targetPoint.z;
      
        geometry_msgs::Point distanceVectorToFinish;
        distanceVectorToStart.x =(float)car_pose.transform.translation.x - finishPoint.x; 
        distanceVectorToStart.y =(float)car_pose.transform.translation.y - finishPoint.y; 
        distanceVectorToStart.z =(float)car_pose.transform.translation.z - finishPoint.z;

        
        // when the currentpos is close enough to startpos (after racing the round)
        if (checkDistanceGoingUpWhenInRange)
        {
            // check if the car is in front of the startposition
            // determine if it is the target/finish or start
            if(DotProduct(directionWhenGoingInRange, distanceVectorToStart) < 0 && !isLoopingTarget)
            {
                amountOfLaps++;
                this->publish();
                ResetLoop();
            }
            else if(DotProduct(directionWhenGoingInRange, distanceVectorToTarget) < 0)
            {
                amountOfLaps++;
                isLoopingTarget = true;
                this->publish();
                ResetLoop();
            }
            else if (DotProduct(directionWhenGoingInRange, distanceVectorToFinish) < 0 && isLoopingTarget)
            {
                amountOfLaps++;
                this->publish();
                return;
            }
            return;
        }
        // init startposition(just te be sure that it isn't {0,0})
        if (startPosition.x == FLT_MAX || startPosition.y == FLT_MAX || startPosition.z == FLT_MAX)
        {
            startPosition.x = car_pose.transform.translation.x;
            startPosition.y = car_pose.transform.translation.y;
            startPosition.z = car_pose.transform.translation.z;
            if(finishPoint.x == FLT_MAX || finishPoint.y == FLT_MAX || finishPoint.z == FLT_MAX)
                finishPoint = startPosition;
        }
        // calculate distance
        float currentDistanceSquare = 0;
        if(isLoopingTarget) currentDistanceSquare = DotProduct(distanceVectorToStart, distanceVectorToTarget);
        else currentDistanceSquare = DotProduct(distanceVectorToStart, distanceVectorToStart);
        //ROS_INFO("%f",currentDistanceSquare);
        // if the car is outside the outer range -> enable to check if the car is going towards the start position
        if (!doNotCheckDistance)
        { 
            float currentDistanceSquare = 0;
            if(isLoopingTarget) currentDistanceSquare = DotProduct(distanceVectorToStart, distanceVectorToTarget);
            else currentDistanceSquare = DotProduct(distanceVectorToStart, distanceVectorToStart);
            ROS_INFO("%f",currentDistanceSquare);
            if (isinf(currentDistanceSquare))
                return;
            if (maxDistanceSquare < currentDistanceSquare)
            {
                doNotCheckDistance = true;
            }
        }
        else
        { 
            float currentDistanceSquareFinish = DotProduct(distanceVectorToFinish,distanceVectorToFinish);
            // if the car is close enough to the finishposition
            if (currentDistanceSquareFinish < minDistanceForClose * minDistanceForClose)
            {
                checkDistanceGoingUpWhenInRange = true;
                directionWhenGoingInRange = distanceVectorToStart;
            }
        }
    }
    float LoopClosure::DotProduct(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    void LoopClosure::ResetLoop()
    {
        doNotCheckDistance = false;
        checkDistanceGoingUpWhenInRange = false;
    }

    void LoopClosure::ResetClosure()
    {
        // Fetch startposition from /tf
        ROS_INFO("Resetting counter. Waiting for initial pose...");
        while (!tfBuffer.canTransform(this->world_frame, this->base_link_frame, ros::Time(0)))
        {
            // Do nothing, except for when process needs to quit
            if (!ros::ok())
            {
                return;
            }
        }
        ROS_INFO("Got initial pose!");

        geometry_msgs::TransformStamped initial_car_pose = tfBuffer.lookupTransform(this->world_frame, this->base_link_frame, ros::Time(0), ros::Duration(0.1));
        startPosition = geometry_msgs::Point();
        startPosition.x = initial_car_pose.transform.translation.x;
        startPosition.y = initial_car_pose.transform.translation.y;
        startPosition.z = initial_car_pose.transform.translation.z;
        ROS_INFO("%f", startPosition.x);
        ROS_INFO("%f", startPosition.y);
        ROS_INFO("%f", startPosition.z);

        doNotCheckDistance = false;
        checkDistanceGoingUpWhenInRange = false;
        directionWhenGoingInRange = geometry_msgs::Point();
        amountOfLaps = 0;

        this->latestTime = ros::Time::now().toSec();

        this->publish();
    }

    void LoopClosure::publish()
    {
        std_msgs::UInt16 msg1;
        msg1.data = amountOfLaps;
        loopClosedPublisher.publish(msg1);
        diagnosticPublisher->publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK, "Loop Closure count",
                                "#laps: " + std::to_string(amountOfLaps));
    }

    bool LoopClosure::handleResetClosureService(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        this->ResetClosure();
        return true;
    }
    bool LoopClosure::handleAdjustFinishLine(slam_loopclosure::FinishPoint::Request   &request, std_srvs::Empty::Response &response)
    {
        this->finishPoint = request.point;
        return true;
    }
    bool LoopClosure::handleAdjustTargetPoint(slam_loopclosure::FinishPoint::Request &request, std_srvs::Empty::Response &response)
    {
        this->targetPoint = request.point;
        return true;
    }
}