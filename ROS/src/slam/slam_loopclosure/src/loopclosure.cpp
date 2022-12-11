#include "loopclosure.hpp"
#include "std_msgs/UInt16.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>

namespace slam
{
    LoopClosure::LoopClosure(ros::NodeHandle &n) : tfListener(tfBuffer),
                                                   n(n),
                                                   base_link_frame(n.param<std::string>("base_link_frame", "ugr/car_base_link")),
                                                   world_frame(n.param<std::string>("world_frame", "ugr/slam_map"))
    {
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
        Point distanceVector = Point{(float)car_pose.transform.translation.x - startPosition.x, (float)car_pose.transform.translation.y - startPosition.y};
        // when the currentpos is close enough to startpos (after racing the round)
        if (checkDistanceGoingUpWhenInRange)
        {
            // check if the car is in front of the startposition
            if (DotProduct(directionWhenGoingInRange, distanceVector) < 0)
            {
                amountOfLaps++;
                this->publish();
                ResetLoop();
                return;
            }
            return;
        }
        // init startposition(just te be sure that it isn't {0,0})
        if (startPosition.x == FLT_MAX || startPosition.y == FLT_MAX)
        {
            startPosition.x = car_pose.transform.translation.x;
            startPosition.y = car_pose.transform.translation.y;
        }
        // calculate distance
        float currentDistanceSquare = DotProduct(distanceVector, distanceVector);
        if (isinf(currentDistanceSquare))
            return;
        // if the car is outside the outer range -> enable to check if the car is going towards the start position
        if (!doNotCheckDistance)
        {
            if (maxDistanceSquare < currentDistanceSquare)
            {
                doNotCheckDistance = true;
            }
        }
        else
        {
            // if the car is close enough to the startposition
            if (currentDistanceSquare < minDistanceForClose * minDistanceForClose)
            {
                checkDistanceGoingUpWhenInRange = true;
                directionWhenGoingInRange = distanceVector;
            }
        }
    }
    float LoopClosure::DotProduct(const Point &a, const Point &b)
    {
        return a.x * b.x + a.y * b.y;
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
        startPosition = slam::Point();
        startPosition.x = initial_car_pose.transform.translation.x;
        startPosition.y = initial_car_pose.transform.translation.y;

        doNotCheckDistance = false;
        checkDistanceGoingUpWhenInRange = false;
        directionWhenGoingInRange = slam::Point();
        amountOfLaps = 0;

        this->latestTime = ros::Time::now().toSec();

        this->publish();
    }

    void LoopClosure::publish()
    {
        std_msgs::UInt16 msg1;
        msg1.data = amountOfLaps;
        loopClosedPublisher.publish(msg1);
    }

    bool LoopClosure::handleResetClosureService(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        this->ResetClosure();
        return true;
    }
}