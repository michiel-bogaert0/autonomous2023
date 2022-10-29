#include "loopclosure.hpp"
#include "std_msgs/UInt16.h"


namespace slam
{
    LoopClosure::LoopClosure(ros::NodeHandle &n)
    {
        observeEndPoint = n.subscribe("/input/odometry",0, &LoopClosure::CheckWhenLoopIsClosed, this);
        this->loopClosedPublisher = n.advertise<std_msgs::UInt16>("/output/loopClosure",5);
        reset_service = n.advertiseService("/reset_closure", &slam::LoopClosure::ResetClosure, this);
    }
    
    void LoopClosure::CheckWhenLoopIsClosed(const nav_msgs::Odometry::ConstPtr& msg)
    {
        Point distanceVector = Point{(float)msg->pose.pose.position.x - startPosition.x, (float)msg->pose.pose.position.y - startPosition.y};
        if(checkDistanceGoingUpWhenInRange)
        {
            if(DotProduct(directionWhenGoingInRange, distanceVector) < 0)
            {
                amountOfLaps++;
                std_msgs::UInt16 msg1;
                msg1.data = amountOfLaps;
                this->loopClosedPublisher.publish(msg1);
                ResetLoop();
                return;
            }
            return;
        }

        if(startPosition.x == FLT_MAX || startPosition.y == FLT_MAX)
        {
            startPosition.x = msg->pose.pose.position.x;
            startPosition.y = msg->pose.pose.position.y;
        }
        float currentDistanceSquare = DotProduct(distanceVector, distanceVector);
        if(isinf(currentDistanceSquare)) return;
        if(!doNotCheckDistance)
        {
            if(maxDistanceSquare < currentDistanceSquare)
            {
                doNotCheckDistance = true;
            }
            //calculate distance if distance is less then maxdistance you can check if end == start
        }
        else
        {
            if(currentDistanceSquare < minDistanceForClose * minDistanceForClose)
            {
                checkDistanceGoingUpWhenInRange = true;
                directionWhenGoingInRange = distanceVector;
            }
        }    
    }
    float LoopClosure::DotProduct(const Point& a,const Point& b)
    {
        return a.x * b.x + a.y * b.y;
    }
    void LoopClosure::ResetLoop()
    {
        doNotCheckDistance=false;
        checkDistanceGoingUpWhenInRange = false;
    }
    
    bool LoopClosure::ResetClosure(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
    {
        ROS_INFO("reset closure");
        startPosition = slam::Point();
        doNotCheckDistance = false;
        checkDistanceGoingUpWhenInRange = false;
        directionWhenGoingInRange = slam::Point();
        amountOfLaps = 0;
        return true;
    }
}