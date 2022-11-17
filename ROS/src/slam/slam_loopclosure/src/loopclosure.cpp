#include "loopclosure.hpp"
#include "std_msgs/UInt16.h"


namespace slam
{
    LoopClosure::LoopClosure(ros::NodeHandle &n)
    {
        observeEndPoint = n.subscribe("/input/odometry",0, &LoopClosure::CheckWhenLoopIsClosed, this);
        loopClosedPublisher = n.advertise<std_msgs::UInt16>("/output/loopClosure",5);
        reset_service = n.advertiseService("/reset_closure", &slam::LoopClosure::ResetClosure, this);
    }
    
    void LoopClosure::CheckWhenLoopIsClosed(const nav_msgs::Odometry::ConstPtr& msg)
    {
        //calculate the vector from startpos to current pos
        Point distanceVector = Point{(float)msg->pose.pose.position.x - startPosition.x, (float)msg->pose.pose.position.y - startPosition.y};
        //when the currentpos is close enough to startpos (after racing the round)
        if(checkDistanceGoingUpWhenInRange)
        {
            //check if the car is in front of the startposition
            if(DotProduct(directionWhenGoingInRange, distanceVector) < 0)
            {
                amountOfLaps++;
                std_msgs::UInt16 msg1;
                msg1.data = amountOfLaps;
                loopClosedPublisher.publish(msg1);
                ResetLoop();
                return;
            }
            return;
        }
        //init startposition(just te be sure that it isn't {0,0})
        if(startPosition.x == FLT_MAX || startPosition.y == FLT_MAX)
        {
            startPosition.x = msg->pose.pose.position.x;
            startPosition.y = msg->pose.pose.position.y;
        }
        //calculate distance 
        float currentDistanceSquare = DotProduct(distanceVector, distanceVector);
        if(isinf(currentDistanceSquare)) return;
        //if the car is outside the outer range -> enable to check if the car is going towards the start position
        if(!doNotCheckDistance)
        {
            if(maxDistanceSquare < currentDistanceSquare)
            {
                doNotCheckDistance = true;
            }
        }
        else
        {
            //if the car is close enough to the startposition
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
        startPosition = slam::Point();
        doNotCheckDistance = false;
        checkDistanceGoingUpWhenInRange = false;
        directionWhenGoingInRange = slam::Point();
        amountOfLaps = 0;
        return true;
    }
}