#include "loopclosure.hpp"
#include <iostream>
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Bool.h"

namespace slam
{
    LoopClosure::LoopClosure(ros::NodeHandle &n)
    {
        observeEndPoint = n.subscribe("/input/odometry",0, &LoopClosure::CheckWhenLoopIsClosed, this);
        this->loopClosedPublisher = n.advertise<std_msgs::Bool>("/output/loopClosure",5);
     }
    
    void LoopClosure::CheckWhenLoopIsClosed(const nav_msgs::Odometry::ConstPtr& msg)
    {
        Point distanceVector = Point{(float)msg->pose.pose.position.x - startPosition.x, (float)msg->pose.pose.position.y - startPosition.y};
        if(checkDistanceGoingUpWhenInRange)
        {
            if(DotProduct(directionWhenGoingInRange, distanceVector) < 0)
            {
                observeEndPoint.shutdown();
                std_msgs::Bool msg1;
                msg1.data = true;
                this->loopClosedPublisher.publish(msg1);
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

}