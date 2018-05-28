#ifndef MOVE_SERVICE_
#define MOVE_SERVICE_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "myGetMap.h"

// extern tf::StampedTransform robotPosInMapFrame;

// void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate);
double getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate);
double getDistance(double x1,double y1,double x2,double y2);

tf::StampedTransform getRobotPosInMapFrame();




#endif
