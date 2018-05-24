#ifndef MOVE_SERVICE_
#define MOVE_SERVICE_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "myGetMap.h"
// void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate);
void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate, double start_x, double start_y);
double getDistance(double x1,double y1,double x2,double y2);

tf::StampedTransform getRobotPosInMapFrame();
#endif
