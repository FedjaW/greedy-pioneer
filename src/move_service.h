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
void rotate(ros::NodeHandle &nh, double rotation_angle);
void sendGoal(double x, double y,double orientation);
// bool isObstacleInViewField(int x0, int y0, int x1, int y1);
bool isObstacleInViewField(ros::NodeHandle &nh, const nav_msgs::OccupancyGrid& map, int x0, int y0, int x1, int y1);




#endif
