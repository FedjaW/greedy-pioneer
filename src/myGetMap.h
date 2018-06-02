#ifndef MyGetMap_
#define MyGetMap_

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include <vector>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "angles/angles.h"

#include "holyWatcher.h" //TODO: position typ auf robotPose ändern!!!!


struct gridCell{
    int row;
    int col;
};


// struct robotPose{
//     double x;
//     double y;
//     double yaw;
// };



// der Name requestMap sollte anders heißten -> getOccupancyGridMap
// ich brauche nav_msgs::OccupancyGrid damit ich grid2Kartesisch benutzen kann!!!
// der ziwschenschritt von requestMap zu requestMap ist also notwendig in meiner Welt
nav_msgs::OccupancyGrid requestMap(ros::NodeHandle &nh);

std::vector<std::vector<int> > readMap(const nav_msgs::OccupancyGrid& map);
//TODO: jakob fragen: ich will OccupancyGrid& map nicht zwei mal übergeben. wie anders machen?
geometry_msgs::Point grid2Kartesisch(const nav_msgs::OccupancyGrid& map, int row, int col);
gridCell kartesisch2grid(const nav_msgs::OccupancyGrid& map, double x, double y);
robotPose getRobotPos(/*ros::NodeHandle &nh*/);
// Costmap von Move_base auslesen. Damit keine Ziele nah an dem Gegenständen
// gewählt werden
std::vector<std::vector<int> > getCostmap();
std::vector<std::vector<int> > getGridMap();
nav_msgs::OccupancyGrid getGrid();


// void OdomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
// void costmapCallback(const nav_msgs::OccupancyGrid& costmap);



#endif
