#ifndef MyGetMap_
#define MyGetMap_

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include <vector>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "angles/angles.h"



struct gridCell{
    int row;
    int col;
};


struct robotPose{
    double x;
    double y;
    double yaw;
};


class MyGetMap{

    public:
        MyGetMap();
        nav_msgs::OccupancyGrid requestMap(ros::NodeHandle &nh);
        std::vector<std::vector<int> > readMap(const nav_msgs::OccupancyGrid& map);
        //TODO: jakob fragen: ich will OccupancyGrid& map nicht zwei mal übergeben. wie anders machen?
        geometry_msgs::Point grid2Kartesisch(const nav_msgs::OccupancyGrid& map, int row, int col);
        gridCell kartesisch2grid(const nav_msgs::OccupancyGrid& map, double x, double y);
        robotPose getRobotPos(ros::NodeHandle &nh);
        // Costmap von Move_base auslesen. Damit keine Ziele nah an dem Gegenständen
        // gewählt werden
        std::vector<std::vector<int> > getCostmap(ros::NodeHandle &nh);

    private:
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
        void costmapCallback(const nav_msgs::OccupancyGrid& costmap);
};



#endif
