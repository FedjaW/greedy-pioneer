#ifndef holyWatcher_
#define holyWatcher_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "nav_msgs/GetMap.h"
#include <map_msgs/OccupancyGridUpdate.h>

// struct position {
//     float x;
//     float y;
// };
struct robotPose{
    double x;
    double y;
    double yaw;
};
// ohne extern geht hier nix!!!! compiler sagt dann multiple definition of roboterPosition
extern robotPose roboterPosition;
extern std::vector<std::vector<int> > costmap;
extern std::vector<std::vector<int> > gridMap;
extern nav_msgs::OccupancyGrid grid;
// extern std::vector<std::vector<int> > costmap_upd;
 //position roboterPosition;
//position getCurrentPos();
void updateRoboterPosition(const nav_msgs::Odometry::ConstPtr& position);
void updateCostmap(const nav_msgs::OccupancyGrid& costmap_msg);
void updateGridMap(const nav_msgs::OccupancyGrid& map);
// void update_callback(const map_msgs::OccupancyGridUpdate& costmapUpdate);
void startPositionWatcher();
void calculateExploratedAreaOverTime(const nav_msgs::OccupancyGrid& map);
void printToFile(double y, double x, std::string str);
void calculateDistanceTraveled();
void calculateAngleRotated();

void everySecond();


#endif
