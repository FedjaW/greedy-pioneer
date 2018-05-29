#ifndef holyWatcher_
#define holyWatcher_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "nav_msgs/GetMap.h"


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
extern std::vector<std::vector<int> > costmap_grid_vec;

 //position roboterPosition;
//position getCurrentPos();
void updateRoboterPosition(const nav_msgs::Odometry::ConstPtr& position);
void updateCostmap(const nav_msgs::OccupancyGrid& costmap_msg);
void startPositionWatcher();



#endif
