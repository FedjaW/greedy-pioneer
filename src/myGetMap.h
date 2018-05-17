#ifndef MYGETMAP_
#define MYGETMAP_

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include <vector>



class myGetMap{


    public:
        myGetMap();

        nav_msgs::OccupancyGrid requestMap(ros::NodeHandle &nh);
        std::vector<std::vector<int> > readMap(const nav_msgs::OccupancyGrid& map);
    
        

};



#endif
