#ifndef MYGETMAP_
#define MYGETMAP_

#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include <vector>
#include <geometry_msgs/Point.h>


struct gridCell{

        int row;
        int col;
};


class MYGETMAP{

    public:

        MYGETMAP();

        nav_msgs::OccupancyGrid requestMap(ros::NodeHandle &nh);

        std::vector<std::vector<int> > readMap(const nav_msgs::OccupancyGrid& map);

        //TODO: jakob fragen: ich will OccupancyGrid& map nicht zwei mal übergeben. wie anders machen?
        geometry_msgs::Point grid2Kartesisch(const nav_msgs::OccupancyGrid& map, int row, int col);

        gridCell kartesisch2grid(const nav_msgs::OccupancyGrid& map, double x, double y);
};



#endif
