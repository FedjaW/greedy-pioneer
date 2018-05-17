#include "myGetMap.h"


int main (int argc, char **argv) {
    ros::init(argc, argv, "myGetMap");
    ros::NodeHandle nh;
    ROS_INFO("Testpunnkt 1");

    MYGETMAP myMap;

    nav_msgs::OccupancyGrid grid = myMap.requestMap(nh);
    std::vector<std::vector<int> > gridMap = myMap.readMap(grid);

    ROS_INFO("gridMap[5][5] = %d", gridMap[5][5]);
    ros::spinOnce();

    return 0;
}
