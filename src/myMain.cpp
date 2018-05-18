#include "myGetMap.h"
// #include <geometry_msgs/Point.h>


int main (int argc, char **argv) {
    ros::init(argc, argv, "myGetMap");
    ros::NodeHandle nh;
     // ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);

     // ros::spinOnce();
     // ros::Rate rate(10);
     // rate.sleep();
     // ros::spinOnce();

    ROS_INFO("Testpunnkt 1");

    MYGETMAP myMap;

    nav_msgs::OccupancyGrid grid = myMap.requestMap(nh);
    std::vector<std::vector<int> > gridMap = myMap.readMap(grid);

    // ROS_INFO("gridMap[5][5] = %d", gridMap[5][5]);
    
    geometry_msgs::Point myPoint = myMap.grid2Kartesisch(grid,0,0);

    // ROS_INFO("Punkt x = %f", myPoint.x);
    // ROS_INFO("Punkt y = %f", myPoint.y);
    
    gridCell myCell =  myMap.kartesisch2grid(grid,-10,-10);

    // ROS_INFO("cell row = %d", myCell.row);
    // ROS_INFO("cell col = %d", myCell.col);
    
    robotPose myRobot;
    myRobot = myMap.getRobotPos(nh);

    ROS_INFO("myRobot.x = %f", myRobot.x);
    ROS_INFO("myRobot.y = %f", myRobot.y);
    ROS_INFO("myRobot.yaw = %f", myRobot.yaw);



    // ros::spin();

    return 0;
}
