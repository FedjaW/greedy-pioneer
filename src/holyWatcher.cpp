#include "holyWatcher.h"
#include <iostream>
#include "angles/angles.h"
#include "tf/transform_broadcaster.h"
#include <ros/ros.h>
#include <fstream>

/* struct position {
    float x;
    float y;
};*/


robotPose roboterPosition;
std::vector<std::vector<int> > costmap;
// std::vector<std::vector<int> > costmap_upd;
std::vector<std::vector<int> > gridMap;
nav_msgs::OccupancyGrid grid;

void updateGridMap(const nav_msgs::OccupancyGrid& map){
    // ROS_INFO("Received a %d X %d GridMap @ %.3f m/px", 
    //         map.info.width,
    //         map.info.height,
    //         map.info.resolution);

    int rows = map.info.height;
    int cols = map.info.width;
    double mapResolution = map.info.resolution;

    std::vector<std::vector<int> > grid_vec;

    // Dynamically resize the Grid
    grid_vec.resize(rows);
    for (int i = 0; i < rows ; i++){
        grid_vec[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            grid_vec[i][j] = map.data[currCell];
            currCell++;
        }
    }
    grid = map;
    gridMap = grid_vec;


    calculateExploratedAreaOverTime(map);

}

void calculateExploratedAreaOverTime(const nav_msgs::OccupancyGrid& map) {
    int cellsInMap = map.info.width * map.info.height;
    int exploredCells = 0;
    for(int i = 0; i < cellsInMap; i++) {
        if(map.data[i] == 0) {
            exploredCells++;
        }
    }
    double area = exploredCells * map.info.resolution * map.info.resolution;
    static double initial_time = ros::Time::now().toSec();
    double time = ros::Time::now().toSec() - initial_time;
    std::cout << "area = "<< area << " / time = " << time << std::endl;
    printToFile(area, time);
}


void printToFile(double area, double time) {
    std::ofstream myFile;
    if(!myFile.is_open())
        myFile.open("AreaOvertime.txt", std::ios::app);
    myFile << area << " , "<< time << std::endl;;
}



void updateCostmap(const nav_msgs::OccupancyGrid& costmap_msg) {
    // ROS_INFO("Received a %d X %d Costmap @ %.3f m/px", 
    //         costmap_msg.info.width,
    //         costmap_msg.info.height,
    //         costmap_msg.info.resolution);

    int rows_cost = costmap_msg.info.height;
    int cols_cost = costmap_msg.info.width;
    double mapResolution_cost = costmap_msg.info.resolution;

    std::vector<std::vector<int> > costmap_grid_vec;
    // Dynamically resize the Grid
    costmap_grid_vec.resize(rows_cost);
    for (int i = 0; i < rows_cost ; i++){
        costmap_grid_vec[i].resize(cols_cost);
    }

    int currCell = 0;
    for (int i = 0; i < rows_cost; i++){
        for(int j = 0; j < cols_cost; j++){
            costmap_grid_vec[i][j] = costmap_msg.data[currCell];
            currCell++;
        }
    }
    costmap = costmap_grid_vec;

}

// void update_callback(const map_msgs::OccupancyGridUpdate& costmapUpdate) {
//     ROS_INFO("Received a %d X %d UPDATECOSTMAP", 
//             costmapUpdate.width,
//             costmapUpdate.height);
//
//     int rows_cost = costmapUpdate.height;
//     int cols_cost = costmapUpdate.width;
//
//     std::vector<std::vector<int> > costmap_update_vec;
//     // Dynamically resize the Grid
//     costmap_update_vec.resize(rows_cost);
//     for (int i = 0; i < rows_cost ; i++){
//         costmap_update_vec[i].resize(cols_cost);
//     }
//
//     int currCell = 0;
//     for (int i = 0; i < rows_cost; i++){
//         for(int j = 0; j < cols_cost; j++){
//             costmap_update_vec[i][j] = costmapUpdate.data[currCell];
//             currCell++;
//         }
//     }
//     costmap_upd = costmap_update_vec;
//
// }
//

void updateRoboterPosition(const nav_msgs::Odometry::ConstPtr& pose_msg) {

    double roll, pitch, yaw;
    // this will initiate the quaternion variable which contain x,y,z and w values;
    tf::Quaternion quater;
    // this will cnvert quaternion msg to quaternion
    tf::quaternionMsgToTF(pose_msg->pose.pose.orientation, quater);
    // this will get the quaternion matrix and represent it in Euler angle
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
    // this normalize the angle between 0 and 2*PI
    roboterPosition.yaw = angles::normalize_angle_positive(yaw);

    roboterPosition.x = pose_msg->pose.pose.position.x;
    roboterPosition.y = pose_msg->pose.pose.position.y;

    calculateDistanceTraveled();
}




void calculateDistanceTraveled() {
    static robotPose oldRoboterPosition = roboterPosition;
    static double distanceTraveled = 0;
    double newDistance = sqrt(pow(roboterPosition.x - oldRoboterPosition.x,2)+
                              pow(roboterPosition.y - oldRoboterPosition.y,2));
    if(newDistance > 0.01) // die if Abfrage dient um das Gitter zu filtern! 
        distanceTraveled = distanceTraveled + newDistance;
    oldRoboterPosition = roboterPosition;
    static double initial_time2 = ros::Time::now().toSec();
    double time = ros::Time::now().toSec() - initial_time2;
    std::cout << "distanceTraveled = "<< distanceTraveled << " / time = " << time << std::endl;
}



void startPositionWatcher() {

    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("odometry/filtered", 
                                                        10, 
                                                        updateRoboterPosition);

    ros::Subscriber sub = nodeHandle.subscribe("/move_base/global_costmap/costmap",
                                                10, 
                                                updateCostmap); // costmap

// ros::Subscriber costmap_update_sub = nodeHandle.subscribe("move_base/global_costmap/costmap_updates", 10, update_callback);

    ros::Subscriber sub2 = nodeHandle.subscribe("/map",
                                                10, 
                                                updateGridMap);

    ros::spin();
}
