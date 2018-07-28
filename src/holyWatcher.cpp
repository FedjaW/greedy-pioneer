#include "holyWatcher.h"
#include <iostream>
#include "angles/angles.h"
#include "tf/transform_broadcaster.h"
#include <ros/ros.h>
#include <fstream>
#include <time.h>
#include <chrono>

#include <thread>
#include <chrono>

/* struct position {
    float x;
    float y;
};*/
double zeiteinheit = 0.5;

bool print = true;

robotPose roboterPosition;
float roboterVelocity;
float roboterAngularVel;
std::vector<std::vector<int> > costmap;
// std::vector<std::vector<int> > costmap_upd;
std::vector<std::vector<int> > gridMap;
nav_msgs::OccupancyGrid grid;

double distanceTraveled = 0;
double angleRotated = 0;

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

// #if print
//     calculateExploratedAreaOverTime(map);
// #endif
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
    static float initial_time = ros::Time::now().toSec();
    // static float time;
    // time = ros::Time::now().toSec() - initial_time;
    static float time = - zeiteinheit;
    time = time + zeiteinheit;

    // std::cout << "area = "<< area << " / time = " << time << std::endl;
    // std::cout << "distanceTraveled = "<< distanceTraveled << " / time = " << time << std::endl;
    // std::cout << "angleRotated = "<< angleRotated << " / time = " << time << std::endl;
    printToFile(time, area, "AreaOverTime.txt");
    // printToFile(time, distanceTraveled, "DistanceTraveled.txt"); // ditanceTraveled ist global 
    //                                                              // damit ich es hier ins file speicher kann
    //                                                              // sonst zuviele aufrufe!
    // printToFile(time, angleRotated, "AngleRotated.txt");
    // printToFile(time, roboterPosition.yaw, "AbsolutAngle.txt");
    // printToFile(time, roboterVelocity, "RoboterVelocity.txt");
    // printToFile(time, roboterAngularVel, "RoboterAngularVel.txt");
    // std::cout << "distanceTraveled = "<< distanceTraveled << " / time = " << time << std::endl;
}


void printToFile(double x, double y, std::string str) {
    std::ofstream myFile;
    if(!myFile.is_open())
        myFile.open(str, std::ios::app);
    myFile << x << " , "<< y << std::endl;;
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

    roboterVelocity = pose_msg->twist.twist.linear.x;
    roboterAngularVel = pose_msg->twist.twist.angular.z;


    calculateDistanceTraveled();
    calculateAngleRotated();
}


void calculateAngleRotated() {
    static double old_yaw = fabs(roboterPosition.yaw);
    // double tolerance = 0;// 0.0170; // Entspricht 5°
    // if((roboterPosition.yaw > (old_yaw + tolerance)) || (roboterPosition.yaw < (old_yaw - tolerance))) {
    double diff = fabs(fabs(roboterPosition.yaw) - old_yaw);
    if(diff < 0.95*2*M_PI){
        angleRotated = angleRotated + diff;
    }
    else {
        diff = fabs(diff - 2*M_PI);
        angleRotated = angleRotated + diff;
    }
    old_yaw = fabs(roboterPosition.yaw);
}


void calculateDistanceTraveled() {
    static robotPose oldRoboterPosition = roboterPosition;
    double newDistance = sqrt(pow(roboterPosition.x - oldRoboterPosition.x,2)+
                              pow(roboterPosition.y - oldRoboterPosition.y,2));
    if(newDistance > 0.001) // "if" Abfrage dient um das Gitter (dschitter) zu filtern! 
        distanceTraveled = distanceTraveled + newDistance; // distanceTraveled ist global und wird weiter oben in ein file gespeichert
    oldRoboterPosition = roboterPosition;
    // static double initial_time2 = ros::Time::now().toSec();
    // double time = ros::Time::now().toSec() - initial_time2;
    // std::cout << "distanceTraveled = "<< distanceTraveled << " / time = " << time << std::endl;
    // printToFile(time, distanceTraveled, "DistanceTraveled.txt");
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



void everySecond() {

    sleep(1);
    while(1) {
        // sleep(zeiteinheit);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        calculateExploratedAreaOverTime(grid);
    }


    // int sec = 0;
    // float t1 = (float)clock()/CLOCKS_PER_SEC;
    // while(sec <= 60) {
    //     float t2 = (float)clock()/CLOCKS_PER_SEC;
    //
    //     while( t2 - t1 < 1 ) {
    //     // while( ( (((float)t2)/CLOCKS_PER_SEC) - (((float)t1)/CLOCKS_PER_SEC) ) < 2 ) {
    //         t2 = (float)clock()/CLOCKS_PER_SEC;
    //         // std::cout << "t2 - t1 = " << t2 - t1 << std::endl;
    //     }
    //     // sec++;
    //     t1 = (float)clock()/CLOCKS_PER_SEC;
    //     std::cout << sec++ << " Sekunden"<< std::endl;
    //     // calculateExploratedAreaOverTime(grid);
    //
    // }
    //     // std::cout << sec << " Sekunden"<< std::endl;
}
