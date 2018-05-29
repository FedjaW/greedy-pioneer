#include "holyWatcher.h"
#include <iostream>
#include "angles/angles.h"
#include "tf/transform_broadcaster.h"
#include <ros/ros.h>

/* struct position {
    float x;
    float y;
};*/


robotPose roboterPosition;
std::vector<std::vector<int> > costmap_grid_vec;

//position getCurrentPos() {
	// ROS_INFO("TESTPUNKT 23");
	//cout << roboterPosition.x << " / " << roboterPosition.y << endl;
	//return roboterPosition;
//}

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

}


void updateCostmap(const nav_msgs::OccupancyGrid& costmap_msg) {
    ROS_INFO("Received a %d X %d Costmap @ %.3f m/px\n", 
            costmap_msg.info.width,
            costmap_msg.info.height,
            costmap_msg.info.resolution);

    int rows_cost = costmap_msg.info.height;
    int cols_cost = costmap_msg.info.width;
    double mapResolution_cost = costmap_msg.info.resolution;

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

}

void startPositionWatcher() {

    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("odometry/filtered", 
                                                        10, 
                                                        updateRoboterPosition);

    ros::Subscriber sub = nodeHandle.subscribe("/move_base/global_costmap/costmap",
                                        10, 
                                        updateCostmap); // costmap

    ros::spin();
}
