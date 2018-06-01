#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "visualize.h"
#include "holyWatcher.h"
#include "move_service.h"
#include "myGetMap.h"

#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate) {
double getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate) {
    geometry_msgs::PoseStamped Start;
    Start.header.seq = 0;
    Start.header.stamp = ros::Time(0);
    Start.header.frame_id = "/map";
    // Start.pose.position.x = getRobotPos().x; // ich brauche die roboterPosition im 
    // Start.pose.position.y = getRobotPos().y; // MAP frame und nicht im odom frame
    Start.pose.position.x = getRobotPosInMapFrame().getOrigin().x();
    Start.pose.position.y = getRobotPosInMapFrame().getOrigin().y(); 
    Start.pose.position.z = 0.0;
    Start.pose.orientation.x = 0.0;
    Start.pose.orientation.y = 0.0;
    Start.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped Goal;
    Goal.header.seq = 0;
    Goal.header.stamp = ros::Time(0);
    Goal.header.frame_id = "/map";
    Goal.pose.position.x = goalCanditate.x;
    Goal.pose.position.y = goalCanditate.y;
    Goal.pose.position.z = 0.0;
    Goal.pose.orientation.x = 0.0;
    Goal.pose.orientation.y = 0.0;
    Goal.pose.orientation.w = 1.0;

    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = Start;
    srv.request.goal = Goal;
    srv.request.tolerance = 1.5;

    ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
    // ROS_INFO("Little reminder: check if move_base is turned on");
    ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

    Visualizer myVisualizer2;
    int id = 0;
    std::vector<geometry_msgs::Pose> vizPos;
    double distance = 0;
    float myX_old, myY_old = 0;
    int path_size = srv.response.plan.poses.size();
    geometry_msgs::PoseStamped myPose;
    for(int i = 0; i < path_size-1; i++) {
        myPose = srv.response.plan.poses[i];
        float myX = myPose.pose.position.x;
        float myY = myPose.pose.position.y;
        if(i == 0) {
            myX_old = myX;
            myY_old = myY;
        }
        distance = distance + getDistance(myX_old, myY_old, myX, myY);
        myX_old = myX;
        myY_old = myY;
        vizPos.push_back(myPose.pose);
    }

    ROS_INFO("Distanz = %f", distance);
    
    myVisualizer2.setMarkerArray(nh, vizPos, 0,1,1);

    id = vizPos.size()+1;
    vizPos.clear();
    
    return distance;

}


tf::StampedTransform getRobotPosInMapFrame() {
    static tf::TransformListener listener;
    static tf::StampedTransform robotPosInMapFrame;

    try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    try {
        listener.lookupTransform("map", "base_link", ros::Time(0), robotPosInMapFrame);
        return robotPosInMapFrame;
    } 
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

}


//__________________________________DISTANCE
double getDistance(double x1,double y1,double x2,double y2){
    return sqrt( pow((x2-x1),2) + pow((y2-y1),2) );
}



void rotate(ros::NodeHandle &nh, double rotation_angle) {
    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);  
    double angle_diff = 0;
    double robot_yaw = 0;
    double tolerance = 0.05;
    geometry_msgs::Twist vel_msg;

    robot_yaw = tf::getYaw(getRobotPosInMapFrame().getRotation());
    rotation_angle = rotation_angle + robot_yaw; // NOTE: Trick to calculate the steering_angle
    do{ 
        robot_yaw = tf::getYaw(getRobotPosInMapFrame().getRotation());
        angle_diff = rotation_angle - robot_yaw;
        vel_msg.angular.z = 2 * angle_diff; 
        if(vel_msg.angular.z > 1.8) vel_msg.angular.z = 1.8;
        if(vel_msg.angular.z < -1.8) vel_msg.angular.z = -1.8;

        velocity_publisher.publish(vel_msg);

    }while(fabs(angle_diff) > tolerance);

    // std::cout << "robot_yaw = " << robot_yaw << std::endl;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Rotation done :-)");
}

 

void sendGoal(double x, double y, double orientation) {
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orientation;

  ROS_INFO("Sending goal to navigation planner");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, we reached the goal :-)");
  else
    ROS_INFO("Dang, we didn't make it to the goal :-(");
}




bool isObstacleInViewField(ros::NodeHandle &nh, const nav_msgs::OccupancyGrid& map, int x0, int y0, int x1, int y1) {
    // Bresenham-Algorithm
    int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2; // error value e_xy 

    Visualizer myVisualizer3;
    int id = 0;
    std::vector<geometry_msgs::Pose> vizPos;
    geometry_msgs::PoseStamped myPose;


    while(1){
        // setPixel(x0,y0);
        
        myPose.pose.position.x = grid2Kartesisch(map,y0,x0).x;
        myPose.pose.position.y = grid2Kartesisch(map,y0,x0).y;
        vizPos.push_back(myPose.pose);

        if (x0==x1 && y0==y1) break;
        e2 = 2*err;
        if (e2 > dy) { err += dy; x0 += sx; } // e_xy+e_x > 0 
        if (e2 < dx) { err += dx; y0 += sy; } // e_xy+e_y < 0 

        if (costmap_grid_vec[x0][y0] > 100) { 
            myVisualizer3.setMarkerArray(nh, vizPos, 1,1,0);
            id = vizPos.size()+1;
            vizPos.clear();
            return true;

        }
    }

    myVisualizer3.setMarkerArray(nh, vizPos, 1,1,0);
    id = vizPos.size()+1;
    vizPos.clear();

    return false;
}
