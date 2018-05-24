#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "visualize.h"

#include "move_service.h"
#include "myGetMap.h"


#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>

// tf::TransformListener * starListener() {
//     tf::TransformListener listener;
//     return &listener
// }
//
tf::StampedTransform getRobotPosInMapFrame() {

    static tf::TransformListener listener;
    static tf::StampedTransform robotPosInMapFrame;

    try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }


    double x_ = 0;
    double y_ = 0;
   // Das listener Objekt muss frühzeitig erzeugt worden sein !! sonst catch error 
    try {
        listener.lookupTransform("map", "base_link", ros::Time(0), robotPosInMapFrame);
        x_ = robotPosInMapFrame.getOrigin().x();
        y_ = robotPosInMapFrame.getOrigin().y();
    } 
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

    std::cout << "Roboter Position x_ = " << x_ << std::endl;
    std::cout << "Roboter Position y_ = " << y_ << std::endl;

    return robotPosInMapFrame;
}




// void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate) {
void getDistanceToFrontier(ros::NodeHandle &nh, geometry_msgs::Point goalCanditate, double start_x, double start_y) {
    geometry_msgs::PoseStamped Start;
    Start.header.seq = 0;
    Start.header.stamp = ros::Time(0);
    Start.header.frame_id = "/map";
    // Start.pose.position.x = getRobotPos().x; // ich brauche die roboterPosition im 
    // Start.pose.position.y = getRobotPos().y; // MAP frame und nicht im odom frame
    Start.pose.position.x = start_x;  
    Start.pose.position.y = start_y; 
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
    ROS_INFO("Little reminder: check if move_base is turned on");
    ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

    Visualizer myVisualizer2;
    int id = 0;
    std::vector<geometry_msgs::Pose> vizPos;
    double distance;
    float myX_old, myY_old = 0;
    int path_size = srv.response.plan.poses.size();
    geometry_msgs::PoseStamped myPose;
    for(int i = 0; i < path_size-1; i++){
        myPose = srv.response.plan.poses[i];
        float myX = myPose.pose.position.x;
        float myY = myPose.pose.position.y;
        // ROS_INFO("x = %f, y = %f", myX, myY);
        if(i == 0){
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



}




//__________________________________DISTANCE
double getDistance(double x1,double y1,double x2,double y2){
    return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}
