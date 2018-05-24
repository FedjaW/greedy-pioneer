#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "tf/transform_broadcaster.h"
#include "angles/angles.h"
#include "math.h"

const double PI = 3.1415;

void setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos);
double getDistance(double x1,double y1,double x2,double y2);
void rotate(ros::NodeHandle &nh, double goal_pos_x, double goal_pos_y, double tolerance);
void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_message);
double radiand2degrees(double angle_in_radiand);
double degrees2rad(double angle_in_degrees);


double yaw_global;
double jackal_position_x;
double jackal_position_y;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_plan_service");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped Start;
    Start.header.seq = 0;
    Start.header.stamp = ros::Time(0);
    Start.header.frame_id = "/map";
    Start.pose.position.x = 0;
    Start.pose.position.y = 0;
    Start.pose.position.z = 0.0;
    Start.pose.orientation.x = 0.0;
    Start.pose.orientation.y = 0.0;
    Start.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped Goal;
    Goal.header.seq = 0;
    Goal.header.stamp = ros::Time(0);
    Goal.header.frame_id = "/map";
    Goal.pose.position.x = 7;
    Goal.pose.position.y = 0;
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
    ROS_INFO("Plan size: %d", srv.response.plan.poses.size());


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
    setMarkerArray(nh, vizPos);

    rotate(nh, 5, 5, 0.05);
}




void setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos){
    
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
    
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
    m.scale.x = 0.02;
    m.scale.y = 0.02;
    m.scale.z = 0.02;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 1;
    m.color.a = 1;
    // m.lifetime = ros::Duration(0);
    // m.frame_locked = true;
    int id = 0;
    for(int i = 0; i < vizPos.size(); i++){
        m.pose.position.x = vizPos[i].position.x;
        m.pose.position.y = vizPos[i].position.y;
        m.id = id;
        markers.push_back(m);
        ++id;
    }
    while (marker_array_publisher.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.2);
    }
    marker_array_publisher.publish(markers_msg);
    ROS_INFO("MarkerArray wurde gesetzt");
}

//__________________________________DISTANCE
double getDistance(double x1,double y1,double x2,double y2){
        return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}


//___________________________________ROTATE
void rotate(ros::NodeHandle &nh, double goal_pos_x, double goal_pos_y, double tolerance){

    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);  
    ros::Subscriber pose_subscriber = nh.subscribe("/odometry/filtered" , 5 , poseCallback);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    double steering_angle;
    int steering_angle_normalized_deg;
    double steering_angle_normalized_rad;
    double angle_difference;

    do{
        steering_angle = atan2(goal_pos_y - jackal_position_y, goal_pos_x - jackal_position_x);
        steering_angle_normalized_deg = ((int) radiand2degrees(steering_angle)+360)%360;
        steering_angle_normalized_rad = degrees2rad(steering_angle_normalized_deg);
        angle_difference = steering_angle_normalized_rad - yaw_global;

        // ROS_INFO("steering_angle %f", steering_angle);
        // ROS_INFO("yaw_global %f", yaw_global);
        vel_msg.angular.z = angle_difference; 
        ROS_INFO("angular.z = %f", vel_msg.angular.z);
        ROS_INFO("angle diff = %f", angle_difference);
        if(vel_msg.angular.z > 2) vel_msg.angular.z = 2;
        if(vel_msg.angular.z < -2) vel_msg.angular.z = -2;

        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }while(fabs(angle_difference) > tolerance);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Rotation done");
}


void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_message){

        double roll, pitch, yaw;

        // this will initiate the quaternion variable which contain x,y,z and w values;
        tf::Quaternion quater;
        // this will cnvert quaternion msg to quaternion
        tf::quaternionMsgToTF(pose_message->pose.pose.orientation, quater);
        // this will get the quaternion matrix and represent it in Euler angle
        tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
        // this normalize the angle between 0 and 2*PI
        yaw_global = angles::normalize_angle_positive(yaw);

        jackal_position_x = pose_message->pose.pose.position.x;
        jackal_position_y = pose_message->pose.pose.position.y;

}


double radiand2degrees(double angle_in_radiand){

        return angle_in_radiand * 180 / PI;
}

double degrees2rad(double angle_in_degrees){

        return angle_in_degrees * PI/180;
}
