#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

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
    Goal.pose.position.x = -5;
    Goal.pose.position.y = -1;
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


// ______________________________________visualization
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
    
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;


    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 1;
    m.color.a = 1;
    // m.lifetime = ros::Duration(0);
    // m.frame_locked = true;

    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;


    int id = 0;
    int path_size = srv.response.plan.poses.size();
    geometry_msgs::PoseStamped myPose;
    for(int i = 0; i < path_size-1; i++){
        myPose = srv.response.plan.poses[i];
        float myX = myPose.pose.position.x;
        float myY = myPose.pose.position.y;
        ROS_INFO("x = %f, y = %f", myX, myY);
        // setMarker(nh,myX,myY,id);
        //
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::SPHERE;
        m.pose.position.x = myX;
        m.pose.position.y = myY;
        m.id = id;
        markers.push_back(m);

        ++id;
    }

    // marker_array_publisher.publish(markers_msg);
    // ROS_INFO("MarkerArray wurde gesetzt");
    while (marker_array_publisher.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.1);
    }
    marker_array_publisher.publish(markers_msg);
    ROS_INFO("MarkerArray wurde gesetzt");
}

