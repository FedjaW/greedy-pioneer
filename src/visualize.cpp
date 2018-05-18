#include "visualize.h"
#include <visualization_msgs/MarkerArray.h>



Visualize::Visualize(){};

void Visualize::setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos){
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
