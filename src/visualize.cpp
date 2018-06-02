#include "visualize.h"
#include <visualization_msgs/MarkerArray.h>



Visualizer::Visualizer() {};
Visualizer::~Visualizer(){};
// Visualizer::Visualizer(double scale_x, double scale_y, double scale_z) {
//     double scale_x_ = scale_x_;
//     double scale_y_ = scale_y_;
//     double scale_z_ = scale_z_;
// }

int id = 0; // KEINE AHNUNG warum das mit der id so klappt ?!??!?!?

void Visualizer::setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos, int r, int g, int b){
    
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(
                                                                    "visualization_marker_array", 200);

    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;


    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
    // m.scale.x = scale_x_;
    // m.scale.y = scale_y_;
    // m.scale.z = scale_z_;
    m.scale.x = 0.04;
    m.scale.y = 0.04;
    m.scale.z = 0.04;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1;
    // m.lifetime = ros::Duration(0);
    // m.frame_locked = true;
    for(int i = 0; i < vizPos.size(); i++){
        m.pose.position.x = vizPos[i].position.x;
        m.pose.position.y = vizPos[i].position.y;
        m.id = id;
        markers.push_back(m);
        ++id;
        // ROS_INFO("id aufbau = %d ", id);
        // ros::Rate rate(100);
        // rate.sleep(); 
    }


    while (marker_array_publisher.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.2);
    }
    marker_array_publisher.publish(markers_msg);
    ROS_INFO("MarkerArray wurde gesetzt");
}



void Visualizer::delteMarkerArray(ros::NodeHandle &nh) {
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(
                                                                    "visualization_marker_array", 200);

    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    for (; id > 0; --id) {
        ROS_INFO("id abbau = %d ", id);
        // ros::Rate rate(50);
        // rate.sleep(); 
        m.id = id;
        markers.push_back(m);
    }
    while (marker_array_publisher.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.2);
    }
    marker_array_publisher.publish(markers_msg);
    ROS_INFO("MarkerArray wurde gesetzt");
}
