#include "visualize.h"
#include <visualization_msgs/MarkerArray.h>



Visualizer::Visualizer() {};
Visualizer::~Visualizer(){};
// Visualizer::Visualizer(double scale_x, double scale_y, double scale_z) {
//     double scale_x_ = scale_x_;
//     double scale_y_ = scale_y_;
//     double scale_z_ = scale_z_;
// }

int id; // KEINE AHNUNG warum das mit der id so klappt ?!??!?!?

void Visualizer::setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos, int r, int g, int b, bool deleteArray){
    
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(
                                                                    "visualization_marker_array", 200);

        sleep(0.2);
        sleep(0.2);
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
    if(deleteArray == 0) {

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
    ros::spinOnce();
    ros::Rate rate(1);
    rate.sleep();
    ROS_INFO("MarkerArray gesetzt");

    }

    if(deleteArray == 1) {

    ROS_INFO("sleep");
    ros::Rate rate(1);
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    ROS_INFO("sleepend");

    for (int i = 0; i < id; i++) {
        m.id = i;
        markers.push_back(m);
    }
    marker_array_publisher.publish(markers_msg);
    id = 0;
    ROS_INFO("MarkerArray gelöscht");
    ROS_INFO("sleep");
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    ROS_INFO("sleepend");
    }
}


void Visualizer::delteMarkerArray(ros::NodeHandle &nh) {
    ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(
                                                                    "visualization_marker_array", 200);

        sleep(0.2);
        sleep(0.2);
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("id_delete = %d ", id);
    for (int i = 0; i < id; i++) {
        m.id = i;
        markers.push_back(m);
    }

        ROS_INFO("id_delete2 = %d ", id);

    while (marker_array_publisher.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.2);
    }
    marker_array_publisher.publish(markers_msg);
    ROS_INFO("MarkerArray gelöscht");
    ros::spinOnce();
    ros::Rate rate(1);
    rate.sleep();

}
