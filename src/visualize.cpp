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
    
static ros::Publisher marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(
                                                                    "visualization_marker_array", 200);

        // sleep(0.2);
        // sleep(0.2);
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.type = visualization_msgs::Marker::SPHERE;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
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
    m.action = visualization_msgs::Marker::ADD;
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
        // ros::spinOnce();
        // ros::Rate rate(1);
        // rate.sleep();
        ROS_INFO("MarkerArray gesetzt");
    }

    if(deleteArray == 1) {
        // ROS_INFO("sleep");
        // rate.sleep();
        // rate.sleep();
        // rate.sleep();
        // rate.sleep();
        // ROS_INFO("sleepend");

        m.action = visualization_msgs::Marker::DELETE;
        for (int i = 0; i < id; i++) {
            m.id = i;
            markers.push_back(m);
        }
        marker_array_publisher.publish(markers_msg);
        id = 0;
        // ROS_INFO("sleep");
        ROS_INFO("Wait until all Markers are deleted");
        // ros::spinOnce();
        // ros::Rate rate(1);
        // rate.sleep();
        // ros::Rate rate(1);
        // rate.sleep();
        // rate.sleep();
        // rate.sleep();
        // rate.sleep();
        ROS_INFO("MarkerArray gelöscht");
        // ROS_INFO("sleepend");
    }
}


void Visualizer::deleteMarkerArray(ros::NodeHandle &nh) {

// static ros::Publisher marker_array_publisher2 = nh.advertise<visualization_msgs::MarkerArray>(
//                                                                     "visualization_marker_array", 200);
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;

    for (int i = 0; i < id; i++) {
        m.id = i;
        markers.push_back(m);
    }


    marker_array_publisher2.publish(markers_msg);
    ROS_INFO("MarkerArray gelöscht");

}
