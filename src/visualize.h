#ifndef VISUALIZE_
#define VISUALIZE_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>



class Visualize{
    public:
        Visualize();
        void setMarkerArray(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> vizPos);
        // void setMarker();
};

#endif
