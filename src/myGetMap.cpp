#include "myGetMap.h"
#include <ros/ros.h>

//_____________________default constructor
MYGETMAP::MYGETMAP(){};

    robotPose myRobot;

//_________________________requestMap
nav_msgs::OccupancyGrid MYGETMAP::requestMap(ros::NodeHandle &nh){

    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    // dynamic map ist a service provided by gmapping: dynamic_map(mav_msgs/GetMap)
    while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){ 
        ROS_INFO("Wating for service dynamic map to become available\n");
    }

    ROS_INFO("Requesting the map..\n");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

    if(mapClient.call(req, res)){
        // readMap(res.map, nh);
        // ROS_INFO("res map orin = %.3f\n",res.map.info.origin.position.x);
        // return true;
        return res.map;
    }
    else{
        ROS_ERROR("Failed to call map service\n");
    }
}

//_______________________________readMap
std::vector<std::vector<int> > MYGETMAP::readMap(const nav_msgs::OccupancyGrid& map){
    ROS_INFO("Received a %d X %d Map @ %.3f m/px", 
            map.info.width,
            map.info.height,
            map.info.resolution);

    // ROS_INFO("Origin.Position x= %.3f", map.info.origin.position.x);
    // ROS_INFO("Origin.Position y= %.3f", map.info.origin.position.y);
    int rows = map.info.height;
    int cols = map.info.width;
    // ROS_INFO("map.info.height = %d", map.info.height);
    // ROS_INFO("map.info.width = %d", map.info.width);
    double mapResolution = map.info.resolution;

    std::vector<std::vector<int> > grid;

    // Dynamically resize the Grid
    grid.resize(rows);
    for (int i = 0; i < rows ; i++){
        grid[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++){
        for(int j = 0; j< cols; j++){
            grid[i][j] = map.data[currCell];
            currCell++;
        }
    }
    return grid;
}


// TODO: Jakob fragen wegen dem Ausdruck: ist geometry_msgs::Point ein namespace? woran erkenne ich was es genau ist? könnte ja auch ein struct sein weil Point ein datentyp ist?! oder ist point eine klasse?
//___________________ von grid zu kartesischen Koordianten 
geometry_msgs::Point MYGETMAP::grid2Kartesisch(const nav_msgs::OccupancyGrid& map, int row, int col){

    geometry_msgs::Point point;

    point.x = row * map.info.resolution + map.info.origin.position.x;
    point.y = col * map.info.resolution + map.info.origin.position.y;

    return point;
}

//___________________ von kartesischen Koordianten zu grid
gridCell MYGETMAP::kartesisch2grid(const nav_msgs::OccupancyGrid& map, double x, double y){

    gridCell cell;
    
    cell.row = (unsigned int) ((x - map.info.origin.position.x)/map.info.resolution);
    cell.col = (unsigned int) ((y - map.info.origin.position.y)/map.info.resolution);

    return cell;
}




robotPose MYGETMAP::getRobotPos(ros::NodeHandle &nh){
    
    ROS_INFO("getRobotPos Funktionsaufruf");
    ros::Subscriber sub = nh.subscribe("odometry/filtered",1, OdomCallback);
    
    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();

    ROS_INFO("getRobotPos Funktionsende");
    return myRobot;
}



// Read out the odometry _________________________________________________________
void OdomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){

    ROS_INFO("OdomCallback aufruf");

    double roll, pitch, yaw;
    // this will initiate the quaternion variable which contain x,y,z and w values;
    tf::Quaternion quater;
    // this will cnvert quaternion msg to quaternion
    tf::quaternionMsgToTF(pose_msg->pose.pose.orientation, quater);
    // this will get the quaternion matrix and represent it in Euler angle
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
    // this normalize the angle between 0 and 2*PI
    myRobot.yaw = angles::normalize_angle_positive(yaw);

    myRobot.x = pose_msg->pose.pose.position.x;
    myRobot.y = pose_msg->pose.pose.position.y;
    // ROS_INFO("x: %f, y: %f",map_frame_x ,map_frame_y );
    ROS_INFO("odom callback");
}
























