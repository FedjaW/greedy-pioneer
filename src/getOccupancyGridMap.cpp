#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <nav_msgs/Odometry.h>

using namespace std;


// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

double map_frame_x;
double map_frame_y;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGridToFile();
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main (int argc, char** argv){

    ros::init(argc, argv, "getOccupancyGridMap"); 
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);
    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();

    if (!requestMap(nh))
        exit(-1);


    printGridToFile();
    return 0;
}



bool requestMap(ros::NodeHandle &nh){

  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){             // dynamic map ist a service provided by gmapping: dynamic_map(mav_msgs/GetMap)
    ROS_INFO("Wating for service dynamic map to become available\n");
  }

  ROS_INFO("Requesting the map..\n");
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

  if(mapClient.call(req, res)){
    readMap(res.map);
    return true;
  }
  else{
    ROS_ERROR("Failed to call map service\n");
    return false;
  }
}


void readMap(const nav_msgs::OccupancyGrid& map){
    ROS_INFO("Received a %d X %d Map @ %.3f m/px\n", 
                                                    map.info.width,
                                                    map.info.height,
                                                    map.info.resolution);
    
  ROS_INFO("Origin.Position x= %.3f \n", map.info.origin.position.x);
  ROS_INFO("Origin.Position y= %.3f \n", map.info.origin.position.y);

    
  ROS_INFO("map_frame_x = %f \n", map_frame_x);
  ROS_INFO("map_frame_y = %f \n", map_frame_y);
    int grid_frame_x = (unsigned int) ((map_frame_x - map.info.origin.position.x)/map.info.resolution);
    int grid_frame_y = (unsigned int) ((map_frame_y - map.info.origin.position.y)/map.info.resolution);
  ROS_INFO("grid_frame_x = %d \n", grid_frame_x);
  ROS_INFO("grid_frame_y = %d \n", grid_frame_y);

  rows = map.info.height;
  cols = map.info.width;
  mapResolution = map.info.resolution;


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
}

void printGridToFile(){
  ofstream gridFile;
  gridFile.open("grid.txt");

  for(int i = grid.size() - 1; i>= 0 ; i--){
        for(int j = 0; j < grid[0].size(); j++){
           // gridFile << (grid[i][j] ? "1" : "0");
            if(i == 148 && j == 455)
                gridFile << "R";
            else
                gridFile << grid[i][j];
        }
        gridFile << endl;
  }
  
  gridFile.close();
  ROS_INFO("Habe das File gespeichert und geschlossen\n");
}

// Read out the odometry _________________________________________________________
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        map_frame_x = msg->pose.pose.position.x;
        map_frame_y = msg->pose.pose.position.y;
         // ROS_INFO("x: %f, y: %f",map_frame_x ,map_frame_y );
        ROS_INFO("odom callback");
}
