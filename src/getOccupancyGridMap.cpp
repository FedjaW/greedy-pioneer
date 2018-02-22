#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>

using namespace std;


// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGridToFile();


int main (int argc, char** argv){

  ros::init(argc, argv, "getOccupancyGridMap"); 
  ros::NodeHandle nh;

  if (!requestMap(nh))
    exit(-1);

  printGridToFile();

  return 0;
}



bool requestMap(ros::NodeHandle &nh){

  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){
    ROS_INFO("Wating for service static map to become available\n");
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
      // if(map.data[currCell] == 0)
      //   grid[i][j] = false;
      // else
      //   grid[i][j] = true;
      currCell++;
    }
  }
}

void printGridToFile(){
    ROS_INFO("Testmarke 1\n");
  ofstream gridFile;
  gridFile.open("grid.txt");

  for(int i = grid.size() - 1; i>= 0 ; i--){
        for(int j = 0; j < grid[0].size(); j++){
           // gridFile << (grid[i][j] ? "1" : "0");
           gridFile << grid[i][j];
        }
        gridFile << endl;
  }
  gridFile.close();

    ROS_INFO("Habe das File gespeichert und geschlossen\n");
}

