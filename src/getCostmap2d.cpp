#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include <vector>
#include <fstream>
using namespace std;

nav_msgs::OccupancyGrid cur_grid;
bool map_set = false;

void printGridToFile();
// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid_vec;

void saveMap(const nav_msgs::OccupancyGrid& grid)
{
    cur_grid = grid;
    map_set = true;
    ROS_INFO("map_set");


    ROS_INFO("Received a %d X %d Map @ %.3f m/px\n", 
            cur_grid.info.width,
            cur_grid.info.height,
            cur_grid.info.resolution);

    rows = cur_grid.info.height;
    cols = cur_grid.info.width;
    mapResolution = cur_grid.info.resolution;

    // Dynamically resize the Grid
    grid_vec.resize(rows);
    for (int i = 0; i < rows ; i++){
        grid_vec[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++){
        for(int j = 0; j< cols; j++){
            grid_vec[i][j] = cur_grid.data[currCell];
            currCell++;
        }
    }
    printGridToFile();
}




void printGridToFile() {
   ofstream gridFile;
   gridFile.open("grid.txt");

   for(int i = grid_vec.size() - 1; i>= 0 ; i--){
         for(int j = 0; j < grid_vec[0].size(); j++){
            // gridFile << (grid[i][j] ? "1" : "0");
                 gridFile << grid_vec[i][j];
         }
         gridFile << endl;
   }
   
   gridFile.close();
   ROS_INFO("Habe das File gespeichert und geschlossen\n");
}


int main (int argc, char **argv) {
	ros::init(argc, argv, "getCostmap2d");
	ros::NodeHandle nh;
	ROS_INFO("Testpunnkt 1");
	ros::Subscriber sub = nh.subscribe("/move_base/global_costmap/costmap", 10, saveMap);
	ros::spin();
  
	return 0;
}

