#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <math.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
    
using namespace std;

 int grid_cell_x[9050];
 int grid_cell_y[9050];
        

int rows_cost;
int cols_cost;
double mapResolution_cost;
nav_msgs::OccupancyGrid cur_grid;
bool map_set = false;
void saveMap(const nav_msgs::OccupancyGrid& grid);
vector<vector<int> > grid_vec;

void extractFrontierRegion();
unsigned int startFrontier[100];
unsigned int endFrontier[100];
unsigned int nr = 0;
unsigned int bigFrontier[20];
unsigned int n = 0;

// void setMarker(visualization_msgs::Marker& marker, ros::NodeHandle &nh);
void setMarker(ros::NodeHandle &nh, double mark_x, double mark_y, int id);
// visualization_msgs::Marker marker_global;

bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout);
// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

double map_frame_x;
double map_frame_y;

void findFrontiers();

int x_such_min;
int x_such_max;
int y_such_min;
int y_such_max;
unsigned int global_k;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg, ros::NodeHandle &nh);

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

//__________________________________________________________________________________________main
int main (int argc, char** argv){

    ros::init(argc, argv, "getOccupancyGridMap"); 
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);
	ros::Subscriber sub2 = nh.subscribe("/move_base/global_costmap/costmap", 10, saveMap); // costmap
    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();

    if (!requestMap(nh))
        exit(-1);
    
    return 0;
}


void setMarker(ros::NodeHandle &nh, double mark_x, double  mark_y, int id){
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type.  
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = mark_x;
    marker.pose.position.y = mark_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

// if (id >= 7) marker.color.r = 0;
if (id % 2 == 0) marker.color.r = 0;
else marker.color.r = 1;

    marker.lifetime = ros::Duration();

    ros::Duration timeout = ros::Duration(3);
    if(waitForSubscribers(marker_pub, timeout)){
        marker_pub.publish(marker);
        ROS_INFO("habe marker %d gesetzt", id);
    }
    else
        ROS_INFO("TIMEOUT");
}




bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout)
{
    if(pub.getNumSubscribers() > 0)
        return true;
    ros::Time start = ros::Time::now();
    ros::Rate waitTime(0.5);
    while(ros::Time::now() - start < timeout) {
        waitTime.sleep();
        if(pub.getNumSubscribers() > 0)
            break;
    }
    return pub.getNumSubscribers() > 0;
}





void saveMap(const nav_msgs::OccupancyGrid& grid)
{
cur_grid = grid;
map_set = true;


ROS_INFO("Received a %d X %d Costmap @ %.3f m/px\n", 
                                                    cur_grid.info.width,
                                                    cur_grid.info.height,
                                                    cur_grid.info.resolution);

    rows_cost = cur_grid.info.height;
    cols_cost = cur_grid.info.width;
    mapResolution_cost = cur_grid.info.resolution;

   // Dynamically resize the Grid
    grid_vec.resize(rows_cost);
    for (int i = 0; i < rows_cost ; i++){
        grid_vec[i].resize(cols_cost);
    }

    int currCell = 0;
    for (int i = 0; i < rows_cost; i++){
        for(int j = 0; j < cols_cost; j++){
            grid_vec[i][j] = cur_grid.data[currCell];
            currCell++;
        }
    }


ROS_INFO("costmap map_set");
}




bool requestMap(ros::NodeHandle &nh){

  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){ // dynamic map ist a service provided by gmapping: dynamic_map(mav_msgs/GetMap)
    ROS_INFO("Wating for service dynamic map to become available\n");
  }

  ROS_INFO("Requesting the map..\n");
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
  
  if(mapClient.call(req, res)){
    readMap(res.map, nh);
    // ROS_INFO("res map orin = %.3f\n",res.map.info.origin.position.x);
    return true;
  }
  else{
    ROS_ERROR("Failed to call map service\n");
    return false;
  }
}


void readMap(const nav_msgs::OccupancyGrid& map, ros::NodeHandle &nh){
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


    double ziel_map_frame_x = 991 * map.info.resolution + map.info.origin.position.x;
    double ziel_map_frame_y = 991 * map.info.resolution + map.info.origin.position.y;
                 double mark_pos_x = ziel_map_frame_x;
                 double mark_pos_y = ziel_map_frame_y;
                 int id = 0;
                 setMarker(nh, mark_pos_x, mark_pos_y, id);

  ziel_map_frame_x = 0 * map.info.resolution + map.info.origin.position.x;
  ziel_map_frame_y = 991 * map.info.resolution + map.info.origin.position.y;
                  mark_pos_x = ziel_map_frame_x;
                  mark_pos_y = ziel_map_frame_y;
                  id = 1;
                 setMarker(nh, mark_pos_x, mark_pos_y, id);
                
                 // mark_pos_x = map_frame_x;
                 // mark_pos_y = map_frame_y;
                 // id = 2;
                 // setMarker(nh, mark_pos_x, mark_pos_y, id);

// ________________________________SUCH REGION __________________________________________________________________
                 id = 2;
                 setMarker(nh, map_frame_x, map_frame_y, id); // map_frame ist die Position des Roboters

                 id = 3;
                 setMarker(nh, map_frame_x-5, map_frame_y-5, id);
                    grid_frame_x = (unsigned int) ((map_frame_x-5 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y-5 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_links_unten = %d \n", grid_frame_x); // x_such_min
                    ROS_INFO("grid_frame_y_links_unten = %d \n", grid_frame_y); // y_such_min
                    x_such_min = grid_frame_x; //x_such_min usw. sind alle Global; grid_frame_x nicht!
                    y_such_min = grid_frame_y;

                 id = 4;
                 setMarker(nh, map_frame_x-5, map_frame_y+5, id);
                    grid_frame_x = (unsigned int) ((map_frame_x-5 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y+5 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_links_oben = %d \n", grid_frame_x);
                    ROS_INFO("grid_frame_y_links_oben = %d \n", grid_frame_y); // y_such_max
                    y_such_max= grid_frame_y;

                 id = 5;
                 setMarker(nh, map_frame_x+5, map_frame_y+5, id);
                    grid_frame_x = (unsigned int) ((map_frame_x+5 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y+5 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_rechts_oben = %d \n", grid_frame_x);
                    ROS_INFO("grid_frame_y_rechts_oben = %d \n", grid_frame_y);

                 id = 6;
                 setMarker(nh, map_frame_x+5, map_frame_y-5, id);
                    grid_frame_x = (unsigned int) ((map_frame_x+5 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y-5 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_rechts_unten = %d \n", grid_frame_x); // x_such_max
                    ROS_INFO("grid_frame_y_rechts_unten = %d \n", grid_frame_y);
                    x_such_max= grid_frame_x;

// ________________________________SUCH REGION ENDE _________________________________________________________________

     // ziel_map_frame_x = grid_frame_x * map.info.resolution + map.info.origin.position.x;
     // ziel_map_frame_y = grid_frame_y * map.info.resolution + map.info.origin.position.y;
     //              mark_pos_x = ziel_map_frame_x;
     //              mark_pos_y = ziel_map_frame_y;
     //              id = 7;
     //             setMarker(nh, mark_pos_x, mark_pos_y, id);


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

 

    findFrontiers();
    // for(int i = 0; i < global_k; i = i + 10){
    //                 ziel_map_frame_x = grid_cell_x[i] * map.info.resolution + map.info.origin.position.x;
    //                 ziel_map_frame_y = grid_cell_y[i] * map.info.resolution + map.info.origin.position.y;
    //                 mark_pos_x = ziel_map_frame_x;
    //                 mark_pos_y = ziel_map_frame_y;
    //                 id = i+7;
    //                 setMarker(nh, mark_pos_x, mark_pos_y, id);
    // }

    extractFrontierRegion();
    id = 7;
    for(int i = 0; i < n; i++){
                    ziel_map_frame_x = grid_cell_x[startFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.x;
                    ziel_map_frame_y = grid_cell_y[startFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.y;
                    mark_pos_x = ziel_map_frame_x;
                    mark_pos_y = ziel_map_frame_y;
                    id++;
                    setMarker(nh, mark_pos_x, mark_pos_y, id);

                    ziel_map_frame_x = grid_cell_x[endFrontier[i]] * map.info.resolution + map.info.origin.position.x;
                    ziel_map_frame_y = grid_cell_y[endFrontier[i]] * map.info.resolution + map.info.origin.position.y;
                    mark_pos_x = ziel_map_frame_x;
                    mark_pos_y = ziel_map_frame_y;
                    id++;
                    setMarker(nh, mark_pos_x, mark_pos_y, id);
    }
}


// Read out the odometry _________________________________________________________
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        map_frame_x = msg->pose.pose.position.x;
        map_frame_y = msg->pose.pose.position.y;
         // ROS_INFO("x: %f, y: %f",map_frame_x ,map_frame_y );
        ROS_INFO("odom callback");
}


// Find frontiers________________________________________________________________
// Finde alle Frontiers nach der Neumann-Nachbarschaft (ToDo: auf Moore-Nachbarschaft ändern)
void findFrontiers(){
    unsigned int k = 0;
    // for(int i = grid.size() - 1; i>= 0 ; i--){
    //     for(int j = 0; j < grid[0].size(); j++){

    for(int i = y_such_max; i >= y_such_min; i--){
        for(int j = x_such_min; j < x_such_max; j++){
                if((grid[i][j] == 0) && (grid_vec[i][j] == 0)){ //TODO: was wenn es keine Costmap gibt?! -> dann auch kein grid_vec -> Abfrage machen
                        // grid_cell_x[k] = j;
                        // grid_cell_y[k] = i;
                        // k++;
                        // ROS_INFO("Free cell [%d]: x = %d",k, i);
                        // ROS_INFO("Free cell [%d]: y = %d",k, j);
                    if(grid[i+1][j] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier rechts");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i][j+1] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier oben");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i-1][j] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier links");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i][j-1] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier unten");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                }
            }
     }
global_k = k;
}


void extractFrontierRegion(){
    for(int i = 0; i < global_k-1; i++){
            // ROS_INFO("i = %d", i);
                // ROS_INFO("fronteri 166 = %u", grid_cell_x[166]);
                // ROS_INFO("fronteri 167 = %u", grid_cell_x[167]);
                // ROS_INFO("__difference_x_166-167 = %u", grid_cell_x[166] - grid_cell_x[167]);
                //  int diff;
                // diff = grid_cell_x[166] - grid_cell_x[167];
                // ROS_INFO("diff = %d", diff);
                // ROS_INFO("__difference_x = %u", grid_cell_x[i] - grid_cell_x[i+1]);
                // ROS_INFO("__difference_y= %u", grid_cell_y[i] - grid_cell_y[i+1]);
        if ( (abs(grid_cell_x[i] - grid_cell_x[i+1]) <= 20) && (abs(grid_cell_y[i] - grid_cell_y[i+1]) <= 20)){
            startFrontier[nr] = i;
            ROS_INFO("StartFrontier[%d] = %d",nr, startFrontier[nr]);
            do{
                i++; 
                // ROS_INFO("difference_x = %d", grid_cell_x[i] - grid_cell_x[i+1]);
                // ROS_INFO("difference_y= %d", grid_cell_y[i] - grid_cell_y[i+1]);
            }while((abs(grid_cell_x[i] - grid_cell_x[i+1]) <= 20) && (abs(grid_cell_y[i] - grid_cell_y[i+1]) <= 20));
            endFrontier[nr] = i;
            ROS_INFO("endFrontier[%d] = %d",nr, endFrontier[nr]);
            ROS_INFO("--------------");
            if (endFrontier[nr] - startFrontier[nr] >= 10){
                bigFrontier[n] = nr;
                n++;
            }
            nr++;
        }
    }
}
